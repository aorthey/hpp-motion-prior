// Copyright (c) 2014 CNRS
// Author: Andreas Orthey
//
// This file is part of the hpp-motion-prior.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include "precomputation.impl.hh"
#include "precomputation-utils.hh"
#include "constraint-manifold-operator.hh"


namespace hpp
{
  namespace corbaserver
  {
    namespace motionprior
    {
      namespace impl
      {
        //Precomputation::Precomputation(corbaServer::Server* server) :
        //  server_(server), problemSolver_(server->problemSolver ())
        //{
        //}
        Precomputation::Precomputation () : problemSolver_ (0x0) {}
        void Precomputation::setProblemSolver
        (const ProblemSolverPtr_t& problemSolver)
        {
          problemSolver_ = problemSolver;
        }


        hpp::floatSeq* Precomputation::getConvexHullCapsules () throw (hpp::Error)
        {
          cvxCaps_.clear();
          computeProjectedConvexHullFromCurrentConfiguration ();
          return capsulePointsToFloatSeq(cvxCaps_);
        }

        void Precomputation::setCurrentConfiguration(const hpp::floatSeq &dofArray) throw (hpp::Error)
        {
          try {
            vector_t q = floatSeqToVector(dofArray);
            setCurrentConfiguration(q);
          } catch (const std::exception& exc) {
            throw hpp::Error (exc.what ());
          }
        }
        void Precomputation::setCurrentConfiguration(const Configuration_t& q) throw (hpp::Error)
        {
          try {
            DevicePtr_t robot = problemSolver_->robot ();
            std::size_t deviceDim = robot->configSize ();
            if(q.size() != deviceDim){
              hppDout (notice, "config dimension: " << q.size() <<",  deviceDim "<<deviceDim);
              throw hpp::Error ("dofVector Does not match");
            }
            robot->currentConfiguration (q);
            robot->computeForwardKinematics ();
          } catch (const std::exception& exc) {
            throw hpp::Error (exc.what ());
          }
        }

        Configuration_t Precomputation::step (const Configuration_t &qq, double lambda) throw (hpp::Error){
          try {
            Configuration_t q = problemSolver_->robot ()->currentConfiguration ();
            Configuration_t q_new; q_new.resize (qq.size()+1);
            //fill in CoM
            for(uint i=0; i<7; i++){
              q_new[i] = q[i];//- lambda*qq[i];
            }
            for(uint i=8; i<qq.size()+1; i++){
              q_new[i] = q[i] - lambda*qq[i-1];
            }
            return q_new;

          } catch (const std::exception& exc) {
            throw hpp::Error (exc.what ());
          }
        }

        hpp::floatSeq* Precomputation::projectUntilIrreducibleConstraint () throw (hpp::Error)
        {
          try {
            double epsilon = 0.001; //convergence threshold
            uint iterations = 0;
            double error = 1;
            double oldC = 10000;
            double lambda = 0.1; //update value
            computeProjectedConvexHullFromCurrentConfiguration ();

            ConstraintManifoldOperator Cop(problemSolver_);
            Cop.reset();
            Cop.init();

            std::vector<ProjectedCapsulePoint> cvxCapsOld;
            cvxCapsOld = cvxCaps_;

            while(error > epsilon){
              Configuration_t qq = this->getGradientVector();
              Configuration_t q = this->step(qq, lambda);

              Cop.apply(q);

              if(Cop.success()){
                //new configuration was successfully projected back on the given
                //contraint manifold
                this->setCurrentConfiguration(q);
                computeProjectedConvexHullFromCurrentConfiguration ();
                double C = this->getVolume();
                error = fabs(C-oldC);
                oldC = C;
                iterations++;
              }else{
                break;
              }
            }
            hppDout(notice, "projection onto irreducible manifold converged after " << iterations << " iterations." );

            DevicePtr_t robot = problemSolver_->robot ();
            Configuration_t q = robot->currentConfiguration();
            return vectorToFloatSeq(q);

          } catch (const std::exception& exc) {
            throw hpp::Error (exc.what ());
          }
        }



        hpp::floatSeq* Precomputation::projectUntilIrreducibleOneStep () throw (hpp::Error)
        {
          try {
            double lambda = 0.1; //update value
            computeProjectedConvexHullFromCurrentConfiguration ();

            Configuration_t qq = this->getGradientVector();
            vector_t q_new = this->step(qq, lambda);

            this->setCurrentConfiguration(q_new);
            computeProjectedConvexHullFromCurrentConfiguration ();

            DevicePtr_t robot = problemSolver_->robot ();
            vector_t q = robot->currentConfiguration();
            return vectorToFloatSeq(q);

          } catch (const std::exception& exc) {
            throw hpp::Error (exc.what ());
          }


        }
        hpp::floatSeq* Precomputation::projectUntilIrreducible () throw (hpp::Error)
        {
          try {
            double epsilon = 0.001; //convergence threshold
            uint iterations = 0;
            double error = 1;
            double oldC = 10000;
            double lambda = 0.1; //update value
            computeProjectedConvexHullFromCurrentConfiguration ();
            while(error > epsilon){
              Configuration_t qq = this->getGradientVector();
              Configuration_t q = this->step(qq, lambda);
              this->setCurrentConfiguration(q);
              computeProjectedConvexHullFromCurrentConfiguration ();
              double C = this->getVolume();
              error = fabs(C-oldC);
              oldC = C;
              iterations++;
            }
            hppDout(notice, "projection onto irreducible manifold converged after " << iterations << " iterations." );

            DevicePtr_t robot = problemSolver_->robot ();
            vector_t q = robot->currentConfiguration();
            return vectorToFloatSeq(q);

          } catch (const std::exception& exc) {
            throw hpp::Error (exc.what ());
          }
        }

        double Precomputation::getVolume () throw (hpp::Error)
        {
          //assume that hull are points on a convex hull, which are sorted
          //counter-clockwise.
          using namespace Eigen;
          Vector2f x1(cvxCaps_.at(0).y, cvxCaps_.at(0).z);
          double volume = 0;
          //compute volume by iterating over all embeded triangles
          for(int i=0; i<cvxCaps_.size()-2; i++){
            Vector2f x2(cvxCaps_.at(i+1).y, cvxCaps_.at(i+1).z);
            Vector2f x3(cvxCaps_.at(i+2).y, cvxCaps_.at(i+2).z);
            Vector2f v12 = x2-x1;
            Vector2f v13 = x3-x1;
            double b = v12.norm();
            Vector2f h_vec = v13 - v13.dot(v12)*v12;
            double h = h_vec.norm();
            double d = 0.5*b*h;
            volume += d;
          }
          return volume;
        }


        void Precomputation::computeProjectedConvexHullFromCurrentConfiguration () 
          throw (hpp::Error)
        {
          DevicePtr_t robot = problemSolver_->robot ();
          std::vector<CapsulePoint> caps = parseCapsulePoints(robot);
          std::vector<ProjectedCapsulePoint> projCaps = projectCapsulePointsOnYZPlane(caps);
          cvxCaps_.clear();
          cvxCaps_ = computeConvexHullFromProjectedCapsulePoints(projCaps);
        }

        Configuration_t Precomputation::getGradientVector(){
          //Compute gradient wrt outer jacobians
          DevicePtr_t robot = problemSolver_->robot ();
          //JointJacobian_t is Eigen::Matrix<double, 6, Eigen::Dynamic> 
          Configuration_t qgrad(robot->numberDof());
          qgrad.setZero();

          for(uint i=0; i<cvxCaps_.size(); i++){
            Configuration_t cvx_pt_eigen(6);
            cvx_pt_eigen << 0,cvxCaps_.at(i).y,cvxCaps_.at(i).z,0,0,0;
            Configuration_t qi = cvxCaps_.at(i).J.transpose()*cvx_pt_eigen;
            qgrad = qgrad + qi;
          }
          return qgrad;
        }
        hpp::floatSeq* Precomputation::getGradient() 
          throw (hpp::Error)
        {
          vector_t qgrad = getGradientVector();
          return vectorToFloatSeq(qgrad);
        }



        //-----------------------------------------------------------------------
        // TODO: replace function
        //-----------------------------------------------------------------------
        static ConfigurationPtr_t dofSeqToConfig
        (core::ProblemSolverPtr_t problemSolver, const hpp::floatSeq& dofArray)
        {
          unsigned int configDim = (unsigned int)dofArray.length();
          ConfigurationPtr_t config (new Configuration_t (configDim));

          // Get robot in hppPlanner object.
          DevicePtr_t robot = problemSolver->robot ();

          // Compare size of input array with number of degrees of freedom of
          // robot.
          if (configDim != robot->configSize ()) {
            hppDout (error, "robot configSize (" << robot->configSize ()
          	   << ") is different from config size ("
          	   << configDim << ")");
            throw std::runtime_error
              ("robot nb dof is different from config size");
          }

          // Fill dof vector with dof array.
          for (unsigned int iDof=0; iDof < configDim; ++iDof) {
            (*config) [iDof] = dofArray [iDof];
          }
          return config;
        }


        hpp::Names_t* Precomputation::addNaturalConstraints
                (const char* prefix, const hpp::floatSeq& dofArray,
                 const char* leftAnkle, const char* rightAnkle) throw (hpp::Error)
        {
          using core::DifferentiableFunctionPtr_t;
          using std::string;
          try {

            using namespace hpp::corbaServer::precomputation;
            ConfigurationPtr_t config = dofSeqToConfig (problemSolver_, dofArray);
            const DevicePtr_t& robot (problemSolver_->robot ());
            if (!robot) {
              throw Error ("Robot has to be set before applying constraints");
            }

            JointPtr_t la = robot->getJointByName (leftAnkle);
            JointPtr_t ra = robot->getJointByName (rightAnkle);

            std::vector<std::string> cnames;
            std::vector <DifferentiableFunctionPtr_t> constraints =
              createNaturalConstraintsManifold (robot, la, ra, *config);

            std::string p (prefix);
            std::string slash ("/");
            for(uint i=0;i<constraints.size();i++){
              DifferentiableFunctionPtr_t dfp = constraints.at(i);
              std::stringstream ss; ss << i; std::string id = ss.str();
              cnames.push_back(p+slash+id+dfp->name());
              problemSolver_->addNumericalConstraint(cnames.at(i), constraints.at(i));
            }

            return stringToNamesT(cnames);

          } catch (const std::exception& exc) {
            throw Error (exc.what ());
          }
        }

      } // end of namespace impl.
    } // end of namespace motionprior.
  } // end of namespace corbaServer.
} // end of namespace hpp.
