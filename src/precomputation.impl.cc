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

# include "precomputation.impl.hh"
# include "precomputation-utils.hh"
# include "constraint-manifold-operator.hh"
# include "capsule-parser.hh"


namespace hpp
{
  namespace corbaserver
  {
    namespace motionprior
    {
      namespace impl
      {
        using namespace hpp::corbaserver::motionprior::capsules;
        //Precomputation::Precomputation(corbaServer::Server* server) :
        //  server_(server), problemSolver_(server->problemSolver ())
        //{
        //}
        Precomputation::Precomputation () : problemSolver_ (0x0) {
        }
        void Precomputation::setProblemSolver
        (const ProblemSolverPtr_t& problemSolver)
        {
          problemSolver_ = problemSolver;
        }

        vector_t Precomputation::shootRandomConfigVector() throw (hpp::Error){
          DevicePtr_t robot = problemSolver_->robot ();

          hpp::core::BasicConfigurationShooter confShooter(robot);
          ConfigurationPtr_t configPtr = confShooter.shoot();

          hpp::model::ConfigurationIn_t config = *configPtr.get();
          robot->currentConfiguration(config);
          vector_t q = robot->currentConfiguration();
          return q;
        }

        hpp::floatSeq* Precomputation::shootRandomConfig() throw (hpp::Error){
          vector_t q = shootRandomConfigVector();
          return vectorToFloatSeq(q);
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

        hpp::floatSeq* Precomputation::getApproximateIrreducibleConfiguration () throw (hpp::Error)
        {
          DevicePtr_t robot = problemSolver_->robot ();
          while(true){
            Configuration_t q = shootRandomConfigVector();
            Configuration_t q_proj;
            if(projectOntoConstraintManifold(q,q_proj)){
              Configuration_t q_irr;
              if(projectOntoIrreducibleManifold(q_proj,q_irr)){
                return vectorToFloatSeq(q_irr);
              }
            }
          }
        }

        bool Precomputation::projectOntoConstraintManifold (vector_t &q) throw (hpp::Error){
          if(!cnstrOp_){
            cnstrOp_.reset( new ConstraintManifoldOperator(problemSolver_) );
          }
          cnstrOp_->deleteConstraints();
          cnstrOp_->init();
          cnstrOp_->apply(q);
          return cnstrOp_->success();
        }

        bool Precomputation::projectOntoIrreducibleManifold (const Configuration_t &q, Configuration_t &q_proj) throw (hpp::Error){
          const double lambda = 0.1; //gradient step size

          Configuration_t qq = this->getGradientVector(q);
          q_proj = this->step(qq, lambda);

          ProjectedCapsulePointVectorPtr cvxCapsOld = getProjectedConvexHullFromConfiguration (q);
          ProjectedCapsulePointVectorPtr cvxCapsNew = getProjectedConvexHullFromConfiguration (q_proj);

          if(isSmallerVolume(cvxCapsNew, cvxCapsOld)){
            return true;
          }else{
            return false;
          }
        }

        /*
        hpp::floatSeq* Precomputation::projectUntilIrreducibleConstraint () throw (hpp::Error)
        {
          try {
            double epsilon = 0.001; //convergence threshold
            uint iterations = 0;
            double error = 1;
            double oldC = 10000;
            double lambda = 0.1; //update value
            computeProjectedConvexHullFromCurrentConfiguration ();

            cnstrOp_.reset( new ConstraintManifoldOperator(problemSolver_) );
            cnstrOp_->deleteConstraints();
            cnstrOp_->init();

            std::vector<ProjectedCapsulePoint> cvxCapsOld;
            cvxCapsOld = cvxCaps_;

            while(error > epsilon){
              Configuration_t qq = this->getGradientVector();
              Configuration_t q = this->step(qq, lambda);

              cnstrOp_->apply(q);

              if(cnstrOp_->success()){
                //new configuration was successfully projected back on the given
                //contraint manifold
                this->setCurrentConfiguration(q);
                computeProjectedConvexHullFromCurrentConfiguration ();

                if(isSmallerVolume(cvxCaps_, cvxCapsOld)){
                  double C = this->getVolume();
                  error = fabs(C-oldC);
                  oldC = C;
                  iterations++;
                  cvxCapsOld = cvxCaps_;
                }else{
                  hppDout(notice, "projection terminated because new volume is not a subset of the last step" );
                  break;
                }
              }else{
                hppDout(notice, "projection terminated due to non-successful projection onto constraint manifold" );
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
        */

        double Precomputation::getVolume () throw (hpp::Error)
        {
          return capsules::getVolume(cvxCaps_);
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

        ProjectedCapsulePointVectorPtr Precomputation::getProjectedConvexHullFromConfiguration (vector_t &q)
          throw (hpp::Error)
        {
          DevicePtr_t robot = problemSolver_->robot ();
          robot->currentConfiguration(q);
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

            using namespace hpp::corbaserver::motionprior;
            ConfigurationPtr_t config = dofSeqToConfig (problemSolver_, dofArray);
            const DevicePtr_t& robot (problemSolver_->robot ());
            if (!robot) {
              throw Error ("Robot has to be set before applying constraints");
            }

            JointPtr_t la = robot->getJointByName (leftAnkle);
            JointPtr_t ra = robot->getJointByName (rightAnkle);

            std::vector<std::string> cnames;
            ConstraintManifoldOperator M (problemSolver_);
            std::vector <DifferentiableFunctionPtr_t> constraints = M.getConstraintSet( robot, la, ra);

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
