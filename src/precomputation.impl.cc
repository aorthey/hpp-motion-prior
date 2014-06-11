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

        hpp::floatSeq* Precomputation::getRandomConfiguration () throw (hpp::Error)
        {
          uint ctr = 0;
          while(true){
            Configuration_t q = shootRandomConfigVector();
            if(projectOntoConstraintManifold(q)){
              hppDout(notice, "[ConstraintManifoldProjector] successful projection after " << ctr << " iterations.");
              //save internal state for future get methods
              q_ = q;
              cvxCaps_ = getProjectedConvexHullFromConfiguration(q_);
              return vectorToFloatSeq(q);
            }
            ctr++;
          }
        }
        hpp::floatSeq* Precomputation::getRandomIrreducibleConfiguration () throw (hpp::Error)
        {
          uint ctr = 0;
          while(true){
            Configuration_t q = shootRandomConfigVector();
            if(projectOntoConstraintManifold(q)){
              if(projectOntoIrreducibleManifold(q)){
                hppDout(notice, "successful projection after " << ctr << " iterations.");
                //save internal state for future get methods
                q_ = q;
                cvxCaps_ = getProjectedConvexHullFromConfiguration(q_);
                return vectorToFloatSeq(q);
              }
            }
            ctr++;
          }
        }

        bool Precomputation::projectOntoConstraintManifold (Configuration_t &q) throw (hpp::Error)
        {
          if(!cnstrOp_){
            cnstrOp_.reset( new ConstraintManifoldOperator(problemSolver_) );
          }
          cnstrOp_->deleteConstraints();
          cnstrOp_->init();
          cnstrOp_->apply(q);
          return cnstrOp_->success();
        }

        bool Precomputation::projectOntoIrreducibleManifold (Configuration_t &q) throw (hpp::Error)
        {
          const double lambda = 0.1; //gradient step size

          Configuration_t qq = this->getGradientFromConfiguration(q);
          Configuration_t q_proj = this->step(qq, lambda);

          ProjectedCapsulePointVectorPtr_t cvxCapsOld = getProjectedConvexHullFromConfiguration (q);
          ProjectedCapsulePointVectorPtr_t cvxCapsNew = getProjectedConvexHullFromConfiguration (q_proj);

          if(isSmallerVolume(cvxCapsNew, cvxCapsOld)){
            return true;
          }else{
            return false;
          }
        }

        double Precomputation::getVolume () throw (hpp::Error)
        {
          return capsules::getVolume(cvxCaps_);
        }
        hpp::floatSeq* Precomputation::getConvexHullCapsules () throw (hpp::Error)
        {
          return capsulePointsToFloatSeq(cvxCaps_);
        }

        ProjectedCapsulePointVectorPtr_t Precomputation::getProjectedConvexHullFromConfiguration (const Configuration_t &q)
          throw (hpp::Error)
        {
          DevicePtr_t robot = problemSolver_->robot ();
          robot->currentConfiguration(q);
          CapsulePointVectorPtr_t caps = parseCapsulePoints(robot);
          ProjectedCapsulePointVectorPtr_t projCaps = projectCapsulePointsOnYZPlane(caps);
          ProjectedCapsulePointVectorPtr_t cvxCaps = computeConvexHullFromProjectedCapsulePoints(projCaps);
          return cvxCaps;
        }

        Configuration_t Precomputation::getGradientFromConfiguration(const Configuration_t &q) throw (hpp::Error)
        {
          //Compute gradient wrt outer jacobians
          DevicePtr_t robot = problemSolver_->robot ();
          ProjectedCapsulePointVectorPtr_t cvxCaps = getProjectedConvexHullFromConfiguration (q);
          cvxCaps_ = cvxCaps;

          Configuration_t qgrad(robot->numberDof());
          qgrad.setZero();

          for(uint i=0; i<cvxCaps.size(); i++){
            Configuration_t cvx_pt_eigen(6);
            cvx_pt_eigen << 0,cvxCaps.at(i)->y,cvxCaps.at(i)->z,0,0,0;
            Configuration_t qi = cvxCaps.at(i)->J.transpose()*cvx_pt_eigen;
            qgrad = qgrad + qi;
          }
          return qgrad;
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

      } // end of namespace impl.
    } // end of namespace motionprior.
  } // end of namespace corbaServer.
} // end of namespace hpp.
