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

#include <boost/assign/list_of.hpp>
#include <hpp/model/humanoid-robot.hh>
#include <hpp/model/joint.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/locked-dof.hh>
#include <hpp/util/debug.hh>

#include <Eigen/Geometry>

#include <hpp/constraints/orientation.hh>
#include <hpp/constraints/position.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/constraints/relative-orientation.hh>
#include <hpp/constraints/relative-position.hh>

#include "constraint-manifold-operator.hh"

namespace hpp
{
  namespace corbaserver
  {
    namespace motionprior
    {

      ConstraintManifoldOperator::ConstraintManifoldOperator(hpp::core::ProblemSolverPtr_t &problemSolver):
              problemSolver_(problemSolver)
      {
      }

      std::vector< DifferentiableFunctionPtr_t > 
      ConstraintManifoldOperator::getConstraintSet( 
                        const DevicePtr_t &robot,
                        const JointPtr_t& joint1,
                        const JointPtr_t& joint2)
        throw (hpp::Error)
      {
        std::vector< DifferentiableFunctionPtr_t > constraintSet;

        const Transform3f& M1 = joint1->currentTransformation ();
        const Transform3f& M2 = joint2->currentTransformation ();

        vector3_t zero; zero.setZero ();
        matrix3_t I3; I3.setIdentity ();
        const vector3_t& com = robot->positionCenterOfMass ();

        hpp::model::matrix3_t RZ90;
        RZ90.setEulerZYX(0,0,-M_PI/2);

        std::vector< bool > zmask = boost::assign::list_of(true )(true )(false);
        std::vector< bool > xmask = boost::assign::list_of(true )(false)(true );
        std::vector< bool > ymask = boost::assign::list_of(false)(true )(true );
        std::vector< bool > xy_translation = boost::assign::list_of(false)(false )(true );
        std::vector< bool > z_translation = boost::assign::list_of(true)(true )(false );
        std::vector< bool > default_mask = boost::assign::list_of(true)(true )(true );

        // --------------------------------------------------------------------
        // Left Foot Constraints 
        // (Orientation fixed 90 degree z-axis, Position at origin)
        // --------------------------------------------------------------------
        constraintSet.push_back (Orientation::create (robot, joint1, RZ90));
        constraintSet.push_back (Position::create (robot, joint1, zero, zero, I3));

        // --------------------------------------------------------------------
        // Right Foot Constraints 
        // (Orientation variable around z-axis, 
        // translation variable in xy-plane at z=0)
        // --------------------------------------------------------------------
        constraintSet.push_back (Position::create (robot, joint2, zero, zero, I3, xy_translation));
        constraintSet.push_back (Orientation::create (robot, joint2, RZ90, zmask));

        // --------------------------------------------------------------------
        // position of center of mass in left ankle frame
        // --------------------------------------------------------------------
        //matrix3_t R1T (M1.getRotation ()); R1T.transpose ();
        //vector3_t xloc = R1T * (com - M1.getTranslation ());
        //constraintSet.push_back (RelativeCom::create (robot, joint1, xloc));

        matrix3_t R1T (M1.getRotation ()); R1T.transpose ();
        matrix3_t R2T (M2.getRotation ()); R2T.transpose ();


        //Eigen::Quaternion<double> rq1(R1T);
        //Eigen::Quaternion<double> rq2;
        //rq2 = M2.getRotation();

        Quaternion_t rq1fcl; rq1fcl.fromRotation (M1.getRotation());
        Quaternion_t rq2fcl; rq2fcl.fromRotation (M2.getRotation());
        Eigen::Quaternion<double> rq1(rq1fcl.getW(), rq1fcl.getX(), rq1fcl.getY(), rq1fcl.getZ());
        Eigen::Quaternion<double> rq2(rq2fcl.getW(), rq2fcl.getX(), rq2fcl.getY(), rq2fcl.getZ());

        Eigen::Quaternion<double> rqc = rq2.slerp(0.5, rq1);
        Quaternion_t rqcfcl(rqc.w(), rqc.x(), rqc.y(), rqc.z());

        matrix3_t RCT;
        rqcfcl.toRotation(RCT);
        RCT.transpose();

        //vector3_t ml = 0.5 * (M1.getTranslation() - M2.getTranslation());
        //vector3_t xloc = RCT * ml;
        vector3_t m2v = M2.getTranslation();
        m2v[0] = 0;
        m2v[1] = 0;
        m2v[2] = 0.7;

        vector3_t xloc = R1T * m2v;

        constraintSet.push_back (RelativeCom::create (robot, joint1, xloc));

        //vector3_t xc = R1T * (com - M1.getTranslation ());
        //constraintSet.push_back (RelativeCom::create (robot, joint1, xc));

        return constraintSet;
      }

      void ConstraintManifoldOperator::init()
        throw (hpp::Error)
      {

	try {
          const DevicePtr_t& robot (problemSolver_->robot ());
  
          const char* constraintSetName = "mv-irr-constraint-set";
          const char* leftAnkle = "LLEG_JOINT5";
          const char* rightAnkle = "RLEG_JOINT5";
  
          JointPtr_t joint1 = robot->getJointByName (leftAnkle);
          JointPtr_t joint2 = robot->getJointByName (rightAnkle);
  
          constraintSet_ = getConstraintSet(robot, joint1, joint2);
  
          std::vector<std::string> cnames;
          std::string p (constraintSetName);
          std::string slash ("/");
  
          for(uint i=0;i<constraintSet_.size();i++){
            DifferentiableFunctionPtr_t dfp = constraintSet_.at(i);
            std::stringstream ss; ss << i; std::string id = ss.str();
            cnames.push_back(p+slash+id+dfp->name());
            problemSolver_->addNumericalConstraint(cnames.at(i), constraintSet_.at(i));
          }
  
	  ConfigProjectorPtr_t configProjector = problemSolver_->constraints()->configProjector ();

	  if (!configProjector) {

	    configProjector = ConfigProjector::create
	      (robot, constraintSetName, problemSolver_->errorThreshold (),
	       problemSolver_->maxIterations ());

	    problemSolver_->constraints()->addConstraint (configProjector);

	  }
	  for (uint i=0; i<cnames.size (); ++i) {
	    configProjector->addConstraint (problemSolver_->numericalConstraint
					    (cnames.at(i)));
          }
          //add locked dofs
          // --------------------------------------------------------------------
          // add locked dofs
          // --------------------------------------------------------------------

          addLockedDof("RARM_JOINT6" , 0.1);
          addLockedDof("LARM_JOINT6" , 0.1);
          addLockedDof("RHAND_JOINT0", 0.0);
          addLockedDof("RHAND_JOINT1", 0.0);
          addLockedDof("RHAND_JOINT2", 0.0);
          addLockedDof("RHAND_JOINT3", 0.0);
          addLockedDof("RHAND_JOINT4", 0.0);
          addLockedDof("LHAND_JOINT0", 0.0);
          addLockedDof("LHAND_JOINT1", 0.0);
          addLockedDof("LHAND_JOINT2", 0.0);
          addLockedDof("LHAND_JOINT3", 0.0);
          addLockedDof("LHAND_JOINT4", 0.0);

	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }
      void ConstraintManifoldOperator::addLockedDof(const char* name, double value){
        JointPtr_t joint = problemSolver_->robot()->getJointByName(name);
	LockedDofPtr_t lockedDof (LockedDof::create (name, joint, value));
	problemSolver_->constraints()->addConstraint (lockedDof);
      }

      double ConstraintManifoldOperator::apply( Configuration_t &q )
        throw (hpp::Error)
      {
        success_ = false;
	try {
	  success_ = problemSolver_->constraints ()->apply (q);

	  if (hpp::core::ConfigProjectorPtr_t configProjector =
	      problemSolver_->constraints ()->configProjector ()) {

	    double residualError = configProjector->residualError ();

            if( !isSelfColliding(q) ){
              return residualError;
            }else{
              success_ = false;
              return residualError;
            }
	  }
          return NAN;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      bool ConstraintManifoldOperator::isSelfColliding( Configuration_t &q ){
        DevicePtr_t robot = problemSolver_->robot ();
        robot->currentConfiguration(q);
	robot->computeForwardKinematics ();
        return (robot->collisionTest());
      }

      void ConstraintManifoldOperator::deleteConstraints()
        throw (hpp::Error)
      {
	try {
	  problemSolver_->resetConstraints ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      bool ConstraintManifoldOperator::success(){
        return success_;
      }
    } // end of namespace motionprior.
  } // end of namespace corbaServer.
} // end of namespace hpp.


