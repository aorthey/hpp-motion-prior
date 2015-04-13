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
#include <hpp/core/locked-joint.hh>
#include <hpp/util/debug.hh>

#include <Eigen/Geometry>

#include <hpp/constraints/orientation.hh>
#include <hpp/constraints/position.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/constraints/relative-orientation.hh>
#include <hpp/constraints/relative-position.hh>

#include "constraint-manifold-operator.hh"
#include "precomputation-utils.hh"

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
                        const Configuration_t &qinit,
                        const JointPtr_t& left_sole,
                        const JointPtr_t& right_sole)
        throw (hpp::Error)
      {
        std::vector< DifferentiableFunctionPtr_t > constraintSet;
        robot->currentConfiguration (qinit);
        robot->computeForwardKinematics ();

        const Transform3f& M1 = left_sole->currentTransformation ();
        const Transform3f& M2 = right_sole->currentTransformation ();

        vector3_t zero; zero.setZero ();
        matrix3_t I3; I3.setIdentity ();
        const vector3_t& com = robot->positionCenterOfMass ();

        hpp::model::matrix3_t RZ90;
        RZ90.setEulerZYX(0,0,-M_PI/2);

        std::vector< bool > pitchmask = boost::assign::list_of(true )(false)(true );
        std::vector< bool > xy_translation = boost::assign::list_of(false)(false )(true );
        std::vector< bool > z_translation = boost::assign::list_of(true)(true )(false );
        std::vector< bool > default_mask = boost::assign::list_of(true)(true )(true );

        // --------------------------------------------------------------------
        // Left Foot Constraints 
        // (Orientation completely fixed 90 degree z-axis, Position at free x/y, fixed z=0)
        // --------------------------------------------------------------------
        //constraintSet.push_back (Position::create (robot, left_sole, zero, M1.getTranslation(), I3, xy_translation));
        //constraintSet.push_back (Orientation::create (robot, left_sole, I3, default_mask));

        // --------------------------------------------------------------------
        // Right Foot Constraints 
        // (Orientation fixed according to left foot,
        // translation variable fixed according to left foot
        // --------------------------------------------------------------------
        matrix3_t R1T (M1.getRotation ()); R1T.transpose ();
        matrix3_t R2T (M2.getRotation ()); R2T.transpose ();

        matrix3_t reference = R1T * M2.getRotation ();
        vector3_t local1; local1.setZero ();
        vector3_t global1 = M1.getTranslation ();
        vector3_t local2 = R2T * (global1 - M2.getTranslation ());

        //constraintSet.push_back (RelativeOrientation::create (robot, left_sole, right_sole, reference));
        //constraintSet.push_back (RelativePosition::create (robot, left_sole, right_sole, local1, local2));

        // --------------------------------------------------------------------
        // fix base_link position X,Y according to left foot
        // --------------------------------------------------------------------

        JointPtr_t base_link_joint = robot->getJointByName ("WAIST");
        const Transform3f& base_link = base_link_joint->currentTransformation ();
        matrix3_t RBT (base_link.getRotation ()); RBT.transpose ();

        //vector3_t local_base = R1T * (base_link.getTranslation() - M1.getTranslation ());
        //constraintSet.push_back (RelativePosition::create (robot, left_sole, base_link_joint, local1, local_base, z_translation));

//matrix3_t R2T (M2.getRotation ()); R2T.transpose ();
//vector3_t local2 = R2T * (global1 - M2.getTranslation ());
        //vector3_t local_base = RBT * (M1.getTranslation() - base_link.getTranslation ());
        //constraintSet.push_back (RelativePosition::create (robot, left_sole, base_link_joint, local1, local_base, default_mask));

        //vector3_t xloc = RBT * (M1.getTranslation() - base_link.getTranslation ());
        //constraintSet.push_back (RelativeCom::create (robot, left_sole, xloc));
        //constraintSet.push_back (RelativePosition::create (robot, left_sole, base_link_joint, local1, xloc));

        matrix3_t local_base_rot = R1T * base_link.getRotation ();
        constraintSet.push_back (RelativeOrientation::create (robot, left_sole, base_link_joint, local_base_rot));


        // --------------------------------------------------------------------
        // fix base_link roll/yaw according to left foot
        // --------------------------------------------------------------------
        //JointPtr_t base_link_joint_theta = robot->getJointByName ("base_joint_SO3");
        //const Transform3f& base_link_theta = base_link_joint_theta->currentTransformation ();

        //matrix3_t referenceRot = R1T * base_link_theta.getRotation ();

        //constraintSet.push_back (RelativeOrientation::create (robot, left_sole, base_link_joint_theta, referenceRot, pitchmask));
  
        return constraintSet;
      }

      hpp::Names_t* ConstraintManifoldOperator::getConstraintSet () throw (hpp::Error)
      {
        return constraint_names_;
      }

      void ConstraintManifoldOperator::init(Configuration_t &qinit)
        throw (hpp::Error)
      {

	try {
          const DevicePtr_t& robot (problemSolver_->robot ());
  
          const char* constraintSetName = "stability-constraints";
          const char* leftAnkle = "LLEG_JOINT5";
          const char* rightAnkle = "RLEG_JOINT5";
  
          JointPtr_t left_sole = robot->getJointByName (leftAnkle);
          JointPtr_t right_sole = robot->getJointByName (rightAnkle);
  
          constraintSet_ = getConstraintSet(robot, qinit, left_sole, right_sole);
  
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
	    configProjector->add(problemSolver_->numericalConstraint
					    (cnames.at(i)));
          }
          // --------------------------------------------------------------------
          // add locked Joints
          // --------------------------------------------------------------------

          addLockedJoint("RARM_JOINT6" , 0.1);
          addLockedJoint("LARM_JOINT6" , 0.1);
          addLockedJoint("RHAND_JOINT0", 0.0);
          addLockedJoint("RHAND_JOINT1", 0.0);
          addLockedJoint("RHAND_JOINT2", 0.0);
          addLockedJoint("RHAND_JOINT3", 0.0);
          addLockedJoint("RHAND_JOINT4", 0.0);
          addLockedJoint("LHAND_JOINT0", 0.0);
          addLockedJoint("LHAND_JOINT1", 0.0);
          addLockedJoint("LHAND_JOINT2", 0.0);
          addLockedJoint("LHAND_JOINT3", 0.0);
          addLockedJoint("LHAND_JOINT4", 0.0);

          constraint_names_ = stringToNamesT(cnames);

	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }
      void ConstraintManifoldOperator::addLockedJoint(const char* name, double value){
        JointPtr_t joint = problemSolver_->robot()->getJointByName(name);
	//LockedJointPtr_t lockedJoint (LockedJoint::create (name, joint, value));
        Eigen::VectorXd V(1);
        V[0]=value;
	LockedJointPtr_t lockedJoint (LockedJoint::create (joint, V));
        ConfigProjectorPtr_t configProjector = problemSolver_->constraints()->configProjector ();
	configProjector->add(lockedJoint);
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


