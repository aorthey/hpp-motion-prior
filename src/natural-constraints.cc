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
#include <hpp/util/debug.hh>
#include <hpp/constraints/orientation.hh>
#include <hpp/constraints/position.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/constraints/relative-orientation.hh>
#include <hpp/constraints/relative-position.hh>
#include <Eigen/Geometry>

#include "natural-constraints.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace precomputation
    {

      using hpp::constraints::Orientation;
      using hpp::constraints::OrientationPtr_t;
      using hpp::constraints::Position;
      using hpp::constraints::PositionPtr_t;
      using hpp::constraints::RelativeOrientation;
      using hpp::constraints::RelativeComPtr_t;
      using hpp::constraints::RelativeCom;
      using hpp::constraints::RelativeOrientationPtr_t;
      using hpp::constraints::RelativePosition;
      using hpp::constraints::RelativePositionPtr_t;

      std::vector <DifferentiableFunctionPtr_t> createNaturalConstraintsManifold
      (const DevicePtr_t& robot, const JointPtr_t& leftAnkle,
       const JointPtr_t& rightAnkle, ConfigurationIn_t configuration)
      {
        std::vector <DifferentiableFunctionPtr_t> result;

        robot->currentConfiguration (configuration);
        robot->computeForwardKinematics ();

        JointPtr_t joint1 = leftAnkle;
        JointPtr_t joint2 = rightAnkle;
        const Transform3f& M1 = joint1->currentTransformation ();
        const Transform3f& M2 = joint2->currentTransformation ();
        const vector3_t& x = robot->positionCenterOfMass ();

        vector3_t zero; zero.setZero ();
        matrix3_t I3; I3.setIdentity ();

        hpp::model::matrix3_t RZ90;
        RZ90.setEulerZYX(M_PI/2,0,0);



        //hpp::model::matrix3_t rotation;
        //fcl::Quaternion3f quat;
        //quat.fromAxisAngle(fcl::Vec3f(0,0,1), M_PI);
        //quat.toRotation(rotation);

        // --------------------------------------------------------------------
        // position of center of mass in left ankle frame
        // --------------------------------------------------------------------
        //matrix3_t R1T (M1.getRotation ()); R1T.transpose ();
        //vector3_t xloc = R1T * (x - M1.getTranslation ());
        //result.push_back (RelativeCom::create (robot, joint1, xloc));

        // --------------------------------------------------------------------
        // Relative orientation of the feet
        // --------------------------------------------------------------------

        // --------------------------------------------------------------------
        // Relative position of the feet
        // --------------------------------------------------------------------
        //vector3_t local1; local1.setZero ();
        //vector3_t global1 = M1.getTranslation ();
        //// global1 = R2 local2 + t2
        //// local2  = R2^T (global1 - t2)
        //matrix3_t R2T (M2.getRotation ()); R2T.transpose ();
        //vector3_t local2 = R2T * (global1 - M2.getTranslation ());
        //result.push_back (RelativePosition::create
        //  		(robot, joint1, joint2, local1, local2));


        std::vector< bool > zmask = boost::assign::list_of(true )(true )(false);
        std::vector< bool > xmask = boost::assign::list_of(true )(false)(true );
        std::vector< bool > ymask = boost::assign::list_of(false)(true )(true );
        std::vector< bool > default_mask = boost::assign::list_of(true)(true )(true );

        //stay on floor
        //result.push_back(Orientation::create (robot, joint1, I3, zmask));

      // Position of the left foot
        /*
        result.push_back (Position::create
			(robot, joint1, zero, zero, I3,
			 boost::assign::list_of (true)(true)(true)));
                         */

        // --------------------------------------------------------------------
        // Position of right foot constraint to be above x-y plane
        // --------------------------------------------------------------------

        // --------------------------------------------------------------------
        // Orientation of the left foot
        // --------------------------------------------------------------------
        //result.push_back (Orientation::create (robot, joint1, RZ90, default_mask));
        // --------------------------------------------------------------------
        // Position of the left foot
        // --------------------------------------------------------------------
        result.push_back
          (Position::create (robot, joint1, zero, zero, I3));
        // --------------------------------------------------------------------
        return result;
      }
    } // end of namespace precomputation
  } // end of namespace corbaServer.
} // end of namespace hpp.
