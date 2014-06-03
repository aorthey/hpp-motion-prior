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

        vector3_t zero; zero.setZero ();
        matrix3_t I3; I3.setIdentity ();
        //const vector3_t& com = robot->positionCenterOfMass ();
        vector3_t com(0,0,0.75);

        hpp::model::matrix3_t RZ90;
        RZ90.setEulerZYX(0,0,-M_PI/2);

        std::vector< bool > zmask = boost::assign::list_of(true )(true )(false);
        std::vector< bool > xmask = boost::assign::list_of(true )(false)(true );
        std::vector< bool > ymask = boost::assign::list_of(false)(true )(true );
        std::vector< bool > xy_translation = boost::assign::list_of(false)(false )(true );
        std::vector< bool > default_mask = boost::assign::list_of(true)(true )(true );

        // --------------------------------------------------------------------
        // position of center of mass in left ankle frame
        // --------------------------------------------------------------------
        matrix3_t R1T (M1.getRotation ()); R1T.transpose ();
        vector3_t xloc = R1T * (com - M1.getTranslation ());
        result.push_back (RelativeCom::create (robot, joint1, xloc));

        // --------------------------------------------------------------------
        // Left Foot Constraints 
        // (Orientation fixed 90 degree z-axis, Position at origin)
        // --------------------------------------------------------------------
        result.push_back (Orientation::create (robot, joint1, RZ90));
        result.push_back (Position::create (robot, joint1, zero, zero, I3));

        // --------------------------------------------------------------------
        // Right Foot Constraints 
        // (Orientation variable around z-axis, 
        // translation variable in xy-plane at z=0)
        // --------------------------------------------------------------------
        result.push_back (Position::create (robot, joint2, zero, zero, I3, xy_translation));
        result.push_back (Orientation::create (robot, joint2, RZ90, zmask));


        return result;
      }
    } // end of namespace precomputation
  } // end of namespace corbaServer.
} // end of namespace hpp.
