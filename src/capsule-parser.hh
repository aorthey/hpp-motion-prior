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

#pragma once
#include <vector>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>

#include <hpp/util/debug.hh>
#include <hpp/model/fwd.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/urdf/util.hh>
#include <hpp/model/object-factory.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/corbaserver/server.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/differentiable-function.hh>
#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/motion-prior/fwd.hh>
#include "precomputation.hh"

namespace hpp
{
  namespace corbaserver
  {
    namespace motionprior
    {
        /// \brief A capsule point represents the center of the top or bottom
        ///  part of the cylinder included in the capsule representation. 
        ///
        /// It contains R^3 coordinates, the length of the cylinder and the radius
        /// of the halfspheres, which are located at the top and bottom of the
        /// cylinder, with the capsule point being its center. Given the two capsule
        /// points of a capsule, we can compute every other point on the capsule
        /// surface. Additionally, we save the jacobian of the associated joint,
        /// such that we can use it for optimization purposes.

        struct CapsulePoint {
          double x,y,z;
          double radius;
          double length;
          hpp::model::JointJacobian_t J;
        };

        struct ProjectedCapsulePoint {
          double y,z;
          hpp::model::JointJacobian_t J;
          uint idx;
          bool operator <(const ProjectedCapsulePoint &rhs) const {
                  return y < rhs.y || (y == rhs.y && z < rhs.z);
          }
          bool isInsideConvexHull(const std::vector<ProjectedCapsulePoint> &cvxHullPts) const;
        };

        /// \brief Parse capsule points from the robot geometry and return them
        ///  in a vector
        std::vector<CapsulePoint> parseCapsulePoints (DevicePtr_t robot) throw (hpp::Error);

        /// \brief Project Capsule Points onto ZY Plane including outer points
        std::vector<ProjectedCapsulePoint> projectCapsulePointsOnYZPlane (const std::vector<CapsulePoint> &capsVec);
        std::vector<ProjectedCapsulePoint> computeConvexHullFromProjectedCapsulePoints (const std::vector<ProjectedCapsulePoint> &capsVec);


        /// \brief Convert capsule point vector to hpp::floatSeq 
        hpp::floatSeq* capsulePointsToFloatSeq (const std::vector<CapsulePoint> &capsVector) 
           throw (hpp::Error);
        hpp::floatSeq* capsulePointsToFloatSeq (const std::vector<ProjectedCapsulePoint> &capsVector) 
           throw (hpp::Error);

    } // end of namespace motionprior
  } // end of namespace corbaServer.
} // end of namespace hpp.
