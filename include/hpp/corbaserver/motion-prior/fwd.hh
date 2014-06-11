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
# include <hpp/core/fwd.hh>
# include <hpp/constraints/fwd.hh>

namespace hpp {
  namespace corbaserver {
    namespace motionprior {
      typedef core::ProblemSolver ProblemSolver;
      typedef core::ProblemSolverPtr_t ProblemSolverPtr_t;
      typedef core::ConfigProjectorPtr_t ConfigProjectorPtr_t;
      typedef core::Configuration_t Configuration_t;
      typedef core::ConfigurationPtr_t ConfigurationPtr_t;
      typedef model::DevicePtr_t DevicePtr_t;
      typedef model::HumanoidRobotPtr_t HumanoidRobotPtr_t;
      typedef model::HumanoidRobot HumanoidRobot;

      typedef model::Transform3f Transform3f;
      typedef model::matrix3_t matrix3_t;
      typedef model::vector3_t vector3_t;
      typedef model::vector_t vector_t;

      typedef model::JointPtr_t JointPtr_t;
      typedef model::JointVector_t JointVector_t;
      typedef hpp::core::DifferentiableFunctionPtr_t DifferentiableFunctionPtr_t;

      typedef model::Body* BodyPtr_t;
      typedef model::ObjectVector_t ObjectVector_t;
      typedef model::ObjectIterator ObjectIterator;
      typedef model::Joint Joint_t;
      typedef model::Device Device_t;
      typedef model::DevicePtr_t DevicePtr_t;

      typedef fcl::CollisionGeometry CollisionGeometry_t;
      typedef boost::shared_ptr <CollisionGeometry_t> CollisionGeometryPtr_t;
      typedef model::CollisionObject CollisionObject_t;
      typedef model::CollisionObjectPtr_t CollisionObjectPtr_t;
      typedef model::Configuration_t Configuration_t;
      typedef core::ConfigurationPtr_t ConfigurationPtr_t;
      typedef core::ConfigIterator_t ConfigIterator_t;

      typedef constraints::Orientation Orientation;
      typedef constraints::OrientationPtr_t OrientationPtr_t;
      typedef constraints::Position Position;
      typedef constraints::PositionPtr_t PositionPtr_t;
      typedef constraints::RelativeOrientation RelativeOrientation;
      typedef constraints::RelativeComPtr_t RelativeComPtr_t;
      typedef constraints::RelativeCom RelativeCom;
      typedef constraints::RelativeOrientationPtr_t RelativeOrientationPtr_t;
      typedef constraints::RelativePosition RelativePosition;
      typedef constraints::RelativePositionPtr_t RelativePositionPtr_t;

      class ConstraintManifoldOperator;
      typedef boost::shared_ptr<ConstraintManifoldOperator> ConstraintManifoldOperatorPtr_t;

      namespace capsules{
        class ProjectedCapsulePoint;
        class CapsulePoint;
      }
      typedef capsules::ProjectedCapsulePoint ProjectedCapsulePoint_t;
      typedef boost::shared_ptr<ProjectedCapsulePoint_t> ProjectedCapsulePointPtr_t;
      typedef std::vector<ProjectedCapsulePointPtr_t> ProjectedCapsulePointVectorPtr_t;

      typedef capsules::CapsulePoint CapsulePoint_t;
      typedef boost::shared_ptr<CapsulePoint_t> CapsulePointPtr_t;
      typedef std::vector<CapsulePointPtr_t> CapsulePointVectorPtr_t;

      class Server;
    } // namespace motionprior
  } // namespace corbaserver
} // namespace hpp
