#pragma once
# include <hpp/core/fwd.hh>

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

      class Server;
    } // namespace motionprior
  } // namespace corbaserver
} // namespace hpp
  

