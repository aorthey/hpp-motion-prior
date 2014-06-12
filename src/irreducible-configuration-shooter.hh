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
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
# include <hpp/core/configuration-shooter.hh>

namespace hpp {
  namespace corbaserver {
    namespace motionprior {
      class HPP_CORE_DLLAPI IrreducibleConfigurationShooter :
        public hpp::core::ConfigurationShooter
      {
      public:
        IrreducibleConfigurationShooter (const DevicePtr_t& robot) : robot_ (robot)
        {
        }
        virtual ConfigurationPtr_t shoot () const
        {
          JointVector_t jv = robot_->getJointVector ();
          ConfigurationPtr_t config (new Configuration_t (robot_->configSize ()));
          for (JointVector_t::const_iterator itJoint = jv.begin ();
               itJoint != jv.end (); itJoint++) {
            std::size_t rank = (*itJoint)->rankInConfiguration ();
            (*itJoint)->configuration ()->uniformlySample (rank, *config);
          }
          //remove locked dof
          return config;
        }
      private:
        const DevicePtr_t& robot_;
      }; // class IrreducibleConfigurationShooter
    } //   namespace motionprior
  } //   namespace corbaserver
} // namespace hpp
