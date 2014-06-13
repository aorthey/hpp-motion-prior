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
      static std::map<std::string, bool> lockedDofs_;
      public:
        IrreducibleConfigurationShooter (const DevicePtr_t& robot) : robot_ (robot)
        {
          static bool firstRun = true;
          if(firstRun){
            firstRun = false;
            lockedDofs_["base_joint_x"]   = 0;
            lockedDofs_["base_joint_y"]   = 0;
            lockedDofs_["base_joint_z"]   = 0;
            lockedDofs_["base_joint_SO3"] = 0;
            lockedDofs_["CHEST_JOINT0"]   = 0;
            lockedDofs_["CHEST_JOINT1"]   = 0;
            lockedDofs_["HEAD_JOINT0"]    = 0;
            lockedDofs_["HEAD_JOINT1"   ] = 0;
            lockedDofs_["LARM_JOINT0"   ] = 0;
            lockedDofs_["LARM_JOINT1"   ] = 0;
            lockedDofs_["LARM_JOINT2"   ] = 0;
            lockedDofs_["LARM_JOINT3"   ] = 0;
            lockedDofs_["LARM_JOINT4"   ] = 0;
            lockedDofs_["LARM_JOINT5"   ] = 0;
            lockedDofs_["LARM_JOINT6"   ] = 1;
            lockedDofs_["LHAND_JOINT0"  ] = 1;
            lockedDofs_["LHAND_JOINT1"  ] = 1;
            lockedDofs_["LHAND_JOINT2"  ] = 1;
            lockedDofs_["LHAND_JOINT3"  ] = 1;
            lockedDofs_["LHAND_JOINT4"  ] = 1;
            lockedDofs_["RARM_JOINT0"   ] = 0;
            lockedDofs_["RARM_JOINT1"   ] = 0;
            lockedDofs_["RARM_JOINT2"   ] = 0;
            lockedDofs_["RARM_JOINT3"   ] = 0;
            lockedDofs_["RARM_JOINT4"   ] = 0;
            lockedDofs_["RARM_JOINT5"   ] = 0;
            lockedDofs_["RARM_JOINT6"   ] = 1;
            lockedDofs_["RHAND_JOINT0"  ] = 1;
            lockedDofs_["RHAND_JOINT1"  ] = 1;
            lockedDofs_["RHAND_JOINT2"  ] = 1;
            lockedDofs_["RHAND_JOINT3"  ] = 1;
            lockedDofs_["RHAND_JOINT4"  ] = 1;
            lockedDofs_["LLEG_JOINT0"   ] = 0;
            lockedDofs_["LLEG_JOINT1"   ] = 0;
            lockedDofs_["LLEG_JOINT2"   ] = 0;
            lockedDofs_["LLEG_JOINT3"   ] = 0;
            lockedDofs_["LLEG_JOINT4"   ] = 0;
            lockedDofs_["LLEG_JOINT5"   ] = 0;
            lockedDofs_["RLEG_JOINT0"   ] = 0;
            lockedDofs_["RLEG_JOINT1"   ] = 0;
            lockedDofs_["RLEG_JOINT2"   ] = 0;
            lockedDofs_["RLEG_JOINT3"   ] = 0;
            lockedDofs_["RLEG_JOINT4"   ] = 0;
            lockedDofs_["RLEG_JOINT5"   ] = 0;
          }
        }

        virtual ConfigurationPtr_t shoot () const
        {
          JointVector_t jv = robot_->getJointVector ();
          ConfigurationPtr_t config (new Configuration_t (robot_->configSize ()));
          config.zero();
          for (JointVector_t::const_iterator itJoint = jv.begin ();
               itJoint != jv.end (); itJoint++) {
            std::size_t rank = (*itJoint)->rankInConfiguration ();
            if( !lockedDofs_[(*itJoint)->name()] ){
              (*itJoint)->configuration ()->uniformlySample (rank, *config);
            }
          }
          return config;
        }
      private:
        const DevicePtr_t& robot_;
      }; // class IrreducibleConfigurationShooter
    } //   namespace motionprior
  } //   namespace corbaserver
} // namespace hpp
