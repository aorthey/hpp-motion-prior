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
      static std::map<std::string, bool> variableDofs_;
      static std::map<std::string, double> lockedDofsFixedValue_;
      public:
        IrreducibleConfigurationShooter (const DevicePtr_t& robot) : robot_ (robot)
        {
          static bool firstRun = true;
          if(firstRun){
            firstRun = false;
            lockedDofs_["base_joint_x"]   = 1;
            lockedDofs_["base_joint_y"]   = 1;
            lockedDofs_["base_joint_z"]   = 1;
            lockedDofs_["base_joint_SO3"] = 1;
            lockedDofs_["CHEST_JOINT0"]   = 1;
            lockedDofs_["CHEST_JOINT1"]   = 0;
            lockedDofs_["HEAD_JOINT0"]    = 1;
            lockedDofs_["HEAD_JOINT1"   ] = 0;
            lockedDofs_["LARM_JOINT0"   ] = 1;
            lockedDofs_["LARM_JOINT1"   ] = 1;
            lockedDofs_["LARM_JOINT2"   ] = 1;
            lockedDofs_["LARM_JOINT3"   ] = 1;
            lockedDofs_["LARM_JOINT4"   ] = 1;
            lockedDofs_["LARM_JOINT5"   ] = 1;
            lockedDofs_["LARM_JOINT6"   ] = 1;
            lockedDofs_["LHAND_JOINT0"  ] = 1;
            lockedDofs_["LHAND_JOINT1"  ] = 1;
            lockedDofs_["LHAND_JOINT2"  ] = 1;
            lockedDofs_["LHAND_JOINT3"  ] = 1;
            lockedDofs_["LHAND_JOINT4"  ] = 1;
            lockedDofs_["RARM_JOINT0"   ] = 1;
            lockedDofs_["RARM_JOINT1"   ] = 1;
            lockedDofs_["RARM_JOINT2"   ] = 1;
            lockedDofs_["RARM_JOINT3"   ] = 1;
            lockedDofs_["RARM_JOINT4"   ] = 1;
            lockedDofs_["RARM_JOINT5"   ] = 1;
            lockedDofs_["RARM_JOINT6"   ] = 1;
            lockedDofs_["RHAND_JOINT0"  ] = 1;
            lockedDofs_["RHAND_JOINT1"  ] = 1;
            lockedDofs_["RHAND_JOINT2"  ] = 1;
            lockedDofs_["RHAND_JOINT3"  ] = 1;
            lockedDofs_["RHAND_JOINT4"  ] = 1;
            lockedDofs_["LLEG_JOINT0"   ] = 1;
            lockedDofs_["LLEG_JOINT1"   ] = 1;
            lockedDofs_["LLEG_JOINT2"   ] = 0;
            lockedDofs_["LLEG_JOINT3"   ] = 0;
            lockedDofs_["LLEG_JOINT4"   ] = 0;
            lockedDofs_["LLEG_JOINT5"   ] = 1;
            lockedDofs_["RLEG_JOINT0"   ] = 1;
            lockedDofs_["RLEG_JOINT1"   ] = 1;
            lockedDofs_["RLEG_JOINT2"   ] = 1;
            lockedDofs_["RLEG_JOINT3"   ] = 1;
            lockedDofs_["RLEG_JOINT4"   ] = 1;
            lockedDofs_["RLEG_JOINT5"   ] = 1;
            //--------------------------------------------------


            variableDofs_["HEAD_JOINT1"]   = 0;
            variableDofs_["CHEST_JOINT1"]  = 0;
            variableDofs_["LLEG_JOINT2"]   = 0;
            variableDofs_["LLEG_JOINT3"]   = 0;
            variableDofs_["LLEG_JOINT4"]   = 0;

            lockedDofsFixedValue_["CHEST_JOINT0"]   = 0.0;
            lockedDofsFixedValue_["HEAD_JOINT0"]    = 0.0;
            lockedDofsFixedValue_["LARM_JOINT0"   ] = 0.0;
            lockedDofsFixedValue_["LARM_JOINT1"   ] = M_PI/2;
            lockedDofsFixedValue_["LARM_JOINT2"   ] = 0.0;
            lockedDofsFixedValue_["LARM_JOINT3"   ] = 0.0;
            lockedDofsFixedValue_["LARM_JOINT4"   ] = 0.0;
            lockedDofsFixedValue_["LARM_JOINT5"   ] = 0.0;
            lockedDofsFixedValue_["LARM_JOINT6"   ] = 0.0;
            lockedDofsFixedValue_["LHAND_JOINT0"  ] = 0.0;
            lockedDofsFixedValue_["LHAND_JOINT1"  ] = 0.0;
            lockedDofsFixedValue_["LHAND_JOINT2"  ] = 0.0;
            lockedDofsFixedValue_["LHAND_JOINT3"  ] = 0.0;
            lockedDofsFixedValue_["LHAND_JOINT4"  ] = 0.0;
            lockedDofsFixedValue_["RARM_JOINT0"   ] = 0.0;
            lockedDofsFixedValue_["RARM_JOINT1"   ] = -M_PI/2;
            lockedDofsFixedValue_["RARM_JOINT2"   ] = 0.0;
            lockedDofsFixedValue_["RARM_JOINT3"   ] = 0.0;
            lockedDofsFixedValue_["RARM_JOINT4"   ] = 0.0;
            lockedDofsFixedValue_["RARM_JOINT5"   ] = 0.0;
            lockedDofsFixedValue_["RARM_JOINT6"   ] = 0.0;
            lockedDofsFixedValue_["RHAND_JOINT0"  ] = 0.0;
            lockedDofsFixedValue_["RHAND_JOINT1"  ] = 0.0;
            lockedDofsFixedValue_["RHAND_JOINT2"  ] = 0.0;
            lockedDofsFixedValue_["RHAND_JOINT3"  ] = 0.0;
            lockedDofsFixedValue_["RHAND_JOINT4"  ] = 0.0;

            lockedDofsFixedValue_["LLEG_JOINT0"   ] = 0.0;
            lockedDofsFixedValue_["LLEG_JOINT1"   ] = 0.0;
            lockedDofsFixedValue_["LLEG_JOINT5"   ] = 0.0;
            lockedDofsFixedValue_["RLEG_JOINT0"   ] = 0.0;
            lockedDofsFixedValue_["RLEG_JOINT1"   ] = 0.0;
            lockedDofsFixedValue_["RLEG_JOINT5"   ] = 0.0;
          }
        }


        virtual ConfigurationPtr_t shoot () const
        {
          static uint shoot_success = 0;
          static uint shoot_try = 0;

          ConfigurationPtr_t config (new Configuration_t (robot_->configSize ()));

          while(true){
            config->setZero();

            //##############################################################################
            //set variable joints
            //##############################################################################

            for (std::map<std::string,bool>::iterator it=variableDofs_.begin(); it!=variableDofs_.end(); ++it){
              const std::string& jn = it->first;
              JointPtr_t jj = robot_->getJointByName(jn);
              std::size_t rank = jj->rankInConfiguration ();
              jj->configuration ()->uniformlySample (rank, *config);
            }

            //##############################################################################
            //set right leg joints
            //##############################################################################

            JointPtr_t jjl;
            JointPtr_t jjr;
            std::size_t rankl;
            std::size_t rankr;

            jjl = robot_->getJointByName("LLEG_JOINT2");
            rankl = jjl->rankInConfiguration ();
            jjr = robot_->getJointByName("RLEG_JOINT2");
            rankr = jjr->rankInConfiguration ();
            (*config)[rankr] = (*config)[rankl];

            jjl = robot_->getJointByName("LLEG_JOINT3");
            rankl = jjl->rankInConfiguration ();
            jjr = robot_->getJointByName("RLEG_JOINT3");
            rankr = jjr->rankInConfiguration ();
            (*config)[rankr] = (*config)[rankl];

            jjl = robot_->getJointByName("LLEG_JOINT4");
            rankl = jjl->rankInConfiguration ();
            jjr = robot_->getJointByName("RLEG_JOINT4");
            rankr = jjr->rankInConfiguration ();
            (*config)[rankr] = (*config)[rankl];

            //##############################################################################
            //set waist joint position (free flyer)
            //##############################################################################

            jjr = robot_->getJointByName("base_joint_x");
            rankr = jjr->rankInConfiguration ();
            (*config)[rankr] = 0;
            jjr = robot_->getJointByName("base_joint_y");
            rankr = jjr->rankInConfiguration ();
            (*config)[rankr] = 0;
            jjr = robot_->getJointByName("base_joint_z");
            rankr = jjr->rankInConfiguration ();
            (*config)[rankr] = 0;
            jjr = robot_->getJointByName("base_joint_SO3");
            rankr = jjr->rankInConfiguration ();
            (*config)[rankr] = 1;
            (*config)[rankr+1] = 0;
            (*config)[rankr+2] = 0;
            (*config)[rankr+3] = 0;

            //##############################################################################
            //set fixed joints
            //##############################################################################

            for (std::map<std::string,double>::iterator it=lockedDofsFixedValue_.begin(); it!=lockedDofsFixedValue_.end(); ++it){
              const std::string& jn = it->first;
              double value = it->second;
              JointPtr_t jj = robot_->getJointByName(jn);
              std::size_t rank = jj->rankInConfiguration ();
              (*config)[rank] = value;
            }

            hpp::model::ConfigurationIn_t qq = *config.get();
            robot_->currentConfiguration(qq);
            robot_->computeForwardKinematics();

            vector3_t com = robot_->positionCenterOfMass();
            shoot_try++;

            jjr = robot_->getJointByName("LLEG_JOINT4");
            //jjr = robot_->getJointByName("CHEST_JOINT0");
            fcl::Transform3f tf = jjr->currentTransformation();
            fcl::Transform3f tfinv = tf.inverse();
            fcl::Vec3f tcom = tfinv.transform(-tf.getTranslation());
            //fcl::Vec3f tcom = -tf.getTranslation();
            //fcl::Matrix3f R = tf.getRotation();
            //fcl::Quaternion3f quat = tf.getQuatRotation();
            //double roll,p,y;
            //quat.toEuler(y,p,roll);
            //fcl::Vec3f tcom = R*t;
            //double xsole = cos(roll)*tcom[0]

            if(comIsInSupportPolygon(tcom)){ 
              shoot_success++;
              hppDout(notice, "random shots: " << shoot_success << "/" << shoot_try << " (" << (double)shoot_success/(double)shoot_try << "%)");
              hppDout(notice, "com: " << tcom[0] << "," << tcom[1] << "," <<tcom[2]);
              //hppDout(notice, "rot: " << roll << "," << p << "," << y);
              return config;
            }
          }
        }

        bool comIsInSupportPolygon (fcl::Vec3f &com) const{
          double x = com[0];
          double y = com[1];
          if(y > -0.1 && y < 0.1){
            return true;
          }else{
            return false;
          }
        }

      private:
        const DevicePtr_t& robot_;
      }; // class IrreducibleConfigurationShooter

      std::map<std::string, bool> IrreducibleConfigurationShooter::lockedDofs_;
      std::map<std::string, bool>  IrreducibleConfigurationShooter::variableDofs_;
      std::map<std::string, double> IrreducibleConfigurationShooter::lockedDofsFixedValue_;
    } //   namespace motionprior
  } //   namespace corbaserver
} // namespace hpp
