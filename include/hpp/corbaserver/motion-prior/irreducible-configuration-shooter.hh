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
# include <hpp/corbaserver/motion-prior/fwd.hh>
# include <hpp/model/device.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
# include <hpp/core/configuration-shooter.hh>

#define DEBUG 0
namespace hpp {
  namespace corbaserver {
    namespace motionprior {
      class HPP_CORE_DLLAPI IrreducibleConfigurationShooter :
        public hpp::core::ConfigurationShooter
      {
      static std::map<std::string, bool> lockedDofs_;
      static std::map<std::string, bool> variableDofs_;
      static std::map<std::string, double> lockedDofsFixedValue_;
      static std::map<std::string, double> lockedDofsFixedValueSubspace_;
      public:
        IrreducibleConfigurationShooter (const DevicePtr_t& robot) : robot_ (robot)
        {
          static bool firstRun = true;
          if(firstRun){
            firstRun = false;
            lockedDofs_["base_joint_xyz"]   = 0;
            //lockedDofs_["base_joint_y"]   = 1;
            //lockedDofs_["base_joint_z"]   = 1;
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

            //variableDofs_["HEAD_JOINT1"]   = 0;
            variableDofs_["CHEST_JOINT1"]  = 0;
            variableDofs_["LLEG_JOINT2"]   = 0;
            variableDofs_["LLEG_JOINT3"]   = 0;
            variableDofs_["LLEG_JOINT4"]   = 0;

            //--------------------------------------------------

            lockedDofsFixedValue_["CHEST_JOINT0"]   = 0.0;
            lockedDofsFixedValue_["HEAD_JOINT0"]    = 0.0;
            lockedDofsFixedValue_["HEAD_JOINT1"]    = 0.0;
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

            //--------------------------------------------------

            lockedDofsFixedValueSubspace_["CHEST_JOINT0"]   = 0.0;
            lockedDofsFixedValueSubspace_["HEAD_JOINT0"]    = 0.0;
            lockedDofsFixedValueSubspace_["HEAD_JOINT1"]    = 0.0;
            lockedDofsFixedValueSubspace_["LLEG_JOINT0"   ] = 0.0;
            lockedDofsFixedValueSubspace_["LLEG_JOINT1"   ] = 0.0;
            lockedDofsFixedValueSubspace_["LLEG_JOINT5"   ] = 0.0;

            //--------------------------------------------------
          }
          if(DEBUG) hppDout(notice, "Invoked IrreducibleConfigurationShooter");
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

            uint N = robot_->configSize();
            //hppDout(notice, "config size" << N);

            if (N > 40){
                    //full body setting

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

                    if(DEBUG) hppDout(notice, "fixed joints");
                    for (std::map<std::string,double>::iterator it=lockedDofsFixedValue_.begin(); it!=lockedDofsFixedValue_.end(); ++it){
                      const std::string& jn = it->first;
                      double value = it->second;
                      JointPtr_t jj = robot_->getJointByName(jn);
                      std::size_t rank = jj->rankInConfiguration ();
                      (*config)[rank] = value;
                    }
            }else{
                    //set only irreducible subspace
                    for (std::map<std::string,double>::iterator it=lockedDofsFixedValueSubspace_.begin(); it!=lockedDofsFixedValueSubspace_.end(); ++it){
                      const std::string& jn = it->first;
                      double value = it->second;
                      JointPtr_t jj = robot_->getJointByName(jn);
                      std::size_t rank = jj->rankInConfiguration ();
                      (*config)[rank] = value;
                    }
            }

            //##############################################################################
            //set fixed joints
            //##############################################################################
            //set SO(3), config is set to zero
            fcl::Quaternion3f quat;
            quat.fromEuler(0.0,0.0,0.0);

            JointPtr_t jj = robot_->getJointByName("base_joint_SO3");
            std::size_t rank = jj->rankInConfiguration ();
            (*config)[rank] = quat[0];
            (*config)[rank+1] = quat[1];
            (*config)[rank+2] = quat[2];
            (*config)[rank+3] = quat[3];

            if(DEBUG) hppDout(notice, "update");
            hpp::model::ConfigurationIn_t qq = *config.get();
            robot_->currentConfiguration(qq);
            robot_->computeForwardKinematics();

            if(DEBUG) hppDout(notice, "done");

            //##############################################################################
            //set waist joint position (free flyer)
            //##############################################################################
            //Get random x,y base_link

            fcl::Vec3f base(0,0,0); //sole position as init

            setRandomXYBaseJoints(config,base);

            fcl::Vec3f sole = computeBaseJointFromLeftSole(config, base);

            //##############################################################################
            //##############################################################################
            //##############################################################################

            shoot_try++;
            if(checkBaseLinkBounds(config)){// && comIsInSupportPolygon(base, sole)){ 
              shoot_success++;
              if(DEBUG && shoot_success%100==0) hppDout(notice, "random shots: " << shoot_success << "/" << shoot_try << " (" << (double)shoot_success/(double)shoot_try << "%)");
              return config;
            }
          }
        }


        bool checkBaseLinkBounds(ConfigurationPtr_t &config) const
        {
           //compute if ZMP (approximated as projection of base_joint_xyz) is in support polygon
           // support polygon is described by epsilon ball around the center
           // between the feet
           JointPtr_t jj;
           std::size_t rank;
           jj = robot_->getJointByName("base_joint_xyz");
           rank = jj->rankInConfiguration ();
           double xb = (*config)[rank];
           double yb = (*config)[rank+1];
           double zb = (*config)[rank+2];

           jj = robot_->getJointByName("l_sole_joint");
           fcl::Vec3f sole_translation = jj->currentTransformation().getTranslation();
           double xs = sole_translation[0];
           double ys = sole_translation[1];

           jj = robot_->getJointByName("base_joint_SO3");
           rank = jj->rankInConfiguration ();
           double q1=(*config)[rank];
           double q2=(*config)[rank+1];
           double q3=(*config)[rank+2];
           double q4=(*config)[rank+3];
           fcl::Quaternion3f quat(q1,q2,q3,q4);

           double r,p,yaw;
           quat.toEuler(yaw,p,r);
           if(DEBUG) {
                   hppDout(notice, "yaw pitch roll: "<<yaw << "|"<< p << "|"<< r);
                   hppDout(notice, "x y z" << xb << "|"<< yb << "|"<< zb);
                   hppDout(notice, "xs ys " << xs << "|" << ys);
                   hppDout(notice, "---------------------------");
           }

           //compute the zmp in perfect condition
           double t=yaw+M_PI*0.5;
           double xrot = 1.0;
           double yrot = 0.0;
           double xt = cos(t)*xrot - sin(t)*yrot;
           double yt = sin(t)*xrot + cos(t)*yrot;

           double d_sole_zmp = -0.1;
           xt *= d_sole_zmp;
           yt *= d_sole_zmp;

           //compute the real zmp
           double xr = xs+xt;
           double yr = ys+yt;

           double dist = sqrtf( (xr-xb)*(xr-xb) + (yr-yb)*(yr-yb));

           if(dist > 0.03){
                   return false;
           }
           if(zb < 0.3){
                   return false;
           }

           return true;

        }
        void setRandomXYBaseJoints(ConfigurationPtr_t &config, fcl::Vec3f &base) const
        {
            JointPtr_t jj = robot_->getJointByName("base_joint_xyz");
            std::size_t rank = jj->rankInConfiguration ();
            jj->configuration ()->uniformlySample (rank, *config);
            jj->configuration ()->uniformlySample (rank+1, *config);
            base[0] = (*config)[rank];
            base[1] = (*config)[rank+1];
            if(DEBUG) hppDout(notice, "sole is set to" << base[0] << "," << base[1]);

        }
        fcl::Vec3f& computeBaseJointFromLeftSole(ConfigurationPtr_t &config, fcl::Vec3f &base) const
        {

          JointPtr_t jj;
          jj = robot_->getJointByName("l_sole_joint");

          fcl::Transform3f t_base_sole = jj->currentTransformation()*base;

          fcl::Transform3f t_sole_base = t_base_sole.inverse();
          fcl::Quaternion3f qrot = t_sole_base.getQuatRotation();
          base = t_sole_base.getTranslation();

          JointPtr_t jjr;
          std::size_t rankr;
          jjr = robot_->getJointByName("base_joint_xyz");
          rankr = jjr->rankInConfiguration ();
          (*config)[rankr] = base[0];
          (*config)[rankr+1] = base[1];
          (*config)[rankr+2] = base[2];

          jjr = robot_->getJointByName("base_joint_SO3");
          rankr = jjr->rankInConfiguration ();
          //jjr->configuration ()->uniformlySample (rankr, *config);

          fcl::Quaternion3f quat = t_base_sole.getQuatRotation();
          double r,p,y;
          quat.toEuler(y,p,r);

          double yaw = ((float)rand()/(float)RAND_MAX)*2.0*M_PI - M_PI;
          quat.fromEuler(yaw,p,r);

          (*config)[rankr]   = quat[0];
          (*config)[rankr+1] = quat[1];
          (*config)[rankr+2] = quat[2];
          (*config)[rankr+3] = quat[3];

          if(DEBUG) {
                  hppDout(notice, "yaw: " << yaw );
                  quat.toEuler(y,p,r);
                  hppDout(notice, "rotation: " << y << " " << p << " " << r);
                  hppDout(notice, "translation: " << base[0] << " " << base[1] << " " << base[2]);
          }

          hpp::model::ConfigurationIn_t qq = *config.get();
          robot_->currentConfiguration(qq);
          robot_->computeForwardKinematics();

          jj = robot_->getJointByName("l_sole_joint");
          fcl::Vec3f sole_translation = jj->currentTransformation().getTranslation();
          return sole_translation;
        }


      private:
        const DevicePtr_t& robot_;
      }; // class IrreducibleConfigurationShooter

      std::map<std::string, bool> IrreducibleConfigurationShooter::lockedDofs_;
      std::map<std::string, bool>  IrreducibleConfigurationShooter::variableDofs_;
      std::map<std::string, double> IrreducibleConfigurationShooter::lockedDofsFixedValue_;
      std::map<std::string, double> IrreducibleConfigurationShooter::lockedDofsFixedValueSubspace_;
    } //   namespace motionprior
  } //   namespace corbaserver
} // namespace hpp
