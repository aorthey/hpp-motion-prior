#!/usr/bin/env python
# Copyright (c) 2014 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-corbaserver.
# hpp-corbaserver is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-corbaserver is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-corbaserver.  If not, see
# <http://www.gnu.org/licenses/>.

from hpp.corbaserver.robot import Robot as Parent

class Robot (Parent):
        urdfName = "hrp2_14_capsule_irreducible"
        #packageName = "hpp-motion-prior"
        packageName = "hrp2_irreducible"
        #urdfName = "hrp2_14_capsule"
        urdfSuffix = ""
        srdfSuffix = ""
        tf_root = "base_link"
        rootJointType = "freeflyer"

        def __init__ (self, load = True):
                Parent.__init__ (self, 'hrp2-irreducible', self.rootJointType, load)

        halfSitting = \
        {"base_joint_xyz": (0.0,0.0,0.648702),
         "base_joint_SO3": (1.0, 0.0, 0.0, 0.0),
         "CHEST_JOINT0": 0.0,
         "CHEST_JOINT1": 0.0,
         "HEAD_JOINT0": 0.0,
         "HEAD_JOINT1": 0.0,
         "LLEG_JOINT0": 0.0,
         "LLEG_JOINT1": 0.0,
         "LLEG_JOINT2": -0.453786,
         "LLEG_JOINT3": 0.872665,
         "LLEG_JOINT4": -0.418879,
         "LLEG_JOINT5": 0.0,
         }
    #def getInitialConfig (self):
    #    q = []
    #    for n in self.jointNames:
    #        dof = self.halfSitting [n]
    #        if type (dof) is tuple:
    #            q += dof
    #        else:
    #            q.append (dof)
    #    return q

        def getInitialConfig(self):
                zfloor=0.64870180180254433111
                q1r = [0.9249114088877176,0,0,-0.38018270043406405]
                q1=[0,1.5,zfloor,q1r[0],q1r[1],q1r[2],q1r[3],0.0,0.0,0.0,0.0,0.0,0.0,-0.45,0.87,-0.41,0.0]
                return q1

        def getGoalConfig(self):
                zfloor=0.64870180180254433111
                q2r = [0.7101853756232854, 0, 0, -0.7040147244559684]
                q2=[0,-2.5,zfloor,q2r[0],q2r[1],q2r[2],q2r[3],0.0,0.0,0.0,0.0,0.0,0.0,-0.45,0.87,-0.41,0.0]
                return q2
