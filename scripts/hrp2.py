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
        urdfName = "hrp2_14_capsule"
        packageName = "hrp2_irreducible"
        urdfSuffix = ""
        srdfSuffix = ""
        tf_root = "base_link"
        rootJointType = "freeflyer"

        def __init__ (self, load = True):
                Parent.__init__ (self, 'hrp2', self.rootJointType, load)

        halfSitting = \
        {"base_joint_xyz": (0.0,0.0,0.648702),
         "base_joint_SO3": (1.0, 0.0, 0.0, 0.0),
         "CHEST_JOINT0": 0.0,
         "CHEST_JOINT1": 0.0,
         "HEAD_JOINT0": 0.0,
         "HEAD_JOINT1": 0.0,
         "LARM_JOINT0": 0.261799,
         "LARM_JOINT1": 0.17453,
         "LARM_JOINT2": 0.0,
         "LARM_JOINT3": -0.523599,
         "LARM_JOINT4": 0.0,
         "LARM_JOINT5": 0.0,
         "LARM_JOINT6": 0.1,
         "LHAND_JOINT0": 0.0,
         "LHAND_JOINT1": 0.0,
         "LHAND_JOINT2": 0.0,
         "LHAND_JOINT3": 0.0,
         "LHAND_JOINT4": 0.0,
         "RARM_JOINT0": 0.261799,
         "RARM_JOINT1": -0.17453,
         "RARM_JOINT2": 0.0,
         "RARM_JOINT3": -0.523599,
         "RARM_JOINT4": 0.0,
         "RARM_JOINT5": 0.0,
         "RARM_JOINT6": 0.1,
         "RHAND_JOINT0": 0.0,
         "RHAND_JOINT1": 0.0,
         "RHAND_JOINT2": 0.0,
         "RHAND_JOINT3": 0.0,
         "RHAND_JOINT4": 0.0,
         "LLEG_JOINT0": 0.0,
         "LLEG_JOINT1": 0.0,
         "LLEG_JOINT2": -0.453786,
         "LLEG_JOINT3": 0.872665,
         "LLEG_JOINT4": -0.418879,
         "LLEG_JOINT5": 0.0,
         "RLEG_JOINT0": 0.0,
         "RLEG_JOINT1": 0.0,
         "RLEG_JOINT2": -0.453786,
         "RLEG_JOINT3": 0.872665,
         "RLEG_JOINT4": -0.418879,
         "RLEG_JOINT5": 0.0
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

        #original from antonio
        def getInitialConfig(self):
                zfloor=0.64870180180254433111
                q1=[0,1.5,zfloor,0.9249114088877176,0,0,-0.38018270043406405,0,0,0,0,0.26179900000000000393,0.1745299999999999907,0,-0.52359900000000003661,0,0,0.1745319999999999927,-0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927,0.26179900000000000393,-0.1745299999999999907,0,-0.52359900000000003661,0,0,0.1745319999999999927,-0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927,0,0,-0.45378600000000002268,0.87266500000000002402,-0.41887900000000000134,0,0,0,-0.45378600000000002268,0.87266500000000002402,-0.41887900000000000134,0]
                return q1
        ##original from antonio
        def getGoalConfig(self):
                zfloor=0.64870180180254433111
                q2=[0,-2.5,zfloor, 0.7101853756232854, 0, 0, -0.7040147244559684, 0,0,0,0,0.26179900000000000393, 0.1745299999999999907,0,-0.52359900000000003661,0,0,0.1745319999999999927, -0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927, 0.1745319999999999927,-0.1745319999999999927,0.26179900000000000393, -0.1745299999999999907,0,-0.52359900000000003661,0,0,0.1745319999999999927, -0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927, 0.1745319999999999927,-0.1745319999999999927,0,0,-0.45378600000000002268, 0.87266500000000002402,-0.41887900000000000134,0,0,0,-0.45378600000000002268, 0.87266500000000002402,-0.41887900000000000134,0]
                return q2

    #def leftHandClosed (self) :
    #    dofs = {"LARM_JOINT6": 0.1,
    #            "LHAND_JOINT0": 0.0,
    #            "LHAND_JOINT1": 0.0,
    #            "LHAND_JOINT2": 0.0,
    #            "LHAND_JOINT3": 0.0,
    #            "LHAND_JOINT4": 0.0}
    #    res = []
    #    for name, value in dofs.iteritems ():
    #        res.append ((self.rankInConfiguration [name], value))
    #    return res

    #def rightHandClosed (self) :
    #    dofs = {"RARM_JOINT6": 0.1,
    #            "RHAND_JOINT0": 0.0,
    #            "RHAND_JOINT1": 0.0,
    #            "RHAND_JOINT2": 0.0,
    #            "RHAND_JOINT3": 0.0,
    #            "RHAND_JOINT4": 0.0}
    #    res = []
    #    for name, value in dofs.iteritems ():
    #        res.append ((self.rankInConfiguration [name], value))
    #    return res
