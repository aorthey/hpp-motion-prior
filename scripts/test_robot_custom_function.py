#/usr/bin/env python
import time
from hpp.corbaserver import Client
from hpp_corbaserver.hpp import Configuration
from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hpp.corbaserver.client import Client as WsClient
from hrp2 import Robot
from math import pi

#################################################################

robot_interface = Robot ()
robot_interface.setTranslationBounds (-3, 3, -3, 3, 0, 1)

cl = robot_interface.client
robot = robot_interface.client.robot

#################################################################
# publish half sitting position in rviz
#################################################################

r = ScenePublisher (robot_interface.jointNames [4:])
q0 = robot_interface.getInitialConfig ()
q1 = [0.0, 0.0, 0.705, 1.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

robot.setCurrentConfig(q1)
r(q0)
time.sleep(0.2)
print robot.getNumberDof()
print robot.getNumberDof()

capsulePos = robot.computeVolume()
#access 3 elements at a time (x,y,z)
scale = 0.5
for i in range(0,len(capsulePos),3):
        r.addSphere(scale*capsulePos[i], scale*capsulePos[i+1], scale*capsulePos[i+2])

r.publishObjects()
print "finished"

#################################################################
