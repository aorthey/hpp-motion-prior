#/usr/bin/env python
import time
from hpp.corbaserver.motion_prior.client import Client as MPClient
import rospy
from hrp2 import Robot 
from hpp_ros import ScenePublisher

print "load mpc model..."
mpc = MPClient()
pc = mpc.precomputation

robot = Robot ()
robot.setTranslationBounds (-0.5, 0.5, -3, 3, 0, 1)
q = robot.getInitialConfig()

sp = ScenePublisher(robot)

mpc = MPClient()
pc = mpc.precomputation
cnames = pc.getNumericalConstraints (q)

#pv = ProjectedVolume()
for i in range(1,100):
  q = pc.getRandomConfiguration()
  r = rospy.Rate(1)
  r.sleep()
  sp(q)
