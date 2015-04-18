#/usr/bin/env python
import time
import sys
from hpp.corbaserver.motion_prior.client import Client as MPClient
#from hpp.corbaserver.wholebody_step.client import Client as WsClient
import rospy
from hrp2 import Robot 
from hpp_ros import ScenePublisher, PathPlayer
robot = Robot ()
#robot.setTranslationBounds (-0.5, 0.5, -3, 3, 0, 1)
client = robot.client

publisher = ScenePublisher(robot)
pathplayer = PathPlayer (client, publisher)

if len(sys.argv) < 2:
        print " 2014 LAAS CNRS "
        print "---------------------------------------------"
        print "usage: traj-replay.py <TRAJECTORY-FILE-NAME> <FILE2> ..."
        sys.exit()

import re
for i in range(1,len(sys.argv)):
        #m = re.search('([0-9]+)-([0-9]+)\.', sys.argv[i])
        #minutes = str(int(m.group(2))*60/100)
        #print minutes
        #posText = [2,1,0]
        #publisher.addText(">>Replaying Trajectory\nPlanning Time: "+m.group(1)+"h"+minutes+"m", posText)
        #publisher.publishObjects()
        #time.sleep(1)
        #publisher.publishObjects()
        pathplayer.fromFile(sys.argv[i])
