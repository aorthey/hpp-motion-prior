#/usr/bin/env python
import time
import sys
import numpy as np
#from hpp.corbaserver.motion_prior.client import Client as MPClient
#from hpp.corbaserver.wholebody_step.client import Client as WsClient
import rospy
import pickle as pk
from hrp2 import Robot 
from hpp_ros import ScenePublisher, PathPlayer

dt = 0.01

def getTrajFromFile(fname):
        fh = open(fname,"rb")                                                                                                             
        tau = []
        while 1:
            try:
                tau.append(pk.load(fh))
            except EOFError:
                break
        fh.close()
        return tau
    
def fromFile(publisher,fname):
        tau = getTrajFromFile(fname)
        for tauK in tau:
                for q in tauK:
                        publisher.robotConfig = q
                        publisher.publishRobots ()
                        time.sleep (dt)
                        sys.exit(0)

robot = Robot ()
#robot.setTranslationBounds (-0.5, 0.5, -3, 3, 0, 1)
#client = robot.client

publisher = ScenePublisher(robot)
#pathplayer = PathPlayer (client, publisher)

if len(sys.argv) < 2:
        #print " 2014 LAAS CNRS "
        #print "---------------------------------------------"
        #print "usage: traj-replay.py <TRAJECTORY-FILE-NAME>"
        fname = "../data-traj/rrt-wall.tau"
        fname = "../data-traj/rrt-floor-irreducible.tau"
        #sys.exit()
else:
        fname = sys.argv[1]

import re
#fromFile(publisher,fname)
tau = getTrajFromFile(fname)

tau = np.array(tau)
M = len(tau[0])
N = len(tau[0][0])
A = np.array(tau).flatten().reshape(M,N)
print A[0:8,0:3]



for tauK in tau:
        for q in tauK:
                #qq = q[::]
                #qq[0] = qq[0:3]
                #qq[1] = qq[3:7]
                #for i in range(2,7):
                #        del qq[i]
                ##to change
                ### LARM_JOINT3
                ### LARM_JOINT5
                #publisher.robotConfig = q
                #publisher.publishRobots ()
                publisher(q)
                #q=setLeftArm(qq)
                Z= zip(q,robot.getJointNames())
                sys.exit(0)
                time.sleep (dt)
