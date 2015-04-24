#/usr/bin/env python
import time
import sys
import numpy as np
#from hpp.corbaserver.motion_prior.client import Client as MPClient
#from hpp.corbaserver.wholebody_step.client import Client as WsClient
import rospy
from math import atan2,pi,asin
import pickle as pk
from hrp2 import Robot 
from hpp_ros import ScenePublisher, PathPlayer
from irreducible_projector import IrreducibleProjector

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
        fname = "../data-traj/rrt-wall-0-time-3-62.tau"
        #sys.exit()
else:
        fname = sys.argv[1]

import re
#fromFile(publisher,fname)
tau = getTrajFromFile(fname)

tau = np.array(tau)
M = len(tau[0])
N = len(tau[0][0])
X = np.array(tau).flatten().reshape(M,N)

L = np.array((0.25,0.25,0.25))
D = np.array((0.08,0.08,0.08,0.08))
P = IrreducibleProjector(X[:,0:3],L,D)

t0,t1 = P.getTimeInterval()

def setLARM(q,theta,yaw):
        q[11]=-1.57
        q[12]=theta[0]+1.57+yaw+3.14
        q[13]=-1.57
        q[14]=theta[1]
        q[15]=0.0
        q[16]=theta[2]
        q[17]=0.1
        return q

def setRARM(q,theta,yaw):
        q[23]=-1.57
        q[24]=theta[0]-1.57+(yaw+3.14)
        q[25]=-1.57
        q[26]=theta[1]
        q[27]=0.0
        q[28]=theta[2]
        q[29]=0.1
        return q

##map [0,N] -> [t0,t1]
N = X.shape[0]
for i in range(0,N):
        ##[0,N] -> [0,N]/N -> [0,1]*(t1-t0) -> [0,t1-t0] -> [t0,t1]
        t=i*(1/N)*(t1-t0)+t0
        [theta,gamma]=P.getJointAnglesAtT(t)

        q = X[i,:]
        qt = q[3:7]

        x=qt[0]
        y=qt[1]
        z=qt[2]
        w=qt[3]
        roll = atan2(-2*y*z+2*x*w,1-2*x*x-2*y*y)
        pitch = asin(2*x*z+2*y*w)
        yaw = atan2(-2*x*y+2*z*w, 1-2*y*y-2*z*z)
        print yaw,roll,pitch,qt
        #theta[0]=theta[0]+yaw-pi/2
        q = setLARM(q,theta,roll)
        q = setRARM(q,theta,roll)
        #sys.exit(0)

        publisher(q)
        time.sleep (dt)
