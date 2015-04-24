import time
import sys
import numpy as np
#from hpp.corbaserver.motion_prior.client import Client as MPClient
#from hpp.corbaserver.wholebody_step.client import Client as WsClient
import rospy
#import ipdb; ipdb.set_trace()
import pickle as pk
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
    

fname = "../data-traj/rrt-floor-irreducible.tau"
import re
tau = getTrajFromFile(fname)

tau = np.array(tau)
M = len(tau[0])
N = len(tau[0][0])
A = np.array(tau).flatten().reshape(M,N)
X =  A[:,0:3]

L = np.array((0.25,0.25,0.25))
D = np.array((0.08,0.08,0.08,0.08))
P = IrreducibleProjector(X,L,D)

P.getJointAnglesAtT(0.32)
#P.plotLinearLinkageAtT(0.32)
P.visualizeLinearLinkageProjection(0.0001)
