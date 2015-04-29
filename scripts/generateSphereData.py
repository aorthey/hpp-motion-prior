from numpy import dot
import numpy as np
#from cvxpy import *
from math import pi,cos,sin,acos,asin,atan
#from scipy.interpolate import interp1d,splev,splrep,splprep
#from scipy.misc import derivative
####################################################
###universal length and theta
delta0 = 0.15
Mpts = 20
pts_stepsize = 0.5
kappa = 1.88
fname = "../data-traj/spheretraj.txt"
####################################################

ze = np.array((0,0,1))
ye = np.array((0,1,0))
xe = np.array((1,0,0))

def Rax(theta, u):
    return [[cos(theta) + u[0]**2 * (1-cos(theta)), 
             u[0] * u[1] * (1-cos(theta)) - u[2] * sin(theta), 
             u[0] * u[2] * (1 - cos(theta)) + u[1] * sin(theta)],
            [u[0] * u[1] * (1-cos(theta)) - u[2] * sin(theta),
             cos(theta) + u[1]**2 * (1-cos(theta)),
             u[1] * u[2] * (1 - cos(theta)) + u[0] * sin(theta)],
            [u[0] * u[2] * (1-cos(theta)) - u[1] * sin(theta),
             u[1] * u[2] * (1-cos(theta)) - u[0] * sin(theta),
             cos(theta) + u[2]**2 * (1-cos(theta))]]

##start configuration of sphere
tau = np.array((0.8,0.0,0.0))
dtau = np.array((0.0,1.0,0.5))
ddtau = np.array((0.0,-0.5,1.0))

currentTau = tau
currentdTau = dtau
currentddTau = ddtau

X = np.zeros((Mpts,3))
dX = np.zeros((Mpts,3))
ddX = np.zeros((Mpts,3))

X[0,:] = tau
dX[0,:] = dtau
ddX[0,:] = ddtau

np.random.seed(3)
for i in range(1,Mpts):
        lmbd = np.random.uniform(0.01,0.5)
        phi = np.random.uniform(-pi/8,pi/8)
        Rr = Rax(phi,currentdTau)
        X[i,:] = X[i-1,:]+pts_stepsize*dX[i-1,:]+np.dot(Rr,lmbd*ddX[i-1,:])
        dX[i,:] = X[i,:] - X[i-1,:]
        ddX[i,:] = np.dot(Rr,ddX[i-1,:])
        dX[i,:] = dX[i,:]/np.linalg.norm(dX[i,:])
        ddX[i,:] = ddX[i,:]/np.linalg.norm(ddX[i,:])

fh = open(fname,"w")
fh.write('## x y z kappa delta0\n')
for i in range(1,Mpts):
        fh.write('%.4f %.4f %.4f %.4f %.4f\n' % (X[i,0],X[i,1],X[i,2],kappa,delta0))
fh.close()
