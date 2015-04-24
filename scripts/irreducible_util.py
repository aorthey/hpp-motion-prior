from numpy import dot
import numpy as np
import math
from math import pi,cos,sin,acos,asin,atan

ze = np.array((0,0,1))
ye = np.array((0,1,0))
xe = np.array((1,0,0))
colorLinks = 'k'

def Rz(t):
        return np.array([[cos(t),-sin(t),0],[sin(t),cos(t),0],[0,0,1]])
def Ry(t):
        return np.array([[cos(t),0,sin(t)],[0,1,0],[-sin(t),0,cos(t)]])
def Rx(t):
        return np.array([[1,0,0],[0,cos(t),-sin(t)],[0,sin(t),cos(t)]])

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

def HTT(T):
        I=np.identity(3)
        return HT(I,T)

def HTR(R):
        return HT(R,np.array((0,0,0)))

def HT(R,T):
        H1 = np.vstack((R,[0,0,0]))
        H1 = np.hstack((H1,[[T[0]],[T[1]],[T[2]],[1]]))
        return H1

def getZYsphericalRot(ap, xl):
        pxy = ap-np.dot(ap,ze)*ze
        pxy = pxy/np.linalg.norm(pxy)
        pzx = ap-np.dot(ap,ye)*ye
        pzx = pzx/np.linalg.norm(pzx)
        xln = xl/np.linalg.norm(xl)

        theta = acos( np.dot(pxy,xln) )
        phi = acos( np.dot(pzx,xln) )
        if np.dot(ap,ze) < 0:
                phi = -phi
        if np.dot(ap,ye) > 0:
                theta = -theta
        return [theta,phi]


def getSphericalCoordinatesFromCart(x,y,z):
        ### compute azimuth/inclination from dtau
        r = math.sqrt(x*x + y*y + z*z)
        phi = math.acos(z/r)
        theta = math.atan2(y,x)

        ## compute inverse
        xx = r*sin(phi)*cos(theta)
        yy = r*sin(phi)*sin(theta)
        zz = r*cos(phi)

        return [theta,phi]

def getGlobalTransformation(tau, dtau):
        spheretheta,spherephi = getSphericalCoordinatesFromCart(dtau[0],dtau[1],dtau[2])
        Rglob = np.dot(Rz(spheretheta),Ry(-(math.pi/2-spherephi)))
        Hglob = HT(Rglob,tau)
        return Hglob

