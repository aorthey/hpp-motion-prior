from numpy import dot
import numpy as np
from pylab import *
from cvxpy import *
from mpl_toolkits.mplot3d import Axes3D
from math import pi,cos,sin,acos,asin,atan

from scipy.interpolate import interp1d,splev,splrep,splprep
from scipy.misc import derivative


from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

import matplotlib.pyplot as plt

ze = np.array((0,0,1))
ye = np.array((0,1,0))
xe = np.array((1,0,0))
colorLinks = 'k'


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)


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

def drawCylinder(ax,p1,p2,linewidth=5,style='-k'):
        plt.plot([p1[0],p1[0]+p2[0]],[p1[1],p1[1]+p2[1]],[p1[2],p1[2]+p2[2]],style,linewidth=linewidth)

def drawSphere(ax,x,y,z,R):
        N = 10
        u, v = np.mgrid[0:2*np.pi:10j, 0:np.pi:10j]
        xx=np.cos(u)*np.sin(v)
        yy=np.sin(u)*np.sin(v)
        zz=np.cos(v)
        xx = R*xx + x
        yy = R*yy + y
        zz = R*zz + z
        ax.plot_surface(xx, yy, zz,  rstride=1, cstride=1, \
                        color='k',alpha=0.5, edgecolors='none')

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
        print phi,theta
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

def plotFromTo(p1,p2,style='-r',linewidth=1):
        plt.plot([p1[0],p1[0]+p2[0]],[p1[1],p1[1]+p2[1]],[p1[2],p1[2]+p2[2]],style,linewidth=linewidth)
def plotArrowFromTo(p1,p2,style='r',linewidth=1):
        a = Arrow3D([p1[0],p1[0]+p2[0]],[p1[1],p1[1]+p2[1]],[p1[2],p1[2]+p2[2]],\
                color=style, mutation_scale=20, lw=linewidth, arrowstyle="-|>")
        ax.add_artist(a)

def computeThetaGamma(f0, df0, ddf0, dddf0, ft, dft, ddft):

        AB_R = np.zeros((3,3))
        AB_R[0,:] = [ np.dot(df0,xe), np.dot(ddf0,xe), np.dot(dddf0,xe)]
        AB_R[1,:] = [ np.dot(df0,ye), np.dot(ddf0,ye), np.dot(dddf0,ye)]
        AB_R[2,:] = [ np.dot(df0,ze), np.dot(ddf0,ze), np.dot(dddf0,ze)]

        pb = ft-f0
        pbn = pb/np.linalg.norm(pb)
        pa = np.dot(AB_R.T,pbn)

        xl = np.array((-1,0,0))
        [theta,gamma] = getZYsphericalRot(pa, xl)

        BC_R = np.dot(Ry(gamma),Rz(theta))

        ### compute the frame at t
        dftnew = np.dot(AB_R,np.dot(BC_R,np.array((1,0,0))))
        ddftnew = np.dot(AB_R,np.dot(BC_R,np.array((0,1,0))))
        dddftnew = np.dot(AB_R,np.dot(BC_R,np.array((0,0,1))))

        v = dftnew
        plotFromTo(ft,dftnew,style='-r',linewidth=2)
        plotFromTo(ft,ddftnew,style='-r',linewidth=2)
        plotFromTo(ft,dddftnew,style='-r',linewidth=2)
        #plt.plot([f0[0],f0[0]+v[0]],[f0[1],f0[1]+v[1]],[f0[2],f0[2]+v[2]],'-r',linewidth=2)
        #v = ddftnew
        #plt.plot([f0[0],f0[0]+v[0]],[f0[1],f0[1]+v[1]],[f0[2],f0[2]+v[2]],'-r',linewidth=2)
        #v = dddftnew
        #plt.plot([f0[0],f0[0]+v[0]],[f0[1],f0[1]+v[1]],[f0[2],f0[2]+v[2]],'-r',linewidth=2)

        #v = 5*pb
        #plt.plot([f0[0],f0[0]+v[0]],[f0[1],f0[1]+v[1]],[f0[2],f0[2]+v[2]],'-m',linewidth=3)
        #v = pa
        #plt.plot([f0[0],f0[0]+v[0]],[f0[1],f0[1]+v[1]],[f0[2],f0[2]+v[2]],'-r',linewidth=3)

        #plt.plot([0,1],[0,0],[0,0],'-k',linewidth=1)
        #plt.plot([0,0],[0,1],[0,0],'-k',linewidth=1)
        #plt.plot([0,0],[0,0],[0,1],'-k',linewidth=1)
        return [theta,gamma,dftnew,ddftnew,dddftnew]

def computeThetaGammaFromTauNgeneralized(f,t0,length,delta):
        N = len(delta)
        theta = np.zeros((N-1,1))
        gamma = np.zeros((N-1,1))

        tcur = t0
        [fcur,dfcur,ddfcur] = funcEval(f,t0)
        dddfcur = np.cross(dfcur,ddfcur)
        assert( np.dot( dfcur, ddfcur) <= 0.001)
        assert( np.dot( dfcur,dddfcur) <= 0.001)
        assert( np.dot(ddfcur,dddfcur) <= 0.001)

        for i in range(0,N-1):
                t = tcur

                while np.linalg.norm(fcur-funcEval(f,t)[0]) < length[i]:
                        t = t-0.01
        
                tnext = t
                [fnext,dfnext,ddfnext] = funcEval(f,tnext)

                [theta[i],gamma[i],dfnext,ddfnext,dddfnext] = computeThetaGamma(fcur,
                                dfcur, ddfcur, dddfcur, fnext, dfnext, ddfnext)

                fcur = fnext
                dfcur = dfnext
                ddfcur = ddfnext
                dddfcur = np.cross(dfnext,ddfnext)
                tcur = tnext

        return [theta,gamma]

def computeThetaGammaFromTau(f,t0,length,delta):
        N = len(delta)
        theta = np.zeros((N-1,1))
        gamma = np.zeros((N-1,1))

        [f0,df0,ddf0] = funcEval(f,t0)
        dddf0 = np.cross(df0,ddf0)

        assert( np.dot(df0,ddf0) <= 0.001)
        assert( np.dot(df0,dddf0) <= 0.001)
        assert( np.dot(ddf0,dddf0) <= 0.001)

        t = t0
        while np.linalg.norm(f0-funcEval(f,t)[0]) < length[0]:
                t = t-0.01
        
        t1 = t
        [ft1,dft1,ddft1] = funcEval(f,t1)

        [theta[0],gamma[0],ndft1,nddft1,ndddft1] = computeThetaGamma(f0, df0, ddf0, dddf0, ft1, dft1, ddft1)

        t = t1
        while np.linalg.norm(ft1-funcEval(f,t)[0]) < length[0]:
                t = t-0.01

        t2 = t
        [ft2,dft2,ddft2] = funcEval(f,t2)


        [theta[1],gamma[1],nd,ndd,nddd] = computeThetaGamma(ft1, ndft1, nddft1, ndddft1, ft2, dft2, ddft2)

        #thetaOut[0] = theta
        #gammaOut[0] = gamma

        return [theta,gamma]


def forwardKinematics(ax,tau,dtau,ddtau,theta,gamma,length,delta):
        N = len(delta)

        ze = np.array((0,0,1))
        ye = np.array((0,1,0))
        xe = np.array((1,0,0))
        dtau = dtau / np.linalg.norm(dtau)
        ddtau = ddtau / np.linalg.norm(ddtau)
        dddtau = np.cross(dtau,ddtau)

        assert( np.dot(dtau,ddtau) <= 0.001)
        assert( np.dot(dtau,dddtau) <= 0.001)
        assert( np.dot(ddtau,dddtau) <= 0.001)

        Hglob = getGlobalTransformation(tau, dtau)

        H = np.zeros((N,4,4))
        HN = Hglob
        H[0,:,:] = HN
        for i in range(1,N):
                T0 = np.array((-length[i-1],0,0))
                HT0 = HTT(T0)

                #R = np.dot(Ry(gamma[i-1]),Rz(theta[i-1]))
                R = np.dot(Rz(theta[i-1]),Ry(gamma[i-1]))
                HR = HTR(R)
                HLI = dot(HR,HT0)
                #HN = dot(dot(HN,HR.T),HT0)
                HN = np.dot(HN,HLI)
                H[i,:,:] = HN

        p0t = np.array((0,0,0,1))
        p=np.zeros((N,4))

        for i in range(0,N):
                p[i,:] = dot(H[i,:,:],p0t)

        for i in range(0,N-1):
                plt.plot([p[i,0],p[i+1,0]],[p[i,1],p[i+1,1]],[p[i,2],p[i+1,2]],'-k',linewidth=5)

        for i in range(0,N):
                drawSphere(ax, p[i,0],p[i,1],p[i,2],delta[i])
        
        plotArrowFromTo(tau,dtau,style='k')

        plt.plot([p[0,0],p[0,0]+xe[0]],[p[0,1],p[0,1]+xe[1]],[p[0,2],p[0,2]+xe[2]],'-k',linewidth=1)
        plt.plot([p[0,0],p[0,0]+ye[0]],[p[0,1],p[0,1]+ye[1]],[p[0,2],p[0,2]+ye[2]],'-k',linewidth=1)
        plt.plot([p[0,0],p[0,0]+ze[0]],[p[0,1],p[0,1]+ze[1]],[p[0,2],p[0,2]+ze[2]],'-k',linewidth=1)

        #v0 = np.hstack([1.0*xe,1])
        #v0 = dot(H[0,:,:],v0)
        #plt.plot([p[0,0],v0[0]],[p[0,1],v0[1]],[p[0,2],v0[2]],'-g',linewidth=3)
        plotArrowFromTo(tau,ddtau,style='g')

        #v0 = np.hstack([-3*xe,1])
        #v0 = dot(H[0,:,:],v0)
        #plt.plot([p[0,0],v0[0]],[p[0,1],v0[1]],[p[0,2],v0[2]],'--k',linewidth=3)

        #v0 = np.hstack([1*ddtau,1])
        #v0 = dot(H[0,:,:],v0)

        #plt.plot([p[0,0],v0[0]],[p[0,1],v0[1]],[p[0,2],v0[2]],'--m',linewidth=2)
        #v0 = np.hstack([1*ye,1])
        #v0 = dot(H[0,:,:],v0)
        #plt.plot([p[0,0],v0[0]],[p[0,1],v0[1]],[p[0,2],v0[2]],'--m',linewidth=2)

        #plt.plot([p[0,0],tau[0]+dtau[0]],[p[0,1],tau[1]+dtau[1]],[p[0,2],tau[2]+dtau[2]],'-r',linewidth=2)
        #plt.plot([p[0,0],tau[0]+ddtau[0]],[p[0,1],tau[1]+ddtau[1]],[p[0,2],tau[2]+ddtau[2]],'-r',linewidth=2)
        #plt.plot([p[0,0],tau[0]+dddtau[0]],[p[0,1],tau[1]+dddtau[1]],[p[0,2],tau[2]+dddtau[2]],'-r',linewidth=2)



def funcEval(f, t):
        f0 = splev(t,f)
        df0 = splev(t,f,der=1)
        ddf0 = [-df0[1],df0[0],0.0]
        f0 = np.array(f0)
        df0 = np.array(df0)
        df0 = df0/np.linalg.norm(df0)
        ddf0 = np.array(ddf0)
        ddf0 = ddf0/np.linalg.norm(ddf0)
        return [f0,df0,ddf0]

def createSpline( t, X):
        f = interp1d(t, X, kind='cubic')
        return t,f

if __name__ == '__main__':
        fig=figure(1)
        ax = fig.gca(projection='3d')

        tau = np.array((0.8,0.0,0.0))
        dtau = np.array((0.0,1.0,0.5))
        ddtau = np.array((0.0,-0.5,1.0))

        ####################################################
        ###universal length and theta
        N = 5
        Lmax = 0.7
        deltaMax = 0.15

        #thetaAll = pi/4
        thetaAll = 0.0
        theta = thetaAll*np.ones((N-1,1)).flatten()
        theta[0]=0.0
        #gammaAll = pi/4
        gammaAll = 0.0
        gamma = gammaAll*np.ones((N-1,1)).flatten()
        length = Lmax*np.ones((N-1,1)).flatten()

        delta = deltaMax*np.ones((N,1)).flatten()
        ####################################################

        M = 10
        dM = 0.5
        currentTau = tau
        currentdTau = dtau
        currentddTau = ddtau
        
        X = np.zeros((M,3))
        dX = np.zeros((M,3))
        ddX = np.zeros((M,3))

        X[0,:] = tau
        dX[0,:] = dtau
        ddX[0,:] = ddtau

        np.random.seed(3)

        for i in range(1,M):
                lmbd = np.random.uniform(0.01,0.5)
                phi = np.random.uniform(-pi/8,pi/8)
                Rr = Rax(phi,currentdTau)
                X[i,:] = X[i-1,:]+dM*dX[i-1,:]+np.dot(Rr,lmbd*ddX[i-1,:])
                dX[i,:] = X[i,:] - X[i-1,:]
                ddX[i,:] = np.dot(Rr,ddX[i-1,:])
                dX[i,:] = dX[i,:]/np.linalg.norm(dX[i,:])
                ddX[i,:] = ddX[i,:]/np.linalg.norm(ddX[i,:])

                #robotFromConfigSimple(ax,X[i,:],dX[i,:],ddX[i,:],theta,gamma,length,delta)


        t = np.linspace(0,1,M)

        tnew = np.linspace(0,1,M*10)
        f,u = splprep(X.T)

        t0 = 1.0

        #[theta,gamma]=computeThetaGammaFromTau(f,t0,length,delta)
        [theta,gamma]=computeThetaGammaFromTauNgeneralized(f,t0,length,delta)

        [f0,df0,ddf0] = funcEval(f,t0)
        forwardKinematics(ax,f0,df0,ddf0,theta,gamma,length,delta)

        fnew = splev(tnew,f)

        plt.plot(fnew[0],fnew[1],fnew[2],'-',linewidth=35,solid_capstyle="round",alpha=0.2)
        lim = 3
        #ax.set_xlim((-lim,lim))
        #ax.set_ylim((-lim,lim))
        #ax.set_zlim((-lim,lim))
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        #ax.set_aspect('equal')
        ax.set_aspect('equal', 'datalim')
        plt.show()
        #return [t,X]


