from numpy import dot
import numpy as np
from math import pi,cos,sin,acos,asin,atan
import time

from scipy.interpolate import interp1d,splev,splrep,splprep
from scipy.misc import derivative

from irreducible_util import *
from irreducible_plotter import *

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

        return [theta,gamma]


def forwardKinematics(tau,dtau,ddtau,theta,gamma,length,delta):
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
        return p
        

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


class IrreducibleProjector():

        def __init__(self):
                createHomogenousLinearLinkage(N,L,D)

        def __init__(self, X, Lall, Dall):
                self.L = Lall
                self.D = Dall
                self.tau = self.__computeExtendedFunction(X,Lall)

        def __computeExtendedFunction(self, X, Lall):
                tau,tmp = splprep(X.T)

                t0 = 0.0
                t1 = 1.0
                [f0,df0,ddf0] = funcEval(tau,t0)

                L0 = np.sum(Lall)
                fI = f0-L0*df0

                ##generate prefix points
                step = 0.01
                lmbda = step

                Xpre = np.array(fI)
                while lmbda < 1.0:
                        Xpre = np.vstack([Xpre,fI + lmbda*L0*df0])
                        lmbda += step

                XX = np.vstack((Xpre,X))

                tau,tmp = splprep(XX.T,s=0.001)

                ##find starting time and ending time
                self.tauStart = 0.0

                while np.linalg.norm(f0-funcEval(tau,self.tauStart)[0]) > 0.03:
                        #print np.linalg.norm(f0-funcEval(tau,self.tauStart)[0])
                        self.tauStart = self.tauStart+0.001

                self.tauEnd =1.0

                print self.tauStart,self.tauEnd
                #self.tauEnd = 1.0
                return tau

        def getJointAnglesAtT(self, t0):
                if t0 < self.tauStart or t0 > self.tauEnd:
                        print "can only evaluate in interval [",self.tauStart,",",self.tauEnd,"]"
                        sys.exit(0)
                [theta,gamma]=computeThetaGammaFromTauNgeneralized(self.tau,t0,self.L,self.D)
                return [theta,gamma]

        def plotLinearLinkageAtT(self, t0):
                if t0 < self.tauStart or t0 > self.tauEnd:
                        print "can only evaluate in interval [",self.tauStart,",",self.tauEnd,"]"
                        sys.exit(0)

                irrplot = IrreduciblePlotter()
                [theta,gamma] = self.getJointAnglesAtT(t0)
                [f0,df0,ddf0] = funcEval(self.tau,t0)
                spheres = forwardKinematics(f0,df0,ddf0,theta,gamma,self.L,self.D)
                irrplot.plotLinearLinkage(self.tau, self.tauStart, self.tauEnd, self.L, self.D, spheres)


        def plotLinearLinkageAtStart(self):
                return self.plotLinearLinkageAtT(self.tauStart)
        def plotLinearLinkageAtGoal(self):
                return self.plotLinearLinkageAtT(self.tauEnd)

        def getTimeInterval(self):
                return [self.tauStart,self.tauEnd]

        def getSublinksPositionAtT(self, t0):
                [theta,gamma] = self.getJointAnglesAtT(t0)
                [f0,df0,ddf0] = funcEval(self.tau,t0)
                S = forwardKinematics(f0,df0,ddf0,theta,gamma,self.L,self.D)

                S = np.array(S).squeeze()
                S = np.delete(S,3,axis=1)
                SL = np.hstack ((S[1:,:],self.L))
                SL = np.hstack ((SL, self.D[1:]))
                return SL

        def evaluateAtT(self, t0):
                return funcEval(self.tau,t0)[0]

        def getSublinksPositionAtRootPosition(self, froot):
                ### get time value at which function is equal to froot
                t = self.tauStart
                while np.linalg.norm(froot-funcEval(self.tau,t)[0]) > 0.03:
                        #print np.linalg.norm(froot-funcEval(self.tau,t)[0]),t,self.evaluateAtT(t),froot
                        t = t+0.001
                        if t>self.tauEnd:
                                break
                print "root link",froot,"time",t

                return self.getSublinksPositionAtT(t)

        def checkNearestPoints(self, X):
                for i in range(0,X.shape[0]):
                        xx = X[i,:]
                        dmin = 1000.0
                        tmin = 0.0
                        taumin = []
                        t = self.tauStart
                        while t<self.tauEnd:
                                d = np.linalg.norm(xx-funcEval(self.tau,t)[0])
                                if d<dmin:
                                        dmin = d
                                        tmin = t
                                        taumin = funcEval(self.tau,t)[0]
                                t = t+0.001

                        print xx,taumin,tmin,dmin




        def visualizeLinearLinkageProjection(self, timeToShowLinkage):
                tstep = 0.05
                t0 = self.tauStart
                irrplot = IrreduciblePlotter()
                while t0<=self.tauEnd:
                        [theta,gamma] = self.getJointAnglesAtT(t0)
                        [f0,df0,ddf0] = funcEval(self.tau,t0)
                        spheres = forwardKinematics(f0,df0,ddf0,theta,gamma,self.L,self.D)
                        irrplot.plotLinearLinkagePause(self.tau, self.tauStart, \
                                        self.tauEnd, self.L, self.D, spheres, \
                                        timeToShowLinkage)
                        t0 += tstep
