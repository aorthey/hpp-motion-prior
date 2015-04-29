from irreducible_util import *
from numpy import dot
import numpy as np
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
from math import pi,cos,sin,acos,asin,atan

from scipy.interpolate import interp1d,splev,splrep,splprep
from scipy.misc import derivative

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

import matplotlib.pyplot as plt

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

def drawCylinder(ax,p1,p2,linewidth=5,style='-k'):
        plt.plot([p1[0],p1[0]+p2[0]],[p1[1],p1[1]+p2[1]],[p1[2],p1[2]+p2[2]],style,linewidth=linewidth)


class IrreduciblePlotter():
        def __init__(self):
                self.fig=figure(1)
                self.ax = self.fig.gca(projection='3d')

        def __drawSphere(self,x,y,z,R):
                N = 10
                u, v = np.mgrid[0:2*np.pi:10j, 0:np.pi:10j]
                xx=np.cos(u)*np.sin(v)
                yy=np.sin(u)*np.sin(v)
                zz=np.cos(v)
                xx = R*xx + x
                yy = R*yy + y
                zz = R*zz + z
                self.ax.plot_surface(xx, yy, zz,  rstride=1, cstride=1, \
                                color='k',alpha=0.5, edgecolors='none')

        def __plotSpheres(self,tau, t0, t1, L, D, p):
                self.ax = self.fig.gca(projection='3d')
                N = p.shape[0]
                ## create 
                M = 50
                tnew = np.linspace(0,t1,M)
                taunew = splev(tnew,tau)
                plt.plot(taunew[0],taunew[1],taunew[2],'-',linewidth=35,solid_capstyle="round",alpha=0.2)

                for i in range(0,N-1):
                        plt.plot([p[i,0],p[i+1,0]],[p[i,1],p[i+1,1]],[p[i,2],p[i+1,2]],'-k',linewidth=5)

                for i in range(0,N):
                        self.__drawSphere(p[i,0],p[i,1],p[i,2],D[i])
                
                #plotArrowFromTo(tau,dtau,style='k')

                plt.plot([p[0,0],p[0,0]+xe[0]],[p[0,1],p[0,1]+xe[1]],[p[0,2],p[0,2]+xe[2]],'-k',linewidth=1)
                plt.plot([p[0,0],p[0,0]+ye[0]],[p[0,1],p[0,1]+ye[1]],[p[0,2],p[0,2]+ye[2]],'-k',linewidth=1)
                plt.plot([p[0,0],p[0,0]+ze[0]],[p[0,1],p[0,1]+ze[1]],[p[0,2],p[0,2]+ze[2]],'-k',linewidth=1)

                #plotArrowFromTo(tau,ddtau,style='g')
                self.ax.set_xlabel('X')
                self.ax.set_ylabel('Y')
                self.ax.set_zlabel('Z')

        def plotLinearLinkage(self,tau, t0, t1, L, D, p):
                plt.clf()
                self.__plotSpheres(tau,t0,t1,L,D,p)
                self.ax.set_aspect('equal', 'datalim')
                plt.show()

        def plotLinearLinkagePause(self,tau, t0, t1, L, D, p, timeToShowLinkage):
                plt.clf()
                self.__plotSpheres(tau,t0,t1,L,D,p)
                lim = 0.5
                self.ax.set_xlim((-lim,lim))
                self.ax.set_ylim((-lim,lim))
                self.ax.set_zlim((-lim,lim))
                plt.pause(timeToShowLinkage)
