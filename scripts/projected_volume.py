#/usr/bin/env python
import time
from hpp.corbaserver import Client
from hpp_ros import ScenePublisher
from hpp.corbaserver.motion_prior.client import Client as MPClient
from hpp.corbaserver.hrp2 import Robot 
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.wholebody_step.client import Client as WsClient
import rospy
import numpy
import math


class ProjectedVolume ():
        def __init__ (self):
                print "load robot model..."
                Robot.urdfSuffix = '_capsule'
                Robot.srdfSuffix= '_capsule'
                robot = Robot ('hrp2_14')

                self.robot_interface = Robot ()
                self.robot_interface.setTranslationBounds (-3, 3, -3, 3, 0, 1)
                print "load client model..."
                self.cl = self.robot_interface.client

                print "load mpc model..."
                self.mpc = MPClient()
                self.precomputation = self.mpc.precomputation

                self.robot = self.cl.robot

                print "load scene model..."
                self.scene_publisher = ScenePublisher (self.robot_interface.jointNames [4:])
                self.q0 = self.robot_interface.getInitialConfig ()
                self.q = self.q0
                self.distanceToProjection = 1
                self.vol_cvx_hull = []

                cnames = self.precomputation.addNaturalConstraints ( "natural-constraints" , self.q, "LLEG_JOINT5", "RLEG_JOINT5")
                self.cl.problem.setNumericalConstraints ("natural-constraints", cnames)
                print cnames

        def displayRandomConvexHull(self):
                self.setRandomConfig()
                self.projectConfigurationUntilIrreducible()
                self.displayRobot()
                self.displayConvexHullOfProjectedCapsules()

        def projectOnConstraintsManifold(self, q_in):

                status, qproj, residual = self.cl.problem.applyConstraints (q_in)
                if status==False or residual > 0.0001:
                  print "[WARNING] Projection Error on constraints manifold: ",residual," successful: ", status
                return qproj, status

        def setConfig(self, q_in):
                self.q = q_in
                self.robot.setCurrentConfig(self.q)

        def setRandomConfig(self):
                self.q_old = self.q

                q_new = self.robot.getRandomConfig()
                #do not change com
                q_new[0]=-0.1
                q_new[1]=0
                q_new[2]=0.7
                #90d rotation around z-axis
                q_new[3]=1
                q_new[4]=0
                q_new[5]=0
                q_new[6]=1

                #q_new, status= self.projectOnConstraintsManifold(q_new)

                #while status==False:
                #        q_new = self.robot.getRandomConfig()
                #        q_new, status= self.projectOnConstraintsManifold(q_new)

                self.q_old = q_new
                self.q = q_new
                self.setConfig(self.q)

        def displayConvexHullOfProjectedCapsules(self):
                self.hull = self.precomputation.getConvexHullCapsules()
                self.hull = zip(*[iter(self.hull)]*3)
                print self.precomputation.getVolume()

                r = rospy.Rate(1)
                r.sleep()
                self.scene_publisher.oid = 0
                #self.scene_publisher.addPolygonFilled(self.hull)
                self.scene_publisher.addPolygon(self.hull, 0.02)
                self.scene_publisher.addWallAroundHole(self.hull)
                self.scene_publisher.publishObjects()
                r.sleep()

        def projectConfigurationUntilIrreducibleConstraint(self):
                self.precomputation.setCurrentConfiguration(self.q)
                self.q = self.precomputation.projectUntilIrreducibleConstraint()
                self.setConfig(self.q)

       # def projectConfigurationUntilIrreducible(self):
       #         self.precomputation.setCurrentConfiguration(self.q)
       #         for i in range(1,20):
       #                 self.q_new = self.precomputation.projectUntilIrreducibleOneStep()
       #                 self.q_new = self.projectOnConstraintsManifold(self.q_new)
       #         self.setConfig(self.q_new)

       # def projectConfigurationUntilIrreducibleOneStep(self):
       #         self.precomputation.setCurrentConfiguration(self.q)
       #         self.q_grad = self.precomputation.getGradient()
       #         self.q_grad_raw = self.q_grad
       #         self.q_grad.insert(3,0)
       #         self.q_grad[0]=0
       #         self.q_grad[1]=0
       #         self.q_grad[2]=0
       #         #make sure that quaternions are not changed
       #         self.q_grad[3]=0
       #         self.q_grad[4]=0
       #         self.q_grad[5]=0
       #         self.q_grad[6]=0
       #         self.q_new = [x-0.1*y for x,y in zip(self.q,self.q_grad)]
       #         #print self.q_new[0:8]
       #         self.q_new = self.projectOnConstraintsManifold(self.q_new)
       #         self.setConfig(self.q_new)
                
        def displayRobot(self):
                r = rospy.Rate(1)
                r.sleep()
                self.scene_publisher(self.q)
