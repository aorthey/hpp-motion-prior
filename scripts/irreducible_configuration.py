#/usr/bin/env python
import time
from hpp.corbaserver import Client
from hpp_corbaserver.hpp import Configuration
from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hpp.corbaserver.motion_prior.client import Client as MPClient
from hrp2 import Robot
import rospy
import numpy
import math

class IrreducibleConfiguration ():
        def __init__ (self, pv):
                print "load robot model..."
                self.robot_interface = Robot ()
                self.robot_interface.setTranslationBounds (-3, 3, -3, 3, 0, 1)
                print "load client model..."
                self.cl = self.robot_interface.client
                self.robot = self.cl.robot
                self.scene_publisher = ScenePublisher (self.robot_interface.jointNames [4:])

                self.volume = pv[0]
                self.hull = pv[1]
                self.q = pv[2]
                self.robot.setCurrentConfig(self.q)

        def display(self):
                r = rospy.Rate(1)
                r.sleep()
                self.scene_publisher.oid = 0
                self.scene_publisher.addPolygon(self.hull, 0.02)
                self.scene_publisher.addWallAroundHole(self.hull)
                self.scene_publisher.publishObjects()
                self.scene_publisher(self.q)
                r.sleep()
                r.sleep()
