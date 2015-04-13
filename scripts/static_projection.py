#/usr/bin/env python

from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hpp.corbaserver.hrp2 import Robot
from hpp.corbaserver.wholebody_step.client import Client as WsClient
import time
import sys


Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'

robot = Robot ('hrp2_14')
robot.setTranslationBounds (-3, 3, -3, 3, 0, 1)
cl = robot.client

r = ScenePublisher (robot.jointNames [4:])
q0 = robot.getInitialConfig ()
r (q0)

# Add constraints
wcl = WsClient ()


var1 = float(sys.argv[2])
var2 = float(sys.argv[3])
var3 = float(sys.argv[4])
print 'vars=', var1, var2, var3

point1=[0,0,0]
point2=[var1, var2, var3]

rotation=[1.5, 0, 0]

reussite = cl.problem.createOrientationConstraint ("orientation_relative", "LARM_JOINT6", "RARM_JOINT6", rotation)

reussite = cl.problem.createPositionConstraint ("position_absolute", sys.argv[1], "", point1, point2)



wcl.problem.addStaticStabilityConstraints ("balance", q0, robot.leftAnkle,
                                           robot.rightAnkle)
cl.problem.setNumericalConstraints ("balance", ["balance/relative-com",
                                                "balance/relative-orientation",
                                                "balance/relative-position",
                                                "balance/orientation-left-foot",
                                                "balance/position-left-foot",
																								"orientation_relative",
																								"position_absolute"])

# lock hands in closed position11
lockedDofs = robot.leftHandClosed ()
for name, value in lockedDofs.iteritems ():
    cl.problem.lockDof (name, value, 0, 0)

lockedDofs = robot.rightHandClosed ()
for name, value in lockedDofs.iteritems ():
    cl.problem.lockDof (name, value, 0, 0)


#q1 = [0.0, 0.0, 0.705, 1.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

(ret, q1proj, residualError) = cl.problem.applyConstraints (q0)

#q2 = [0.0, 0.0, 0.705, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]


#q2proj = cl.problem.applyConstraints (q2)
#cl.problem.setInitialConfig (q1proj)
#cl.problem.addGoalConfig (q2proj)
#cl.problem.solve ()


q = robot.client.robot.getCurrentConfig()
r(q)
r(q)
r(q)
r(q)

time.sleep(1)

r(q)

print 'reussite=', ret, '   erreur=', residualError


#p = PathPlayer (cl, r)
#p (1)
