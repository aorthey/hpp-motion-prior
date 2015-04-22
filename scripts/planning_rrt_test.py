#/usr/bin/env python
import time
from hpp.corbaserver.motion_prior.client import Client as MPClient
import rospy
import sys

from hrp2_irreducible import Robot 
robot = Robot ()
robot.setJointBounds ("base_joint_xyz", [-1, 1, -3, 3, 0, 2])

from hpp_ros import ScenePublisher, PathPlayer
publisher = ScenePublisher(robot)

from hpp.corbaserver import ProblemSolver
solver = ProblemSolver (robot)
solver.resetGoalConfigs()
#robot.setTranslationBounds (-0.5, 0.5, -3, 3, 0, 1)
#robot.setJointBounds ("base_link", [-0.5, 0.5])

#names = robot.getJointNames()
#q_init = robot.getInitialConfig ()
#print q_init
#q_goal = q_init [::]
#zfloor = 0.648
#q_init[0:3] = [-0.2, 0.5, zfloor]
#q_goal[0:3] = [0.3, 0.8, zfloor]

#original from antonio
zfloor=0.64870180180254433111
#zfloor=0.74970180180254433111
#q1=[0,1.5,zfloor,0.9249114088877176,0,0,-0.38018270043406405,0,0,0,0,0.26179900000000000393,0.1745299999999999907,0,-0.52359900000000003661,0,0,0.1745319999999999927,-0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927,0.26179900000000000393,-0.1745299999999999907,0,-0.52359900000000003661,0,0,0.1745319999999999927,-0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927,0,0,-0.45378600000000002268,0.87266500000000002402,-0.41887900000000000134,0,0,0,-0.45378600000000002268,0.87266500000000002402,-0.41887900000000000134,0]
#
##original from antonio
#q2=[0,-2.5,zfloor, 0.7101853756232854, 0, 0, -0.7040147244559684, 0,0,0,0,0.26179900000000000393, 0.1745299999999999907,0,-0.52359900000000003661,0,0,0.1745319999999999927, -0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927, 0.1745319999999999927,-0.1745319999999999927,0.26179900000000000393, -0.1745299999999999907,0,-0.52359900000000003661,0,0,0.1745319999999999927, -0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927, 0.1745319999999999927,-0.1745319999999999927,0,0,-0.45378600000000002268, 0.87266500000000002402,-0.41887900000000000134,0,0,0,-0.45378600000000002268, 0.87266500000000002402,-0.41887900000000000134,0]

q1r = [0.9249114088877176,0,0,-0.38018270043406405]
q2r = [0.7101853756232854, 0, 0, -0.7040147244559684]

q1=[0,1.5,zfloor,q1r[0],q1r[1],q1r[2],q1r[3],0.0,0.0,0.0,0.0,0.0,0.0,-0.45,0.87,-0.41,0.0]
q2=[0,-2.5,zfloor,q2r[0],q2r[1],q2r[2],q2r[3],0.0,0.0,0.0,0.0,0.0,0.0,-0.45,0.87,-0.41,0.0]

q1r = [1,0,0,0]
q1=[0,0.0,zfloor,q1r[0],q1r[1],q1r[2],q1r[3],0.0,0.0,0.0,0.0,0.0,0.0,-0.0,0.0,0.0,0.0]
q2 = q1[::]
q2[1] += 0.1

#qI = robot.shootRandomConfig()
#qG = robot.shootRandomConfig()

qI = robot.getInitialConfig()
qG = robot.getGoalConfig()

#qI = q1
#qG = q2

#solver.loadObstacleFromUrdf("hpp-motion-prior","floor","")
#solver.loadObstacleFromUrdf("hpp-motion-prior","floor_obstacle","")
#solver.loadObstacleFromUrdf("hpp-motion-prior","wall","")
## Add constraints
#qI[1]=1.0

solver.setInitialConfig (qI)
solver.addGoalConfig (qG)

#publisher(q)
#print q
#sys.exit(0)

qqinit = solver.getInitialConfig()[0:3]
qqgoal = solver.getGoalConfigs()[0][0:3]
print "planning from",qqinit,"to",qqgoal
#print "Irreducible Planner"
#solver.selectPathPlanner("PRM")
#solver.selectPathPlanner("DiffusingPlanner")
print "solve"
start_time = float(time.time())

solver.selectPathPlanner("IrreduciblePlanner")
solver.solve ()

end_time = float(time.time())
seconds = end_time - start_time
hours = seconds/3600
print("--- %s seconds ---" % seconds)
print("--- %s hours   ---" % hours)
fname = "../data-traj/rrt-wall.tau"

pathplayer = PathPlayer (robot.client, publisher)

pathplayer(0)
pathplayer(1)
pathplayer.toFile(1,fname)
print "path was written to file",fname
