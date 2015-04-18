#/usr/bin/env python
import time
from hpp.corbaserver.motion_prior.client import Client as MPClient
import rospy
from hrp2 import Robot 
from hpp_ros import ScenePublisher, PathPlayer
from hpp.corbaserver import ProblemSolver

robot = Robot ()
robot.setTranslationBounds (-0.5, 0.5, -3, 3, 0, 1)
client = robot.client

q1=[0,1.5,0.64870180180254433111, 0.9249114088877176,0,0,-0.38018270043406405,0,0,0,0,0.26179900000000000393,0.1745299999999999907,0,-0.52359900000000003661,0,0,0.1745319999999999927,-0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927,0.26179900000000000393,-0.1745299999999999907,0,-0.52359900000000003661,0,0,0.1745319999999999927,-0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927,0,0,-0.45378600000000002268,0.87266500000000002402,-0.41887900000000000134,0,0,0,-0.45378600000000002268,0.87266500000000002402,-0.41887900000000000134,0]
q2=[0,-2.5,0.64870180180254433111, 0.7101853756232854, 0, 0, -0.7040147244559684, 0,0,0,0,0.26179900000000000393, 0.1745299999999999907,0,-0.52359900000000003661,0,0,0.1745319999999999927, -0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927, 0.1745319999999999927,-0.1745319999999999927,0.26179900000000000393, -0.1745299999999999907,0,-0.52359900000000003661,0,0,0.1745319999999999927, -0.1745319999999999927,0.1745319999999999927,-0.1745319999999999927, 0.1745319999999999927,-0.1745319999999999927,0,0,-0.45378600000000002268, 0.87266500000000002402,-0.41887900000000000134,0,0,0,-0.45378600000000002268, 0.87266500000000002402,-0.41887900000000000134,0]

publisher = ScenePublisher(robot)

## Add constraints

for i in range(0,10):
    solver = ProblemSolver (robot)
    print "Planning for Problem %d" % i

    solver.selectPathPlanner("DiffusingPlanner")
    solver.loadObstacleFromUrdf("hpp_ros","wall")
    solver.setInitialConfig (q1)
    solver.addGoalConfig (q2)
    
    start_time = float(time.time())
    solver.solve ()
    end_time = float(time.time())
    seconds = end_time - start_time
    hours = seconds/3600
    print("--- %s hours   ---" % hours)
    minutes = (hours-int(hours))*60
    fname = "../data-traj/rrt-wall-%d-time-%dh-%sm.tau" %(i,int(hours),int(minutes))
    pathplayer = PathPlayer (client, publisher)
    pathplayer.toFile(1,fname)
