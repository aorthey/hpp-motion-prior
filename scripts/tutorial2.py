#from hpp.corbaserver.pr2 import Robot
from hrp2 import Robot 
robot = Robot ()
#robot.setJointBounds ("base_joint_xy", [-4, -3, -5, -3])
#robot.setJointBounds ("base_joint_xy", [-2, 0, -2, 0])
robot.setJointBounds ("base_joint_xyz", [-1, 1, -3, 3, 0, 2])

from hpp_ros import ScenePublisher
r = ScenePublisher (robot)

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
ps.resetGoalConfigs()

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:2] = [-0.0, -0.5]
#r (q_init)
q_goal [0:2] = [-0.0, 0.8]
r (q_goal)

#ps.loadObstacleFromUrdf ("iai_maps", "kitchen_area", "")

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
print ps.getGoalConfigs()
#ps.selectPathPlanner ("PRM")
ps.selectPathPlanner ("IrreduciblePlanner")
ps.solve ()


from hpp_ros import PathPlayer
pp = PathPlayer (robot.client, r)

pp (0)
pp (1)
