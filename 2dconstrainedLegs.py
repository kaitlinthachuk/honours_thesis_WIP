import pybullet as p
import pybullet_data
import math as math

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
planeId = p.loadURDF("plane.urdf")
legsStartPos = [0,0,2]
p.setGravity(0,0,-10)
legsStartOrientation = p.getQuaternionFromEuler([0,1,0])
legsID = p.loadMJCF("legs.mjcf")[0]
p.resetBasePositionAndOrientation(legsID, legsStartPos, legsStartOrientation)

# gives free body ragdoll motion
numJoints = p.getNumJoints(legsID)
maxForce = [0] * 6
mode = p.VELOCITY_CONTROL
p.setJointMotorControlArray(legsID, [5,8,11,14,17,20], controlMode=mode, forces=[0,0,0,0,0,0])
p.setRealTimeSimulation(1)

# import pybullet as p
# import pybullet_data
# import time
# import math
#
# p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
# p.loadURDF("plane.urdf")
# cubeId = p.loadURDF("cube_small.urdf",0,0,1)
# p.setGravity(0,0,-10)
# p.setRealTimeSimulation(1)
# cid = p.createConstraint(cubeId,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,1])
# prev=[0,0,1]
# a=-math.pi
# while 1:
# 	a=a+0.01
# 	if (a>math.pi):
# 		a=-math.pi
# 	time.sleep(.01)
# 	p.setGravity(0,0,-10)
# 	pivot=[a,0,1]
# 	orn = p.getQuaternionFromEuler([a,0,0])
# 	p.changeConstraint(cid,pivot,jointChildFrameOrientation=orn, maxForce=50)
#
# p.removeConstraint(cid)