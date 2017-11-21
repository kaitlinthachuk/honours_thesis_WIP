import pybullet as p
import pybullet_data


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
planeId = p.loadURDF("plane.urdf")
p.setGravity(0,0,-10)
legsStartPos = [0,0,2]
legsStartOrientation = p.getQuaternionFromEuler([0,0,0])
legsID = p.loadURDF("legs.urdf", legsStartPos,legsStartOrientation)

# gives free body ragdoll motion
numJoints = p.getNumJoints(legsID)
maxForce = [0] * numJoints
mode = p.VELOCITY_CONTROL
p.setJointMotorControlArray(legsID, range(0, numJoints), controlMode=mode, forces=maxForce)
p.setRealTimeSimulation(1)




