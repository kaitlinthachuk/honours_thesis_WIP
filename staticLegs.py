import pybullet as p
import pybullet_data

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
planeId = p.loadURDF("plane.urdf")
p.setGravity(0,0,-10)
legsStartPos = [0,0,0.3]
legsStartOrientation = p.getQuaternionFromEuler([0,0,0])
legsID = p.loadURDF("legs.urdf", legsStartPos,legsStartOrientation)
#p.setRealTimeSimulation(1)
