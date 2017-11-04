import pybullet as p
import pybullet_data

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,5]
cubeStartOrientation = p.getQuaternionFromEuler([1,10,5])
cubeId = p.loadURDF("cube.urdf", cubeStartPos,cubeStartOrientation)
p.setRealTimeSimulation(1)
