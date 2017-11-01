import pybullet as p
import pybullet_data
from time import sleep

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,20]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
cubeId = p.loadURDF("cube.urdf", cubeStartPos,cubeStartOrientation)
p.setRealTimeSimulation(1)
sleep(10)