import pybullet as p
import pybullet_data


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
r2d2StartPos = [0,0,0.3]
r2d2StartOrientation = p.getQuaternionFromEuler([0,0,0])
r2d2ID = p.loadURDF("r2d2.urdf", r2d2StartPos,r2d2StartOrientation)
