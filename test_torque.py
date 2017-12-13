import pybullet as p
import pybullet_data
import time
import numpy as np

BASE_ANGLE = -1.3
NECK_ANGLE = 2.5 
def reset(luxo):
    p.setJointMotorControl2(luxo,BASE_JOINT_ID,p.POSITION_CONTROL,targetPosition=BASE_ANGLE,force=maxForce)
    p.setJointMotorControl2(luxo,NECK_JOINT_ID,p.POSITION_CONTROL,targetPosition=NECK_ANGLE,force=maxForce)
    p.resetJointState(luxo,BASE_JOINT_ID,BASE_ANGLE)
    p.resetJointState(luxo,NECK_JOINT_ID,NECK_ANGLE)

def normalize(v):
    norm=np.linalg.norm(v)
    if norm==0: 
       return v,0
    return v/norm,norm
conid = p.connect(p.SHARED_MEMORY)
if (conid<0):
    p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[2,0,0])
p.setInternalSimFlags(0)
p.resetSimulation()

luxo = p.loadMJCF("torque.xml")[0]
BASE_JOINT_ID = 4 
NECK_JOINT_ID = 6 
maxForce = 100
p.loadURDF("plane.urdf",useMaximalCoordinates=True)
numJoints = p.getNumJoints(luxo)
print("Number of joints in Luxo:",numJoints)
for j in range(numJoints):
    jointInfo = p.getJointInfo(luxo,j)
    print("Joints info:",jointInfo)
for j in range(numJoints):
    linkInfo = p.getDynamicsInfo(luxo,j)
    print("Link info:",linkInfo)


useRealTime = 0
p.setRealTimeSimulation(useRealTime)
reset(luxo)
gravXid = p.addUserDebugParameter("gravityX",-10,10,0)
gravYid = p.addUserDebugParameter("gravityY",-10,10,0)
gravZid = p.addUserDebugParameter("gravityZ",-20,10,-20)
baseAngleid = p.addUserDebugParameter("baseAngle",-1000,1000,-500.0)
neckAngleid = p.addUserDebugParameter("neckAngle",-1000,1000,0.01)

STATES = [1,2,3,4,5]

steps = 0
while True:
    time.sleep(0.01)
    steps = steps+1
    gravX = p.readUserDebugParameter(gravXid)
    gravY = p.readUserDebugParameter(gravYid)
    gravZ = p.readUserDebugParameter(gravZid)
    baseAngle = p.readUserDebugParameter(baseAngleid)
    neckAngle = p.readUserDebugParameter(neckAngleid)
    p.setGravity(gravX,gravY,gravZ)
        

    if(steps < 100):
        p.setJointMotorControl2(luxo,BASE_JOINT_ID,p.POSITION_CONTROL,targetPosition=BASE_ANGLE,force=maxForce)
        p.setJointMotorControl2(luxo,NECK_JOINT_ID,p.POSITION_CONTROL,targetPosition=NECK_ANGLE,force=maxForce)
    else:
        p.setJointMotorControl2(luxo,NECK_JOINT_ID,p.TORQUE_CONTROL,force=neckAngle)
        p.setJointMotorControl2(luxo,BASE_JOINT_ID,p.TORQUE_CONTROL,force=baseAngle)

    if (useRealTime==0):
        p.stepSimulation()
