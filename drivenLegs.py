import pybullet as p
import pybullet_data
import math as math

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
planeId = p.loadURDF("plane.urdf")
legsStartPos = [0,0,2]
legsStartOrientation = p.getQuaternionFromEuler([0,0,0])
legsID = p.loadURDF("legs.urdf", legsStartPos,legsStartOrientation)

#drives the right thigh joint with a sin wave, left with cos
p.setJointMotorControl2(legsID, 1, controlMode=p.VELOCITY_CONTROL, force=0)
time_step = 0.01
p.setTimeStep(time_step)
steps = 10000
kp = 300
kd = 30

for step_counter in range(steps):
    t = step_counter * time_step
    #position = joint_state[0], velocity - joint_state[1]
    joint_state_1 = p.getJointState(legsID,1)
    tau_1 = kp*(math.sin(t) - joint_state_1[0]) - kd*joint_state_1[1]

    joint_state_7 = p.getJointState(legsID, 7)
    tau_7 = kp * (math.cos(t) - joint_state_7[0]) - kd *joint_state_7[1]
    p.setJointMotorControlArray(legsID, [1,7], controlMode=p.TORQUE_CONTROL, forces=[tau_1, tau_7])

    p.stepSimulation()
p.disconnect()