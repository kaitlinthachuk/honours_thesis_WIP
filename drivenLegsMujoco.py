import pybullet as p
import pybullet_data
import math as math

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
planeId = p.loadURDF("plane.urdf")
legsStartPos = [0,0,2]
p.setGravity(0,0,-10)
legsStartOrientation = p.getQuaternionFromEuler([0,0,0])
legsID = p.loadMJCF("legs.mjcf")[0]
p.resetBasePositionAndOrientation(legsID, legsStartPos, legsStartOrientation)

#drives the right thigh joint with a sin wave, left with cos
#p.setJointMotorControl2(legsID, 1, controlMode=p.VELOCITY_CONTROL, force=0)
time_step = 0.01
p.setTimeStep(time_step)
steps = 10000
kp = 150
kd = 15


for step_counter in range(steps):
    t = step_counter * time_step
    #position = joint_state[0], velocity - joint_state[1]
    right_hip = p.getJointState(legsID, 5)
    tau_1 = kp * (math.sin(t) - right_hip[0]) - kd * right_hip[1]

    left_hip = p.getJointState(legsID, 14)
    tau_7 = kp * (math.cos(t) - left_hip[0]) - kd * left_hip[1]
    p.setJointMotorControlArray(legsID, [1,7], controlMode=p.TORQUE_CONTROL, forces=[tau_1, tau_7])

    p.stepSimulation()
