import pybullet as p
import pybullet_data
import math as math

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
#set up environment
planeId = p.loadURDF("plane.urdf")
p.setGravity(0,0,-5)
legsStartPos = [0,0,2]
legsStartOrientation = p.getQuaternionFromEuler([0,0,0])
legsID = p.loadURDF("legs.urdf", legsStartPos,legsStartOrientation)

time_step = 0.01
p.setTimeStep(time_step)
steps = 10000
kp = 300
kd = 30
numJoints = p.getNumJoints(legsID)
for step_counter in range(steps):
    t = step_counter * time_step

    #position = joint_state[0], velocity - joint_state[1]
    joint_states = p.getJointStates(legsID, range(0, numJoints))

    # target angle for thighs is -45 deg
    joint_state_1 = joint_states[1]
    tau_1 = kp*(-0.785398 - joint_state_1[0]) - kd*joint_state_1[1]
    joint_state_7 = joint_states[7]
    tau_7 = kp * (-0.785398 - joint_state_7[0]) - kd *joint_state_7[1]

    #target angle for knees is 60 deg
    joint_state_3 = joint_states[3]
    tau_3 = kp * (1.0472 - joint_state_3[0]) - kd * joint_state_3[1]
    joint_state_9 = joint_states[9]
    tau_9 = kp * (1.0472 - joint_state_9[0]) - kd * joint_state_9[1]

    # target angle for ankles is 10 deg
    # joint_state_5 = joint_states[5]
    # tau_5 = kp * (-0.174533 - joint_state_5[0]) - kd * joint_state_5[1]
    # joint_state_11 = joint_states[11]
    # tau_11 = kp * (-0.174533 - joint_state_11[0]) - kd * joint_state_11[1]

    #target angle
    p.setJointMotorControlArray(legsID, [1, 3, 7, 9], controlMode=p.TORQUE_CONTROL, forces=[tau_1, tau_3, tau_7, tau_9])
    p.stepSimulation()
