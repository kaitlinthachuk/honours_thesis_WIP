import pybullet as p
import pybullet_data
import time
import numpy as np

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
planeId = p.loadURDF("plane.urdf")
legsStartPos = [0, 0, 1]
p.setGravity(0, 0, -10)
legsStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
legsID = p.loadMJCF("legs.xml")[0]
p.resetBasePositionAndOrientation(legsID, legsStartPos, legsStartOrientation)

# joint and link numbers
base_link = 3

right_hip_joint = 5
right_knee_joint = 8
right_ankle_joint = 11
right_foot_link = 12

left_hip_joint = 14
left_knee_joint = 17
left_ankle_joint = 20
left_foot_link = 21

# target angles
swing_hip_0_2 = -0.4
swing_knee_0_2 = 1.10
swing_hip_1_3 = 0.7
swing_knee_1_3 = 0.05

stance_knee_0_2 = 0.05
stance_ankle_0_2 = -0.20
stance_knee_1_3 = 0.10
stance_ankle_1_3 = -0.2

# PD controller constants
kp = 1600
kd = 160

# simulation parameters
time_step = 0.01
p.setTimeStep(time_step)
t = 0
foot_contact_tol = 0.000001


def set_torques(state):
    if state == 0:
        # swing leg
        right_hip = p.getJointState(legsID, right_hip_joint)
        tau_r_hip = kp * (swing_hip_0_2 - right_hip[0]) - kd * right_hip[1]

        right_knee = p.getJointState(legsID, right_knee_joint)
        tau_r_knee = kp * (swing_knee_0_2 - right_knee[0]) - kd * right_knee[1]

        # stance leg
        left_knee = p.getJointState(legsID, left_knee_joint)
        tau_l_knee = kp * (stance_knee_0_2 - left_knee[0]) - kd * left_knee[1]

        left_ankle = p.getJointState(legsID, left_ankle_joint)
        tau_l_ankle = kp * (stance_ankle_0_2 - left_ankle[0]) - kd * left_ankle[1]

        # update forces
        p.setJointMotorControlArray(legsID, [right_hip_joint, right_knee_joint, left_knee_joint, left_ankle_joint],
                                    controlMode=p.TORQUE_CONTROL,
                                    forces=[tau_r_hip, tau_r_knee, tau_l_knee, tau_l_ankle])
    elif state == 1:
        # swing leg
        right_hip = p.getJointState(legsID, right_hip_joint)
        tau_R_hip = kp * (swing_hip_1_3 - right_hip[0]) - kd * right_hip[1]

        right_knee = p.getJointState(legsID, right_knee_joint)
        tau_R_knee = kp * (swing_knee_1_3 - right_knee[0]) - kd * right_knee[1]

        # stance leg
        left_knee = p.getJointState(legsID, left_knee_joint)
        tau_L_knee = kp * (stance_knee_1_3 - left_knee[0]) - kd * left_knee[1]

        left_ankle = p.getJointState(legsID, left_ankle_joint)
        tau_L_ankle = kp * (stance_ankle_1_3 - left_ankle[0]) - kd * left_ankle[1]

        # update forces
        p.setJointMotorControlArray(legsID, [right_hip_joint, right_knee_joint, left_knee_joint, left_ankle_joint],
                                    controlMode=p.TORQUE_CONTROL,
                                    forces=[tau_R_hip, tau_R_knee, tau_L_knee, tau_L_ankle])
    elif state == 2:
        # swing leg
        left_hip = p.getJointState(legsID, left_hip_joint)
        tau_L_hip = kp * (swing_hip_0_2 - left_hip[0]) - kd * left_hip[1]

        left_knee = p.getJointState(legsID, left_knee_joint)
        tau_L_knee = kp * (swing_knee_0_2 - left_knee[0]) - kd * left_knee[1]

        # stance leg
        right_knee = p.getJointState(legsID, right_knee_joint)
        tau_R_knee = kp * (stance_knee_0_2 - right_knee[0]) - kd * right_knee[1]

        right_ankle = p.getJointState(legsID, right_ankle_joint)
        tau_R_ankle = kp * (stance_ankle_0_2 - right_ankle[0]) - kd * right_ankle[1]

        # update forces
        p.setJointMotorControlArray(legsID, [right_knee_joint, right_ankle_joint, left_hip_joint, left_knee_joint],
                                    controlMode=p.TORQUE_CONTROL,
                                    forces=[tau_R_knee, tau_R_ankle, tau_L_hip, tau_L_knee])
    else:
        # swing leg
        left_hip = p.getJointState(legsID, left_hip_joint)
        tau_L_hip = kp * (swing_hip_1_3 - left_hip[0]) - kd * left_hip[1]

        left_knee = p.getJointState(legsID, left_knee_joint)
        tau_L_knee = kp * (swing_knee_1_3 - left_knee[0]) - kd * left_knee[1]

        # stance leg
        right_knee = p.getJointState(legsID, right_knee_joint)
        tau_R_knee = kp * (stance_knee_1_3 - right_knee[0]) - kd * right_knee[1]

        right_ankle = p.getJointState(legsID, right_ankle_joint)
        tau_R_ankle = kp * (stance_ankle_1_3 - right_ankle[0]) - kd * right_ankle[1]

        # update forces
        p.setJointMotorControlArray(legsID, [right_knee_joint, right_ankle_joint, left_hip_joint, left_knee_joint],
                                    controlMode=p.TORQUE_CONTROL,
                                    forces=[tau_R_knee, tau_R_ankle, tau_L_hip, tau_L_knee])


# state definitions
def state0():
    t = time.time()
    while time.time() - t < 0.3:
        set_torques(0)
        print("state0")
        p.stepSimulation()

    state1()


def state1():
    contact_distance = np.inf

    while contact_distance > foot_contact_tol:
        contact_points = p.getContactPoints(bodyA=planeId, bodyB=legsID)
        contact_distance = contact_points[0][8]
        set_torques(1)
        print("state 1")
        p.stepSimulation()

    state2()


def state2():
    t = time.time()
    while time.time() - t < 0.3:
        set_torques(2)
        print("state2")
        p.stepSimulation()
    state3()


def state3():
    contact_distance = np.inf

    while contact_distance > foot_contact_tol:
        contact_points = p.getContactPoints(bodyA=planeId, bodyB=legsID)
        contact_distance = contact_points[0][8]
        set_torques(3)
        print("state3")
        p.stepSimulation()
    state0()




#start the FSM
state0()
