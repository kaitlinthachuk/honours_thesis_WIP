import pybullet as p
import pybullet_data
import time
import numpy as np

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
planeId = p.loadURDF("plane.urdf")
legsStartPos = [0, 0, 1.5]
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
base_angle = 0

swing_hip_0_2 = 0.4
swing_knee_0_2 = -1.10
swing_hip_1_3 = -0.7
swing_knee_1_3 = -0.05

stance_knee_0_2 = -0.05
stance_ankle_0_2 = 0.20
stance_knee_1_3 = -0.10
stance_ankle_1_3 = 0.2

# PD controller constants
kp = 800
kd = 80

# simulation parameters
time_step = 0.01
p.setTimeStep(time_step)
foot_contact_tol = 0.0001


def stance_leg_torque(swing_hip_torque):
    body = p.getJointState(legsID, base_link)
    tau_body = kp*(base_angle - body[0]) - kd * body[1]
    tau_stance = -tau_body - swing_hip_torque
    return tau_stance


def set_torques(state):
    if state == 0:
        # swing leg
        right_hip = p.getJointState(legsID, right_hip_joint)
        tau_r_hip = kp * (swing_hip_0_2 - right_hip[0]) - kd * right_hip[1]

        right_knee = p.getJointState(legsID, right_knee_joint)
        tau_r_knee = kp * (swing_knee_0_2 - right_knee[0]) - kd * right_knee[1]

        # stance leg
        tau_l_hip = stance_leg_torque(tau_r_hip)

        left_knee = p.getJointState(legsID, left_knee_joint)
        tau_l_knee = kp * (stance_knee_0_2 - left_knee[0]) - kd * left_knee[1]

        left_ankle = p.getJointState(legsID, left_ankle_joint)
        tau_l_ankle = kp * (stance_ankle_0_2 - left_ankle[0]) - kd * left_ankle[1]

        # update forces
        p.setJointMotorControlArray(legsID, [right_hip_joint, right_knee_joint, left_hip_joint, left_knee_joint,
                                             left_ankle_joint], controlMode=p.TORQUE_CONTROL,
                                    forces=[tau_r_hip, tau_r_knee, tau_l_hip, tau_l_knee, tau_l_ankle])
    elif state == 1:
        # swing leg
        right_hip = p.getJointState(legsID, right_hip_joint)
        tau_r_hip = kp * (swing_hip_1_3 - right_hip[0]) - kd * right_hip[1]

        right_knee = p.getJointState(legsID, right_knee_joint)
        tau_r_knee = kp * (swing_knee_1_3 - right_knee[0]) - kd * right_knee[1]

        # stance leg
        tau_l_hip = stance_leg_torque(tau_r_hip)

        left_knee = p.getJointState(legsID, left_knee_joint)
        tau_l_knee = kp * (stance_knee_1_3 - left_knee[0]) - kd * left_knee[1]

        left_ankle = p.getJointState(legsID, left_ankle_joint)
        tau_l_ankle = kp * (stance_ankle_1_3 - left_ankle[0]) - kd * left_ankle[1]

        # update forces
        p.setJointMotorControlArray(legsID, [right_hip_joint, right_knee_joint, left_hip_joint, left_knee_joint,
                                             left_ankle_joint], controlMode=p.TORQUE_CONTROL,
                                    forces=[tau_r_hip, tau_r_knee, tau_l_hip, tau_l_knee, tau_l_ankle])
    elif state == 2:
        # swing leg
        left_hip = p.getJointState(legsID, left_hip_joint)
        tau_l_hip = kp * (swing_hip_0_2 - left_hip[0]) - kd * left_hip[1]

        left_knee = p.getJointState(legsID, left_knee_joint)
        tau_l_knee = kp * (swing_knee_0_2 - left_knee[0]) - kd * left_knee[1]

        # stance leg
        tau_r_hip = stance_leg_torque(tau_l_hip)

        right_knee = p.getJointState(legsID, right_knee_joint)
        tau_r_knee = kp * (stance_knee_0_2 - right_knee[0]) - kd * right_knee[1]

        right_ankle = p.getJointState(legsID, right_ankle_joint)
        tau_r_ankle = kp * (stance_ankle_0_2 - right_ankle[0]) - kd * right_ankle[1]

        # update forces
        p.setJointMotorControlArray(legsID, [right_hip_joint, right_knee_joint, right_ankle_joint, left_hip_joint,
                                             left_knee_joint], controlMode=p.TORQUE_CONTROL,
                                    forces=[tau_r_hip, tau_r_knee, tau_r_ankle, tau_l_hip, tau_l_knee])
    else:
        # swing leg
        left_hip = p.getJointState(legsID, left_hip_joint)
        tau_l_hip = kp * (swing_hip_1_3 - left_hip[0]) - kd * left_hip[1]

        left_knee = p.getJointState(legsID, left_knee_joint)
        tau_l_knee = kp * (swing_knee_1_3 - left_knee[0]) - kd * left_knee[1]

        # stance leg
        tau_r_hip = stance_leg_torque(tau_l_hip)

        right_knee = p.getJointState(legsID, right_knee_joint)
        tau_r_knee = kp * (stance_knee_1_3 - right_knee[0]) - kd * right_knee[1]

        right_ankle = p.getJointState(legsID, right_ankle_joint)
        tau_r_ankle = kp * (stance_ankle_1_3 - right_ankle[0]) - kd * right_ankle[1]

        # update forces
        p.setJointMotorControlArray(legsID, [right_hip_joint, right_knee_joint, right_ankle_joint, left_hip_joint,
                                             left_knee_joint], controlMode=p.TORQUE_CONTROL,
                                    forces=[tau_r_hip, tau_r_knee, tau_r_ankle, tau_l_hip, tau_l_knee])
    p.stepSimulation()


# state definitions
def state0():
    t = time.time()
    while time.time() - t < 0.3:
        set_torques(0)

    state1()


def state1():
    # no_contact = 1
    # while no_contact:
    #     #swing leg right leg, stance left
    #     set_torques(1)
    #     contact_points = p.getContactPoints(bodyA=planeId, bodyB=legsID, linkIndexB=right_foot_link)
    #     print("contact points: " + str(contact_points))
    #     if len(contact_points) > 0:
    #         contact_distance = contact_points[0][8]
    #         if contact_distance < foot_contact_tol:
    #             no_contact = 0
    #             print("no contact =0")
    #     print("state 1")
    t = time.time()
    while time.time() - t < 0.3:
        set_torques(1)
    state2()


def state2():
    t = time.time()
    while time.time() - t < 0.3:
        set_torques(2)
        print("state 2")
    state3()


def state3():
    # no_contact = 1
    # while no_contact:
    #     # swing leg left leg, stance right
    #     set_torques(3)
    #     contact_points = p.getContactPoints(bodyA=planeId, bodyB=legsID, linkIndexB=left_foot_link)
    #
    #     if len(contact_points) > 0:
    #         contact_distance = contact_points[0][8]
    #         if contact_distance < foot_contact_tol:
    #             no_contact = 0
    #     print("state 3")

    t = time.time()
    while time.time() - t < 0.3:
        set_torques(3)
    state0()


# start the FSM
state0()
