import serial
import pybullet as p
import pybullet_data
import math
import numpy as np
import time

import motorController as m


physicsClient = p.connect(p.GUI)  # connect to local GUI
p.resetSimulation()

timeStep = 1./2000.
numIterations = 400

# p.setTimeStep(timeStep)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # load urdf
p.setPhysicsEngineParameter(numSolverIterations=numIterations)

robotStartPosition = [0, 0, 1.1237]
robotStartOrn = p.getQuaternionFromEuler([1.57, 0, 0])

# load objects - plane and robot
plane = p.loadURDF("plane.urdf")
robotID = p.loadURDF(
    "urdf/moveTesting.urdf", robotStartPosition, robotStartOrn, useFixedBase=False)


motor_kd = 0.5
motor_kp = 0.5
motor_torque = 1.5
motor_max_velocity = 5.0
timeStep = 1./240.

jd = [0.1]
motor_c = m.MotorController(robotID, physicsClient,
                            timeStep, motor_kp, motor_kd, motor_torque, motor_max_velocity, jd)


jointNameToId, jointNum_revolute, jointId_list = motor_c.getRevoluteJointIndex()


# disable default constraint-based motors
for i in range(jointNum_revolute):
    p.setJointMotorControl2(
        robotID, jointId_list[i], p.POSITION_CONTROL, targetPosition=0, force=0)

# ----------------------------
# set motor of each joint
# ----------------------------
#  --> body
# motor_hip = jointNameToId['joint_baseLink_to_hip']
motor_body = jointNameToId['joint_body']
#  --> arm_l
motor_shoulder_l = jointNameToId['joint_shoulder_l']
motor_arm_l_02 = jointNameToId['joint_arm_l_02']
motor_arm_l_03 = jointNameToId['joint_arm_l_03']
motor_hand_l = jointNameToId['joint_hand_l']
#  --> arm_r
motor_shoulder_r = jointNameToId['joint_shoulder_r']
motor_arm_r_02 = jointNameToId['joint_arm_r_02']
motor_arm_r_03 = jointNameToId['joint_arm_r_03']
motor_hand_r = jointNameToId['joint_hand_r']
#  --> leg_l
motor_leg_l_01 = jointNameToId['joint_leg_l_01']
motor_leg_l_02 = jointNameToId['joint_leg_l_02']
# motor_leg_l_03 = jointNameToId['joint_leg_l_03']
motor_foot_l = jointNameToId['joint_foot_l']
#  --> leg_r
motor_leg_r_01 = jointNameToId['joint_leg_r_01']
motor_leg_r_02 = jointNameToId['joint_leg_r_02']
# motor_leg_r_03 = jointNameToId['joint_leg_r_03']
motor_foot_r = jointNameToId['joint_foot_r']

# ----------------------------
# stand
# ----------------------------
stand_pos = np.array(robotStartPosition)
target_pos = stand_pos
stand_orn = robotStartOrn
stand_jointPoses = np.array(p.calculateInverseKinematics(
    robotID, motor_body, target_pos, stand_orn, jointDamping=jd*jointNum_revolute), dtype=np.float)
# print("\nstand_jointPoses: ", stand_jointPoses)

for i in range(jointNum_revolute):
    p.setJointMotorControl2(
        robotID, jointId_list[i], p.POSITION_CONTROL, targetPosition=stand_jointPoses[i], targetVelocity=0.5,
        positionGain=0.5, velocityGain=0.5, maxVelocity=5.0)

delayTime = 0.4  # range 120
motor_c.delayTime(delayTime)
print("stand still")


while 1:
    p.stepSimulation()
    time.sleep(1./240.)
