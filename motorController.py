import numpy as np
import time
import pybullet as p


class MotorController():

    def __init__(self, robotID, physicsClientID, timeStep, motor_kp, motor_kd, motor_torque, motor_max_velocity, jd):
        self.robotID = robotID
        self.physicsClientID = physicsClientID
        self.timeStep = timeStep
        self.kd = motor_kd
        self.kp = motor_kp
        self.torque = motor_torque
        self.max_velocity = motor_max_velocity
        self.jd = jd

        self.jointNameToId, self.jointNum_revolute, self.jointId_list = self.getRevoluteJointIndex()
        self.jointPos_list = self.getJointPositionList()

        # create np array
        self.joint_curPos = np.array(self.jointPos_list, dtype=np.float)
        self.joint_targetPos = np.array(self.jointPos_list, dtype=np.float)

    def getRevoluteJointIndex(self):
        jointNameToId = {}
        jointId_list = []
        jointNum_all = p.getNumJoints(self.robotID)
        jointNum_revolute = 0
        # go through all joint, find revolute joint
        for i in range(jointNum_all):
            jointInfo = p.getJointInfo(self.robotID, i)
            if jointInfo[2] == 0:  # if joint is type revolute than add to joint index
                jointId_list.append(jointInfo[0])
                jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
                jointNum_revolute += 1
        return jointNameToId, jointNum_revolute, jointId_list

    # get revolute joint position only - np.array to joint_curPos
    def getJointPositionList(self):
        jointPos_list = []
        for i in range(self.jointNum_revolute):
            jointInfo = p.getJointInfo(self.robotID, self.jointId_list[i])
            if jointInfo[2] == 0:  # recheck, if joint is type revolute
                jointPos_list.append(p.getJointState(
                    self.robotID, jointInfo[0])[0])
        return jointPos_list

    def getJointCurPosition(self):
        for i in range(self.jointNum_revolute):
            self.joint_curPos[i] = p.getJointState(
                self.robotID, self.jointId_list[i], physicsClientId=self.physicsClientID)[0]
        return self.joint_curPos

    def setJointPosition(self, jointPoses):
        self.joint_targetPos = np.array(jointPoses, dtype=np.float)
        for i in range(self.jointNum_revolute):
            p.setJointMotorControl2(
                robotID, self.jointId_list[i], p.POSITION_CONTROL, targetPosition=self.joint_targetPos[i], targetVelocity=0.5,
                positionGain=0.5, velocityGain=0.5, maxVelocity=5.0)
        print("update positions")

    def delayTime(self, delayTime):
        if delayTime != 0:
            for _ in range(int(delayTime/self.timeStep)):
                p.stepSimulation()

    def searchMotorInJointIdList(self, motor):
        joint_nameId = self.jointNameToId[motor]
        for i in range(self.jointNum_revolute):
            if self.jointId_list[i] == joint_nameId:
                id_in_list = i
                return joint_nameId, id_in_list
        print("end search")
