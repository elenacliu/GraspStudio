# File: solver.py
# Author: Chang Liu (chang.liu at stu.pku.edu.cn)
# Date: November 8, 2023
# Description: This script implements different independent motion solvers for the robot arm.

import pybullet as p
import pybullet_data as pd
import numpy as np
from scipy.spatial.transform import Rotation

from motion_solver.constants import FrankaConstants


class MotionSolver:
    def __init__(self, upperlimits, lowerlimits, jointranges) -> None:
        self.upperlimits = upperlimits
        self.lowerlimits = lowerlimits
        self.jointranges = jointranges

    def ik(self, current_joints, target_pose : np.array) -> list:
        raise NotImplementedError


class PybulletMotionSolver(MotionSolver):
    def __init__(self, upperlimits=FrankaConstants.JOINT_LIMITS_UPPER.value,
                 lowerlimits=FrankaConstants.JOINT_LIMITS_LOWER.value, 
                 jointranges=FrankaConstants.JOINT_RANGES.value, 
                 robot_config='franka_panda/panda.urdf'):
        '''
        Initialize the pybullet motion solver. The default configuration is to use the franka panda robot.
        Args:
            upperlimits: the upper joints limits for the robot
            lowerlimits: the lower joints limits for the robot
            jointranges:  The jointranges defines the range of motion allowed for each joint of the robot arm. 
                        It specifies the maximum allowed range of rotation for each joint. By setting appropriate joint ranges, 
                        you can limit the search space for the inverse kinematics solver. This helps ensure that the calculated 
                        target joints fall within the physical limits of the robot's joints.
            robot_config: the path of your robot configuration file
        '''
        super().__init__(upperlimits, lowerlimits, jointranges)
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pd.getDataPath())
        self.body_id = p.loadURDF(robot_config)
        reset_angle_list = [-0.00047377701846702965, -0.7855447937431189, 0.0003260311383163978, -2.3561892689822015, 0.000589521053350634, 1.5704794415504568, 0.7849731242977285] 
        for i in range(7):
            p.resetJointState(self.body_id, i, reset_angle_list[i])

    def ik(self, current_joints, target_pose : np.array) -> list:
        '''
        Args:
            current_joints: the current joint positions of the robot
            target_pose: the target pose of the robot
        Returns:
            target_joints: the target joint positions of the robot
        '''
        if type(current_joints) == np.ndarray:
            current_joints = current_joints.tolist()
        target_position = target_pose[:3, 3].tolist()
        target_orientation = Rotation.from_matrix(target_pose[:3, :3]).as_quat().tolist()
        target_joints = p.calculateInverseKinematics(bodyUniqueId=self.body_id, endEffectorLinkIndex=11, 
                                                       targetPosition=target_position, targetOrientation=target_orientation,
                                                       lowerLimits=self.lowerlimits, upperLimits=self.upperlimits, jointRanges=self.jointranges,
                                                       restPoses=current_joints, maxNumIterations=1000, residualThreshold=.001)
        target_joints = list(target_joints)

        for i in range(6):
            while target_joints[i] > self.upperlimits[i]:
                target_joints[i] -= np.pi * 2
            while target_joints[i] < self.lowerlimits[i]:
                target_joints[i] += np.pi * 2

        if  target_joints[6] > self.upperlimits[6]-0.3:
            target_joints[6] -= np.pi
        
        if  target_joints[6] < self.lowerlimits[6]+0.3:
            target_joints[6] += np.pi

        for i in range(7):
            if target_joints[i] >= self.upperlimits[i] or target_joints[i] <= self.lowerlimits[i]:
                print('joint {} out of range'.format(i))
                raise ValueError
        target_joints = np.array(target_joints[:7]).tolist()
        return target_joints