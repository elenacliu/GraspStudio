# Copyright 2023 Chang Liu.  All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from panda_py import libfranka
import panda_py
import numpy as np
import os
import time
from dataclasses import dataclass, field
from typing import List, Type
from numpy.typing import NDArray

from grasp import Grasp, GraspConfig
from motion_solver import PybulletMotionSolverConfig


@dataclass
class PandaGraspConfig(GraspConfig):
    _target: Type = field(default_factory=lambda : PandaGrasp)
    gripper_speed: float = 0.1
    move_speed: float = 0.2
    hostname: str = '172.16.0.2'
    max_grasp_force: int = 60
    epsilon_inner: float = 0.04
    epsilon_outer: float = 0.04


class PandaGrasp(Grasp):
    config : PandaGraspConfig

    def __init__(self, config : PandaGraspConfig):
        super().__init__(config=config)
        self.panda = panda_py.Panda(self.config.hostname)            # initialize panda (franka emika) robot arm
        self.gripper = libfranka.Gripper(self.config.hostname)       # initialize franka gripper

    def goto_joints(self, joints: List):
        self.panda.move_to_joint_position(joints, speed_factor=self.config.move_speed)
    
    def goto_pose(self, pose: NDArray):
        self.panda.move_to_pose(pose, speed_factor=self.config.move_speed)

    def close_gripper(self):
        self.gripper.move(width=0.0, speed=self.config.gripper_speed)

    def open_gripper(self):
        self.gripper.move(width=self.config.max_gripper_width, speed=self.config.gripper_speed)
    
    def grasp(self):
        self.gripper.grasp(width=0.0, speed=self.config.gripper_speed, force=self.config.max_grasp_force,
                            epsilon_inner=self.config.epsilon_inner, epsilon_outer=self.config.epsilon_outer)
        return self.gripper.read_once().is_grasped

    def get_cam_to_robot_pose(self):
        gripper_transform = self.panda.get_pose()
        cam_transform = gripper_transform @ self.cam2gripper_transformation
        return cam_transform

    def execute_pick_and_place(self, pre_grasp_transform, grasp_transform, after_grasp_transform, dst_joints):
        self.gripper.move(width=self.max_gripper_width, speed=self.gripper_speed)
        
        if self.ik_solver is not None:
            current_joints = self.panda.q
            pre_grasp_joints = np.array(self.ik_solver.ik(current_joints, pre_grasp_transform))
            grasp_joints = np.array(self.ik_solver.ik(current_joints, grasp_transform))
            after_grasp_joints = np.array(self.ik_solver.ik(current_joints, after_grasp_transform))

            self.panda.move_to_joint_position(pre_grasp_joints, speed_factor=self.move_speed)
            time.sleep(0.03)
            
            self.panda.move_to_joint_position(grasp_joints, speed_factor=self.move_speed)
            self.panda.recover()
            time.sleep(5)
            
            # sometimes the gripper will not grasp objects
            self.gripper.grasp(width=0.0, speed=self.config.gripper_speed, force=self.config.max_gripper_width,
                                epsilon_inner=self.config.epsilon_inner, epsilon_outer=self.config.epsilon_outer)

            self.panda.move_to_joint_position(after_grasp_joints, speed_factor=self.config.move_speed)

            self.panda.move_to_joint_position(dst_joints, speed_factor=self.config.move_speed)
            self.gripper.move(width=self.config.max_gripper_width, speed=self.config.gripper_speed)
            time.sleep(0.03)


if __name__=='__main__':
    # panda-py must provide third-party solver
    robot = PandaGraspConfig(motion_solver_config=PybulletMotionSolverConfig()).setup()
    robot.grasp_loop('rgbd')
