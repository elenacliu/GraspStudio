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

"""A base class GraspBot for diifferent implementations of grasps."""

import numpy as np
import copy
from dataclasses import dataclass, field
from typing import Type, Optional, List
from numpy.typing import NDArray
import time

from cameras import CameraConfig, RealSenseCameraConfig, Camera
from motion_solver import PybulletMotionSolverConfig, MotionSolver
from config import InstantiateConfig


@dataclass
class GraspConfig(InstantiateConfig):
    _target: Type = field(default_factory=lambda : Grasp)
    max_gripper_width: float = 0.08
    initial_camera2robot_transformation: NDArray = np.array([[-0.08920106, -0.99592763,  0.01308891,  0.33658066],
                                                        [-0.99519613,  0.08965247,  0.03933318,  0.02753368],
                                                        [-0.04034645, -0.00951747, -0.99914042,  0.6019472],
                                                        [ 0.,          0. ,         0. ,         1.        ]])
    initial_joints: NDArray = np.array([-0.02159332, -0.80462398,  0.00235787, -2.16951674,  0.0373164, 1.35462832, 0.8590827])
    place_joints: NDArray = np.array([1.8, -0.7855447937431189, 0.0003260311383163978, -2.3561892689822015, 0.000589521053350634, 1.5704794415504568, 0.7849731242977285])
    camera_config: CameraConfig = field(default_factory=lambda : RealSenseCameraConfig)
    motion_solver_config : Optional[PybulletMotionSolverConfig] = None
    debug : bool = False


class Grasp:
    config: GraspConfig
    camera: Camera
    ik_solver: Optional[MotionSolver]

    def __init__(self, config : GraspConfig) -> None:
        self.config = config
        self.camera = config.camera_config.setup()
        if self.config.motion_solver_config:
            self.ik_solver = self.config.motion_solver_config.setup()
        else:
            self.ik_solver = None
        self.initial_gripper2robot_transformation = self.config.initial_camera2robot_transformation @ np.linalg.inv(self.config.camera_config.calibration)

    def goto_pose(self, pose : NDArray, **kwargs):
        raise NotImplementedError

    def goto_joints(self, joints: List, **kwargs):
        raise NotImplementedError

    def close_gripper(self):
        raise NotImplementedError

    def open_gripper(self):
        raise NotImplementedError
    
    def grasp(self) -> bool:
        raise NotImplementedError

    @property
    def pose(self):
        raise NotImplementedError

    @property
    def joints(self):
        raise NotImplementedError
        
    def get_cam_to_robot_pose(self):
        raise NotImplementedError
    
    def go_to_start(self, **kwargs):
        # first go to the home position with gripper close and open
        self.goto_joints(self.config.initial_joints, **kwargs)
        # self.close_gripper()
        self.open_gripper()
    
    def grasp_once(self, method, **kwargs):
        raise NotImplementedError

    def grasp_loop(self, method, **kwargs):
        while True:
            self.grasp_once(method, **kwargs)

    def generate_grasping_trajectories(self, grasp_pose_matrix, pre_dis=0.2, after_height=0.25):
        back_matrix = np.eye(4)
        back_matrix[2][3] = -pre_dis
        pre_g2b_matrix = grasp_pose_matrix @ back_matrix
        after_g2b_matrix = copy.deepcopy(grasp_pose_matrix)
        after_g2b_matrix[2,3] += after_height
        return pre_g2b_matrix, after_g2b_matrix
    
    def execute_pick_and_place(self, pre_grasp_transform, grasp_transform, after_grasp_transform, dst_transform, **kwargs):
        if self.ik_solver is not None:
            current_joints = self.joints
            pre_grasp_joints = self.ik_solver.ik(current_joints, pre_grasp_transform)
            grasp_joints = self.ik_solver.ik(current_joints, grasp_transform)
            after_grasp_joints = self.ik_solver.ik(current_joints, after_grasp_transform)
            dst_joints = self.ik_solver.ik(current_joints, dst_transform)

            self.goto_joints(pre_grasp_joints, **kwargs)
            time.sleep(0.03)

            self.goto_joints(grasp_joints, **kwargs)
            grasp_res = self.grasp()

            if grasp_res:
                print('successful grasp')
            else:
                print('failed grasp')

            time.sleep(0.03)

            self.goto_joints(after_grasp_joints, **kwargs)
            time.sleep(0.03)

            self.goto_joints(dst_joints, **kwargs)
            self.open_gripper()
        else:
            self.goto_pose(pre_grasp_transform, **kwargs)
            time.sleep(0.03)

            self.goto_pose(grasp_transform, **kwargs)
            grasp_res = self.grasp()
            if grasp_res:
                print('successful grasp')
            else:
                print('failed grasp')
            time.sleep(0.03)

            self.goto_pose(after_grasp_transform, **kwargs)
            time.sleep(0.03)
            self.goto_pose(dst_transform, **kwargs)
            self.open_gripper()
        