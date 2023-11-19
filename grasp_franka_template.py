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


import numpy as np
from scipy.spatial.transform import Rotation
import time
from autolab_core import RigidTransform
from frankapy import FrankaArm
import os

from grasp import Grasp


class FrankaGrasp(Grasp):
    def __init__(self, max_gripper_width=0.08, move_speed=0.1, ik_solver=None, camera_type='realsense', remote_server_config='./config/credential.yml', debug=False) -> None:
        super().__init__(max_gripper_width, move_speed, ik_solver, camera_type, 
                         os.path.join(os.path.dirname(os.path.realpath(__file__)), remote_server_config))
        self.debug = debug
        self.franka_arm = FrankaArm()

    def _franka_goto_pose(self, transform: np.array, duration=3, use_impedance=True):
        rigid_gripper_transform = RigidTransform(
            rotation=transform[:3,:3],
            translation=transform[:3,3],
            from_frame='franka_tool',
            to_frame='world'
        )
        self.franka_arm.goto_pose(rigid_gripper_transform, duration=duration, use_impedance=use_impedance, ignore_errors=False, ignore_virtual_walls=True)

    def _get_cam_to_robot_pose(self):
        gripper_transform = self.franka_arm.get_pose().matrix
        cam_transform = gripper_transform @ self.cam2gripper_transformation
        return cam_transform

    def _go_to_start(self):
        self.franka_arm.goto_joints(self.initial_joints, use_impedance=False)
        self.franka_arm.close_gripper()
        time.sleep(1)
        self.franka_arm.open_gripper()

    def _grasp_nerf(self):
        pass
    
    def _grasp_rgbd(self):
        pass

    def _execute_pick_and_place(self, pre_grasp_transform, grasp_transform, after_grasp_transform, dst_joints):
        self.franka_arm.open_gripper()

        if self.ik_solver is not None:
            current_joints = self.franka_arm.get_joints()
            pre_grasp_joints = self.ik_solver.ik(current_joints, pre_grasp_transform)
            grasp_joints = self.ik_solver.ik(current_joints, grasp_transform)
            after_grasp_joints = self.ik_solver.ik(current_joints, after_grasp_transform)

            self.franka_arm.goto_joints(pre_grasp_joints, ignore_virtual_walls=True)
            time.sleep(0.03)

            self.franka_arm.goto_joints(grasp_joints, ignore_virtual_walls=True)
            self.franka_arm.close_gripper()
            time.sleep(0.03)

            self.franka_arm.goto_joints(after_grasp_joints, ignore_virtual_walls=True)
            time.sleep(0.03)
        else:
            self._franka_goto_pose(pre_grasp_transform, duration=5)
            time.sleep(0.03)

            self._franka_goto_pose(grasp_transform)
            self.franka_arm.close_gripper()
            time.sleep(0.03)

            self._franka_goto_pose(after_grasp_transform)
            time.sleep(0.03)
        
        if type(dst_joints) == list:
            self.franka_arm.goto_joints(dst_joints, duration=5, ignore_virtual_walls=True)
        else:
            self._franka_goto_pose(dst_joints, duration=5)

        self.franka_arm.open_gripper()


if __name__=='__main__':
    robot = FrankaGrasp(camera_type='realsense')
    robot.grasp_loop('rgbd')