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


from dataclasses import dataclass, field
from typing import Type, List
from numpy.typing import NDArray
import time
from autolab_core import RigidTransform
from frankapy import FrankaArm

from grasp import Grasp, GraspConfig


@dataclass
class FrankaGraspConfig(GraspConfig):
    _target: Type = field(default_factory=lambda : FrankaGrasp)
    with_gripper: bool = True


class FrankaGrasp(Grasp):
    config: FrankaGraspConfig

    def __init__(self, config : FrankaGraspConfig) -> None:
        super().__init__(config=config)
        self.franka_arm = FrankaArm(with_gripper=self.config.with_gripper)

    def goto_pose(self, transform: NDArray, **kwargs):
        duration=3
        use_impedance=True

        if 'duration' in kwargs:
            duration = kwargs['duration']
        if 'use_impedance' in kwargs:
            use_impedance = kwargs['use_impedance']

        rigid_gripper_transform = RigidTransform(
            rotation=transform[:3,:3],
            translation=transform[:3,3],
            from_frame='franka_tool',
            to_frame='world'
        )
        self.franka_arm.goto_pose(rigid_gripper_transform, duration=duration, use_impedance=use_impedance, ignore_errors=False, ignore_virtual_walls=True)

    def goto_joints(self, joints: List, **kwargs):
        duration=5
        use_impedance=False

        if 'duration' in kwargs:
            duration = kwargs['duration']
        if 'use_impedance' in kwargs:
            use_impedance = kwargs['use_impedance']

        self.franka_arm.goto_joints(joints, use_impedance=use_impedance, duration=duration)

    def close_gripper(self):
        self.franka_arm.close_gripper(grasp=False)

    def open_gripper(self):
        self.franka_arm.open_gripper()
    
    def grasp(self):
        self.franka_arm.close_gripper(grasp=True)
        return self.franka_arm.get_gripper_is_grasped()

    @property
    def pose(self) -> NDArray:
        return self.franka_arm.get_pose().matrix

    @property
    def joints(self) -> List:
        return self.franka_arm.get_joints().tolist()

    def get_cam_to_robot_pose(self):
        gripper_transform = self.pose
        cam_transform = gripper_transform @ self.config.camera_config.calibration
        return cam_transform

    def execute_pick_and_place(self, pre_grasp_transform, grasp_transform, after_grasp_transform, dst_joints, **kwargs):
        self.open_gripper()

        if self.ik_solver is not None:
            current_joints = self.joints
            pre_grasp_joints = self.ik_solver.ik(current_joints, pre_grasp_transform)
            grasp_joints = self.ik_solver.ik(current_joints, grasp_transform)
            after_grasp_joints = self.ik_solver.ik(current_joints, after_grasp_transform)

            self.goto_joints(pre_grasp_joints, ignore_virtual_walls=True)
            time.sleep(0.03)

            # self.goto_joints(grasp_joints, ignore_virtual_walls=True, use_impedance=True)
            self.goto_joints(grasp_joints, ignore_virtual_walls=True, use_impedance=False)
            grasp_res = self.grasp()

            time.sleep(0.03)

            self.goto_joints(after_grasp_joints, ignore_virtual_walls=True)
            time.sleep(0.03)
        else:
            self.goto_pose(pre_grasp_transform, duration=5, use_impedance=False)
            time.sleep(0.03)

            self.goto_pose(grasp_transform)
            grasp_res = self.grasp()
            time.sleep(0.03)

            self.goto_pose(after_grasp_transform, use_impedance=False)
            time.sleep(0.03)
        
        if type(dst_joints) == list:
            self.goto_joints(dst_joints, duration=5, ignore_virtual_walls=True)
        else:
            self.goto_pose(dst_joints, duration=5)

        self.franka_arm.open_gripper()


if __name__=='__main__':
    robot = FrankaGraspConfig().setup()
    robot.grasp_loop('nerf')