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

from grasp import Grasp
from motion_solver import PybulletMotionSolver


class PandaGrasp(Grasp):
    def __init__(self, max_gripper_width=0.08, gripper_speed=0.1, move_speed=0.2, ik_solver=None, camera_type='realsense', hostname='172.16.0.2', remote_server_config='./config/credential.yml', debug=False) -> None:
        super().__init__(max_gripper_width, move_speed, ik_solver, camera_type, 
                         os.path.join(os.path.dirname(os.path.realpath(__file__)), remote_server_config))
        self.gripper_speed = gripper_speed
        self.panda = panda_py.Panda(hostname)            # initialize panda (franka emika) robot arm
        self.gripper = libfranka.Gripper(hostname)       # initialize franka gripper
        self.debug = debug

    def _go_to_start(self):
        # first go to the home position with gripper open
        self.panda.move_to_joint_position(self.initial_joints, speed_factor=self.move_speed)
        # open gripper
        self.gripper.move(width=0.0, speed=self.gripper_speed)
        self.gripper.move(width=self.max_gripper_width, speed=self.gripper_speed)

    def _get_cam_to_robot_pose(self):
        gripper_transform = self.panda.get_pose()
        cam_transform = gripper_transform @ self.cam2gripper_transformation
        return cam_transform
    
    def _grasp_nerf(self):
        pass
            
    def _grasp_rgbd(self):
        pass

    def _grasp_fixed(self):
        pass

    def _execute_pick_and_place(self, pre_grasp_transform, grasp_transform, after_grasp_transform, dst_joints):
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
            
            print('########## begin grasping ##########')
            # sometimes the gripper will not grasp objects
            self.gripper.grasp(width=0.0, speed=self.gripper_speed, force=100, epsilon_inner=0.04, epsilon_outer=0.04)

            self.panda.move_to_joint_position(after_grasp_joints, speed_factor=self.move_speed)

            # self.panda.move_to_joint_position(dst_joints, speed_factor=self.move_speed)
            self.gripper.move(width=self.max_gripper_width, speed=self.gripper_speed)
            time.sleep(0.03)


if __name__=='__main__':
    # panda-py must provide third-party solver
    solver = PybulletMotionSolver()
    robot = PandaGrasp(ik_solver=solver, debug=True, camera_type=None)
    robot.grasp_loop('rgbd')
