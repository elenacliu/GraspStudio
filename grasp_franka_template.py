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

"""
A template configuration and script for running the grasp class
"""
import numpy as np
from frankapy import FrankaConstants
from grasp import MyFrankaGraspConfig, MyFrankaGrasp
from motion_solver import PybulletMotionSolverConfig
from cameras import RealSenseCameraConfig


config = MyFrankaGraspConfig(
    initial_joints=FrankaConstants.HOME_JOINTS,
    camera_config=RealSenseCameraConfig(
        fx=912.646,
        fy=912.646,
        ppx=649.243774414,
        ppy=356.938446,
        w=1280,
        h=720,
        calibration=np.array(
            [[-0.00325625, -0.99974043,  0.02254913,  0.05886039],
            [ 0.99997401, -0.0034004,  -0.00635746, -0.03087357],
            [ 0.00643248,  0.02252784,  0.99972552, -0.04284756],
            [ 0.,          0.,          0.,          1.        ]]
        ),
        depth_fx=912.646,
        depth_fy=912.646,
        depth_ppx=649.243774414,
        depth_ppy=356.938446,
        depth_w=1280,
        depth_h=720,
        image_size_w=1280,
        image_size_h=720
    ),
    motion_solver_config=PybulletMotionSolverConfig(),
)
robot = MyFrankaGrasp(config=config)
robot.grasp_loop('rgbd')
