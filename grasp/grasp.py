# File: grasp.py
# Author: Chang Liu (chang.liu at stu.pku.edu.cn)
# Date: November 8, 2023
# Description: This script implements a base class GraspBot for diifferent implementations of grasps.

import numpy as np
from scipy.spatial.transform import Rotation
import cv2
import pyrealsense2 as rs
import pathlib
import copy
import yaml

from utils import get_waypoint_steps_hemisphere, get_cam2gripper_transformation, quaternion2rotation, \
     convert_array_to_transformation_matrix, convert_transformation_matrix_to_array


class GraspBot:
    def __init__(self, max_gripper_width=0.08, move_speed=0.1, ik_solver=None, camera_type='realsense', remote_server_config='../config/credential.yml') -> None:
        self.max_gripper_width = max_gripper_width
        self.move_speed = move_speed
        self.cam2gripper_transformation = get_cam2gripper_transformation(mode='latest')  # get the transformation matrix from camera to gripper
        self.initial_camera2robot_transformation = np.array([[-0.08920106, -0.99592763,  0.01308891,  0.33658066],
                                                        [-0.99519613,  0.08965247,  0.03933318,  0.02753368],
                                                        [-0.04034645, -0.00951747, -0.99914042,  0.6019472],
                                                        [ 0.,          0. ,         0. ,         1.        ]])
        self.initial_gripper2robot_transformation = self.initial_camera2robot_transformation @ np.linalg.inv(self.cam2gripper_transformation)
        self.initial_joints = [-0.02159332, -0.80462398,  0.00235787, -2.16951674,  0.0373164, 1.35462832, 0.8590827]
        self.place_joints = [1.8, -0.7855447937431189, 0.0003260311383163978, -2.3561892689822015, 0.000589521053350634, 1.5704794415504568, 0.7849731242977285]
        self.ik_solver = ik_solver
        if type(remote_server_config) == str or type(remote_server_config) == pathlib.Path:
            if pathlib.Path(remote_server_config).exists() and pathlib.Path(remote_server_config).suffix == '.yml':
                with open(remote_server_config, "r") as stream:
                    try:
                        self.remote_server_config = yaml.safe_load(stream)
                    except yaml.YAMLError as exc:
                        print(exc)
            else:
                raise ValueError('The remote server config file does not exist or the format is not correct! \n Please provide correct yaml file!')
        else:
            self.remote_server_config = None
                
        if camera_type == 'realsense':
            self.initialize_realsense()
        
    def initialize_realsense(self):
        self.pipeline = rs.pipeline()
        rsconfig = rs.config()

        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = rsconfig.resolve(self.pipeline_wrapper)
        device = self.pipeline_profile.get_device()

        #CENTERDREAM
        rsconfig.enable_stream(rs.stream.color, rs.format.rgb8, 30)

        #GRASPNET
        rsconfig.enable_stream(rs.stream.depth,rs.format.z16, 30)
        rsconfig.enable_stream(rs.stream.infrared, 1)
        rsconfig.enable_stream(rs.stream.infrared, 2)


        cfg = self.pipeline.start(rsconfig)
        profile = self.pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        depth_intrinsics = depth_profile.get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height
        # PC
        pc = rs.pointcloud()
        # Decimation
        state_decimate = 1
        decimate = rs.decimation_filter()
        # Depth to disparity
        depth_to_disparity = rs.disparity_transform(True)
        disparity_to_depth = rs.disparity_transform(False)
        # Spatial:
        spatial = rs.spatial_filter()
        # Temporal:
        temporal = rs.temporal_filter()
        hole_filling = rs.hole_filling_filter()
        colorizer = rs.colorizer()

        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        color_intrinsics = color_profile.get_intrinsics()
        cam_intrinsic = {
            'xres': color_intrinsics.width,
            'yres': color_intrinsics.height,
            'fx': color_intrinsics.fx,
            'fy': color_intrinsics.fy,
            'cx': color_intrinsics.ppx, 
            'cy': color_intrinsics.ppy
        }

        print(cam_intrinsic)

        out = np.empty((h, w, 3), dtype=np.uint8)
        sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
        sensor.set_option(rs.option.exposure, 500.000)
    
    def save_rgb_image(self, img_path):
        frames = self.pipeline.wait_for_frames()
        align = rs.align(rs.stream.color)
        frames = align.process(frames)
        
        color_frame = frames.get_color_frame()
        color_bgr = np.asanyarray(color_frame.get_data())
        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
        cv2.imwrite(img_path, color_rgb)

    def _generate_grasping_trajectories(self, grasp_pose_matrix, pre_dis=0.2, after_height=0.25):
        back_matrix = np.eye(4)
        back_matrix[2][3] = -pre_dis
        pre_g2b_matrix = grasp_pose_matrix @ back_matrix
        after_g2b_matrix = copy.deepcopy(grasp_pose_matrix)
        after_g2b_matrix[2,3] += after_height
        return pre_g2b_matrix, after_g2b_matrix

    def _get_gripper_transforms(self, method):
        gripper_transforms = []
        if method == 'nerf':
            distance_threshold = 0.25
            scene_center = np.array([0.5, 0.0, 0.02])
            initial_cam_pos = self.initial_camera2robot_transformation[:3,3]
            final_cam_pos = 2 * scene_center - initial_cam_pos
            final_cam_pos[2] = distance_threshold
            n_steps = 24

            pos, quat = get_waypoint_steps_hemisphere(initial_cam_pos.tolist(), final_cam_pos.tolist(), scene_center=scene_center, n_steps=n_steps)
            for pp, qq in zip(pos[:n_steps // 2], quat[:n_steps // 2]):
                rotmat_blender = quaternion2rotation(qq)
                rotmat_opencv = rotmat_blender @ np.array([[1,0,0],[0,-1,0],[0,0,-1]])
                rotquat_xyzw = Rotation.as_quat(Rotation.from_matrix(rotmat_opencv))
                # you need to compute the gripper pose from camera pose
                cam_transform = convert_array_to_transformation_matrix([pp, rotquat_xyzw])
                gripper_transform = cam_transform @ np.linalg.inv(self.cam2gripper_transformation)
                gripper_transforms.append(gripper_transform)

            return gripper_transforms
        elif method == 'rgbd':
            raise NotImplementedError
        elif method == 'graspnerf':
            # todo: you should add the position of graspnerf, how to match?
            raise NotImplementedError
    
    def _get_cam_to_robot_pose(self):
        raise NotImplementedError
    
    def _go_to_start(self):
        raise NotImplementedError
    
    def _grasp_nerf(self):
        raise NotImplementedError
    
    def _grasp_rgbd(self):
        raise NotImplementedError
    
    def _grasp_fixed(self):
        raise NotImplementedError
    
    def _grasp_once(self, method):
        self._go_to_start()
        if method == 'nerf':
            self._grasp_nerf()
        elif method == 'rgbd':
            self._grasp_rgbd()
        elif method == 'graspnerf':
            self._grasp_fixed()
        else:
            raise NotImplementedError('Other methods are not implemented yet!')
        
    def grasp_loop(self, method):
        while True:
            self._grasp_once(method)

    def _execute_pick_and_place(self, pre_grasp_transform, grasp_transform, after_grasp_transform, dst_transform):
        raise NotImplementedError