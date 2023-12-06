# File: export_camera_config.py
# Author: Chang Liu (chang.liu at stu.pku.edu.cn)
# Date: November 10, 2023
# Description: Use this script to export the camera configuration file.

import pyrealsense2 as rs
import yaml

def export_realsense_configuration(config_path):
    pipeline = rs.pipeline()
    rsconfig = rs.config()

    rsconfig.enable_stream(rs.stream.color, rs.format.rgb8, 30)

    rsconfig.enable_stream(rs.stream.depth,rs.format.z16, 30)
    rsconfig.enable_stream(rs.stream.infrared, 1)
    rsconfig.enable_stream(rs.stream.infrared, 2)

    cfg = pipeline.start(rsconfig)
    profile = pipeline.get_active_profile()
    color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    color_intrinsics = color_profile.get_intrinsics()


    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()

    print(color_intrinsics.width, color_intrinsics.height)

    config = {
        'name': rs.camera_info.name,
        'serial_number': rs.camera_info.serial_number,
        'color_intriniscs': color_intrinsics,
        'depth_intrinsics': depth_intrinsics
    }

    print(config)