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
from typing import Type
import pyrealsense2 as rs
import numpy as np
import cv2

from .camera import CameraConfig, Camera


@dataclass
class RealSenseCameraConfig(CameraConfig):
    _target: Type = field(default_factory=lambda : RealSenseCamera)
    exposure: float = 500.0
    max_depth_value: float = 800.0


class RealSenseCamera(Camera):
    config: RealSenseCameraConfig

    def __init__(self, config: RealSenseCameraConfig):
        super().__init__(config)
        self.pipeline = rs.pipeline()
        rsconfig = rs.config()

        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = rsconfig.resolve(self.pipeline_wrapper)
        device = self.pipeline_profile.get_device()

        #CENTERDREAM
        rsconfig.enable_stream(rs.stream.color, rs.format.rgb8, 30)

        rsconfig.enable_stream(rs.stream.depth,rs.format.z16, 30)
        rsconfig.enable_stream(rs.stream.infrared, 1)
        rsconfig.enable_stream(rs.stream.infrared, 2)

        cfg = self.pipeline.start(rsconfig)
        # PC
        pc = rs.pointcloud()
        # Decimation
        state_decimate = 1
        decimate = rs.decimation_filter()
        # Depth to disparity
        depth_to_disparity = rs.disparity_transform(True)
        disparity_to_depth = rs.disparity_transform(False)
        # Spatial:
        self.spatial = rs.spatial_filter()
        # Temporal:
        self.temporal = rs.temporal_filter()
        self.hole_filling = rs.hole_filling_filter()
        self.colorizer = rs.colorizer()

        sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
        sensor.set_option(rs.option.exposure, self.config.exposure)

    def rgb(self):
        frames = self.pipeline.wait_for_frames()
        align = rs.align(rs.stream.color)
        frames = align.process(frames)
        
        color_frame = frames.get_color_frame()
        color_bgr = np.asanyarray(color_frame.get_data())
        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
        return color_rgb

    def rgbd(self):
        frames = self.pipeline.wait_for_frames()
        align = rs.align(rs.stream.color)
        frames = align.process(frames)
        
        color_frame = frames.get_color_frame()
        color_bgr = np.asanyarray(color_frame.get_data())
        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)

        depth_frame = frames.get_depth_frame()
        depth_frame = self.spatial.process(depth_frame)
        depth_frame = self.hole_filling.process(depth_frame)

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image[depth_image > self.config.max_depth_value] = 0.
        return color_rgb, depth_image
