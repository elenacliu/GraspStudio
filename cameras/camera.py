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
from typing import Type, Tuple, Optional
from numpy.typing import NDArray
import numpy as np
import torch
import cv2

from config import InstantiateConfig


@dataclass
class CameraConfig(InstantiateConfig):
    """Camera Config"""
    _target: Type = field(default_factory=lambda : Camera)
    # focal length of x axis
    fx: float = 0.0
    # focal length of y axis
    fy: float = 0.0
    # optical center of x
    ppx: float = 0.0
    # optical center of y
    ppy: float = 0.0
    # resolution x (width)
    w: int = 0.0
    # resolution y (height)
    h: int = 0.0
    # image size
    image_size_w: int = 1280
    image_size_h: int = 720
    # calibration matrix (camera on hand or camera on base)
    calibration: NDArray[np.float64] = None
    # depth camera focal length of x axis (optional)
    depth_fx: Optional[float] = None
    # depth camera focal length of y axis (optional)
    depth_fy: Optional[float] = None
    # depth camera ppx
    depth_ppx: Optional[float] = None
    # depth camera ppy
    depth_ppy: Optional[float] = None
    # depth resolution x (width)
    depth_w: Optional[int] = None
    # depth esolution y (height)
    depth_h: Optional[int] = None


class Camera:
    config: CameraConfig

    def __init__(self, config : CameraConfig):
        self.config = config
    
    def rgb(self) -> NDArray:
        raise NotImplementedError('You should use a specified subclass!')

    def rgbd(self) -> Tuple[NDArray, NDArray]:
        raise NotImplementedError('You should use a specified subclass!')

    def depth_to_point_cloud(self, organized=False) -> Tuple[NDArray, NDArray]:
        """
        organized: bool
                whether to keep the cloud in image shape (H,W,3)
        """
        color_img, depth_img = self.rgbd()
        color_img = np.array(color_img, dtype=np.float32) / 255.0
        depth_img = np.array(depth_img / 1000, dtype=np.float32)

        # depth image resize to the color image size
        # just use the original size of depth image and color image
        # depth_img = cv2.resize(depth_img, (self.config.image_size_w, self.config.image_size_h), interpolation=cv2.INTER_NEAREST)
        # color_img = cv2.resize(color_img, (self.config.image_size_w, self.config.image_size_h), interpolation=cv2.INTER_LINEAR)
        # the scale should be considering again
        h, w = depth_img.shape

        # scale camera parameters
        scale_x = w / self.config.depth_w
        scale_y = h / self.config.depth_h

        fx = self.config.depth_fx * scale_x
        fy = self.config.depth_fy * scale_y

        x_offset = self.config.depth_ppx * scale_x
        y_offset = self.config.depth_ppy * scale_y

        indices = torch.from_numpy(np.indices((h, w), dtype=np.float32).transpose(1,2,0))
        
        z_e = torch.from_numpy(depth_img)
        x_e = (indices[..., 1] - x_offset) * z_e / fx
        y_e = (indices[..., 0] - y_offset) * z_e / fy
        point_cloud = torch.stack([x_e, y_e, z_e], axis=-1).numpy()  # Shape: [H x W x 3]

        if not organized:
            color_img = color_img.reshape(-1, 3)
            point_cloud = point_cloud.reshape(-1, 3)
        return color_img, point_cloud

    @property
    def intrinsic(self):
        return {
            'fx': self.config.fx,
            'fy': self.config.fy,
            'cx': self.config.ppx,
            'cy': self.config.ppy,
            'w': self.config.w,
            'h': self.config.h
        }

    @property
    def depth_intrinsic(self):
        return {
            'fx': self.config.depth_fx,
            'fy': self.config.depth_fy,
            'cx': self.config.depth_ppx,
            'cy': self.config.depth_ppy,
            'w': self.config.depth_w,
            'h': self.config.depth_h
        }