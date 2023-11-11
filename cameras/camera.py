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

from config import InstantiateConfig


@dataclass
class CameraConfig(InstantiateConfig):
    """Camera Config"""
    _target: Type = field(default_factory=lambda : Camera)
    # focal length of x axis
    fx: float
    # focal length of y axis
    fy: float
    # optical center of x
    ppx: float
    # optical center of y
    ppy: float
    # resolution x (width)
    w: int
    # resolution y (height)
    h: int
    # calibration matrix (camera on hand or camera on base)
    calibration: NDArray[np.float64]
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


@dataclass
class Camera:
    config: CameraConfig
    
    def rgb(self) -> NDArray:
        raise NotImplementedError('You should use a specified subclass!')

    def rgbd(self) -> Tuple[NDArray, NDArray]:
        raise NotImplementedError('You should use a specified subclass!')

    def depth_to_point_cloud(self) -> NDArray:
        depth_img = self.rgbd()
        h, w = depth_img.shape

        # scale camera parameters
        scale_x = w / self.config.depth_w
        scale_y = h / self.config.depth_h

        fx = self.config.depth_fx * scale_x
        fy = self.config.depth_fy * scale_y

        x_offset = self.config.depth_ppx * scale_x
        y_offset = self.config.depth_ppy * scale_y

        indices = torch.from_numpy(np.indices((h, w), dtype=np.float32).transpose(1,2,0))
        
        z_e = depth_img
        x_e = (indices[..., 1] - x_offset) * z_e / fx
        y_e = (indices[..., 0] - y_offset) * z_e / fy
        point_cloud = torch.stack([x_e, y_e, z_e], axis=-1)  # Shape: [H x W x 3]
        return point_cloud

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