# File: constants.py
# Author: Chang Liu (chang.liu at stu.pku.edu.cn)
# Date: November 8, 2023
# Description: This script implements some constants of robot arms.

from enum import Enum
import numpy as np


class FrankaConstants(Enum):
    JOINT_RANGES=(5.8, 3.5, 5.8, 3.1, 5.8, 3.8, 5.8)
    JOINT_LIMITS_LOWER=(-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
    JOINT_LIMITS_UPPER=(2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)