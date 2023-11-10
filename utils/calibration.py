import numpy as np
from scipy.spatial.transform import Rotation as R

def get_cam2gripper_transformation(mode='2023-10-13'):
    if mode == '2023-10-13':
        cam2gripper_transformation = np.identity(4)
        cam2gripper_transformation[:3, :3] = R.from_quat(np.array([0.005212092223996157, 0.011696285696401243, 0.672074080860455, 0.7403731902433608])).as_matrix()
        cam2gripper_transformation[:3, -1] = np.array([0.05625886640755267, -0.023919033461539393, -0.017180147566057003]).T
    elif mode == 'latest':
        cam2gripper_transformation = np.identity(4)
        cam2gripper_transformation[:3, :3] = R.from_quat(np.array([0.010230236077871374, 0.005707993558820679, 0.7082341022957731, 0.7058804554771465])).as_matrix()
        cam2gripper_transformation[:3, -1] = np.array([0.05886039129977443, -0.0308735663409243, -0.04284755991904652]).T
    elif mode == 'old':
        cam2gripper_transformation = np.identity(4)
        cam2gripper_transformation[:3, :3] = R.from_quat(np.array([0.016383728422721285, 0.0104608374348375, 0.711064867885953445, 0.7028576655208821])).as_matrix()
        cam2gripper_transformation[:3, -1] = np.array([0.05885619565897187, -0.03532924201087158, -0.05503220506495983]).T
    else:
        cam2gripper_transformation = None
    return cam2gripper_transformation