from graspnetAPI import Grasp
import numpy as np

def Rz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                     [np.sin(theta), np.cos(theta), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def convert_grasp_to_transformation_matrix(grasp : Grasp):
    translation = grasp.translation
    translation[2] -= 0.04
    rot_mat = grasp.rotation_matrix @ np.array([[0,0,1],[0,1,0],[-1,0,0]])

    transform_mat = np.eye(4)
    transform_mat[0:3,0:3] = rot_mat
    transform_mat[0:3,-1] = translation

    return transform_mat

