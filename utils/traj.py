import numpy as np
import math
from scipy.spatial.transform import Rotation


def getRTFromAToB(pointCloudA, pointCloudB):

    muA = np.mean(pointCloudA, axis=0)
    muB = np.mean(pointCloudB, axis=0)

    zeroMeanA = pointCloudA - muA
    zeroMeanB = pointCloudB - muB

    covMat = np.matmul(np.transpose(zeroMeanA), zeroMeanB)
    U, S, Vt = np.linalg.svd(covMat)
    R = np.matmul(Vt.T, U.T)

    if np.linalg.det(R) < 0:
        print("Reflection detected")
        Vt[2, :] *= -1
        R = Vt.T * U.T
    T = (-np.matmul(R, muA.T) + muB.T).reshape(3, 1)
    return R, T


def cameraPositionCalibrat(start_position, look_at_position): 

    up = np.array([0, 0, 1])
    vectorZ = (look_at_position - start_position)/np.linalg.norm(look_at_position - start_position)
    vectorX = np.cross(up, vectorZ)/np.linalg.norm(np.cross(up, vectorZ))
    vectorY = np.cross(vectorZ, vectorX)/np.linalg.norm(np.cross(vectorX, vectorZ))

    # points in camera coordinates
    pointSensor= np.array([[0., 0., 0.], [1., 0., 0.], [0., 2., 0.], [0., 0., 3.]])

    # points in world coordinates
    pointWorld = np.array([start_position,
                            start_position + vectorX,
                            start_position + vectorY * 2,
                            start_position + vectorZ * 3])

    resR, resT = getRTFromAToB(pointSensor, pointWorld)
    resRot = Rotation.from_matrix(resR)

    return resRot


def quaternionFromRotMat(rotation_matrix):
    rotation_matrix = np.reshape(rotation_matrix, (1, 9))[0]
    w = math.sqrt(rotation_matrix[0]+rotation_matrix[4]+rotation_matrix[8]+1 + 1e-6)/2
    x = math.sqrt(rotation_matrix[0]-rotation_matrix[4]-rotation_matrix[8]+1 + 1e-6)/2
    y = math.sqrt(-rotation_matrix[0]+rotation_matrix[4]-rotation_matrix[8]+1 + 1e-6)/2
    z = math.sqrt(-rotation_matrix[0]-rotation_matrix[4]+rotation_matrix[8]+1 + 1e-6)/2
    a = [w,x,y,z]
    m = a.index(max(a))
    if m == 0:
        x = (rotation_matrix[7]-rotation_matrix[5])/(4*w)
        y = (rotation_matrix[2]-rotation_matrix[6])/(4*w)
        z = (rotation_matrix[3]-rotation_matrix[1])/(4*w)
    if m == 1:
        w = (rotation_matrix[7]-rotation_matrix[5])/(4*x)
        y = (rotation_matrix[1]+rotation_matrix[3])/(4*x)
        z = (rotation_matrix[6]+rotation_matrix[2])/(4*x)
    if m == 2:
        w = (rotation_matrix[2]-rotation_matrix[6])/(4*y)
        x = (rotation_matrix[1]+rotation_matrix[3])/(4*y)
        z = (rotation_matrix[5]+rotation_matrix[7])/(4*y)
    if m == 3:
        w = (rotation_matrix[3]-rotation_matrix[1])/(4*z)
        x = (rotation_matrix[6]+rotation_matrix[2])/(4*z)
        y = (rotation_matrix[5]+rotation_matrix[7])/(4*z)
    quaternion = (w,x,y,z)
    return quaternion

# 这个版本的转换函数默认q=[w,x,y,z]，pybullet get_observation和scipy的四元数定义是[x,y,z,w]=w+xi+yj+zz
def quaternion2rotation(q : list):
    return np.array([
        [1-2*q[2]*q[2]-2*q[3]*q[3], 2*q[1]*q[2]-2*q[3]*q[0], 2*q[1]*q[3]+2*q[0]*q[2]],
        [2*q[1]*q[2]+2*q[0]*q[3], 1-2*q[1]*q[1]-2*q[3]*q[3], 2*q[2]*q[3]-2*q[0]*q[1]],
        [2*q[1]*q[3]-2*q[0]*q[2], 2*q[2]*q[3]+2*q[0]*q[1], 1-2*q[1]*q[1]-2*q[2]*q[2]],
    ])

def get_c2w_from_waypoint_wxyz(waypoint):
    q = waypoint[1]
    rot = quaternion2rotation(q)
    trans = np.array(waypoint[0]).transpose()
    c2w = np.zeros((4, 4))
    c2w[:3,:3] = rot
    c2w[3, 3] = 1
    c2w[:3, 3] = trans
    return c2w

def convert_array_to_transformation_matrix(gg):
    translation = np.zeros(3)
    rotation = np.zeros(4)
    translation[0]=gg[0][0]
    translation[1]=gg[0][1]
    translation[2]=gg[0][2]

    rotation[0]=gg[1][0]
    rotation[1]=gg[1][1]
    rotation[2]=gg[1][2]
    rotation[3]=gg[1][3]
    rot_mat = Rotation.from_quat(rotation).as_matrix()

    transform_mat = np.eye(4)
    transform_mat[0:3,0:3] = rot_mat
    transform_mat[0:3,-1] = translation

    return transform_mat

def convert_transformation_matrix_to_array(transform_mat):
    trans = transform_mat[0:3,-1]
    rotation = transform_mat[0:3,0:3]
    quat = Rotation.from_matrix(rotation).as_quat()

    T_c_r = np.array([[0.,0.,0.],[0.,0.,0.,0.]], dtype=object)
    T_c_r[0][0]=trans[0]
    T_c_r[0][1]=trans[1]
    T_c_r[0][2]=trans[2]
    T_c_r[1][0]=quat[0]
    T_c_r[1][1]=quat[1]
    T_c_r[1][2]=quat[2]
    T_c_r[1][3]=quat[3]
    return T_c_r

def get_waypoint_steps_hemisphere(initial_position, final_position, scene_center, n_steps=48):
    o_ground = [(initial_position[0] + final_position[0]) / 2, (initial_position[1] + final_position[1]) / 2, final_position[2]]
    
    Trans = np.zeros((4))
    Trans[:3] = np.array(o_ground)
    tg_alpha = (final_position[1] - initial_position[1]) / (final_position[0] - initial_position[0])
    alpha = math.atan(tg_alpha)
    beta = -(math.pi - alpha)
    Rz = np.array([
        [math.cos(beta), -math.sin(beta), 0, 0],
        [math.sin(beta), math.cos(beta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    theta = np.linspace(0, np.pi, num=n_steps)
   
    r = np.sqrt((o_ground[1] - final_position[1]) ** 2 + (o_ground[0] - final_position[0]) ** 2)
    points = np.dstack(
        (r * np.cos(theta),
        r * np.sin(theta),
        np.zeros((n_steps)),
        np.ones((n_steps)))
    ).squeeze()
    
    old_points = (Rz @ points.T + Trans[:, None]).T
    
    xs, ys = old_points[:, 0], old_points[:, 1]
    zs = np.linspace(initial_position[2], final_position[2], num=n_steps)
    
    # calculate the rotation of camera
    # vectorZ/X/Y 是 camera 的朝向在 world 中的方向表示
    look_at = scene_center
    up = np.array([0, 0, 1])
    positions = []
    quats = []
    for x, y, z in zip(xs.tolist(), ys.tolist(), zs.tolist()):
        position = np.array([x,y,z])
        vectorZ = - (look_at - position)/np.linalg.norm(look_at - position)
        vectorX = np.cross(up, vectorZ)/np.linalg.norm(np.cross(up, vectorZ))
        vectorY = np.cross(vectorZ, vectorX)/np.linalg.norm(np.cross(vectorX, vectorZ))

        # points in camera coordinates
        pointSensor = np.array([[0., 0., 0.], [1., 0., 0.], [0., 2., 0.], [0., 0., 3.]])

        # points in world coordinates
        pointWorld = np.array([position,
                                position + vectorX,
                                position + vectorY * 2,
                                position + vectorZ * 3])

        resR, resT = getRTFromAToB(pointSensor, pointWorld)
        # w,x,y,z
        resQ = quaternionFromRotMat(resR)
        positions.append(position)
        quats.append(resQ)
    return positions, quats