from math import cos, sin, acos, atan2, radians, degrees, sqrt
import numpy.linalg as npl
from mpmath import norm
from numpy import *
import numpy as np
import time
import matplotlib.pyplot as plt
import modern_robotics as mr
from scipy.spatial.transform import Rotation as R

"""
nach einer Untersuchung zwischen dem Paper und aller Ergebnissen:
adjoint_Matrix = [ R   0; (P_tilde * R)   0]
Adjoint_Transpose  = [ R_transpose   (P_tilde * R)_transpose, 0   R_transpose]
das muss beachtet werden, wenn man von einem Frame zu einem anderen transformiert 

for simplification: H_{1}^{2} = H_1_2 / R_{1}^{2} = R_1_2  / P_{1}^{2} = R_1_2 
H_n_0 => H_n^0: transformation matrix of the actual point in the base frame (0 frame)

mr.so3ToVec: is a command in the modern Robotics library, that is an operator that gives the skew-symmetric
part of a square matrix. The same as the as() in the algorithm (paper)
"""


def getRotationVector(T):
    v = R.from_matrix(T)
    rotation_vector = v.as_rotvec()
    return rotation_vector


def pose_trans(pose_a, pose_b):  # tested
    TransMatr_a = getTransformationMatrix(pose_a)
    TransMatr_b = getTransformationMatrix(pose_b)
    rot = TransMatr_a @ TransMatr_b
    [rot_m, P_vec] = mr.TransToRp(rot)
    rotv = getRotationVector(rot_m)
    # rotv_UR = rotmat2rotvec(rot_m)
    pose_T = np.array([P_vec[0], P_vec[1], P_vec[2], rotv[0], rotv[1], rotv[2]])
    return pose_T


def pose_inv(pose_a):  # tested **
    actual_TransMatr = getTransformationMatrix(pose_a)
    [R_a, P_a] = mr.TransToRp(actual_TransMatr)
    T_Inv_Matrix = mr.RpToTrans(npl.inv(R_a), -npl.inv(R_a) @ np.transpose(P_a))
    [R_a_Inv, P_a_Inv] = mr.TransToRp(T_Inv_Matrix)
    a_rotv_Inv = getRotationVector(R_a_Inv)  # tested, both functions getRotationVector and rotmat2rotvec deliver
    a_rotv_Inv_UR = rotmat2rotvec(R_a_Inv)  # the sane result getRotationVector == rotmat2rotvec
    pose_a_Inv = [P_a_Inv[0], P_a_Inv[1], P_a_Inv[2], a_rotv_Inv_UR[0], a_rotv_Inv_UR[1],
                  a_rotv_Inv_UR[2]]
    return pose_a_Inv


def rotmat2rotvec(rotmat):  # tested
    # array to matrix
    r11 = rotmat[0][0]
    r12 = rotmat[0][1]
    r13 = rotmat[0][2]

    r21 = rotmat[1][0]
    r22 = rotmat[1][1]
    r23 = rotmat[1][2]

    r31 = rotmat[2][0]
    r32 = rotmat[2][1]
    r33 = rotmat[2][2]

    # rotation matrix to rotation vector
    theta = acos((r11 + r22 + r33 - 1) / 2)
    sth = sin(theta)

    if (theta > radians(179.99)) or (theta < radians(-179.99)):
        theta = radians(180)
        if r21 < 0:
            if r31 < 0:
                ux = sqrt((r11 + 1) / 2)
                uy = -sqrt((r22 + 1) / 2)
                uz = -sqrt((r33 + 1) / 2)
            else:
                ux = sqrt((r11 + 1) / 2)
                uy = -sqrt((r22 + 1) / 2)
                uz = sqrt((r33 + 1) / 2)
        else:
            if r31 < 0:
                ux = sqrt((r11 + 1) / 2)
                uy = sqrt((r22 + 1) / 2)
                uz = -sqrt((r33 + 1) / 2)
            else:
                ux = sqrt((r11 + 1) / 2)
                uy = sqrt((r22 + 1) / 2)
                uz = sqrt((r33 + 1) / 2)
    else:
        ux = (r32 - r23) / (2 * sth)
        uy = (r13 - r31) / (2 * sth)
        uz = (r21 - r12) / (2 * sth)

    rotvec = np.array([(theta * ux), (theta * uy), (theta * uz)])
    return rotvec


def rotvec2rotmat(rotvec):  # tested
    rx = rotvec[0]
    ry = rotvec[1]
    rz = rotvec[2]

    # rotation vector to angle and unit vector
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    ux = rx / theta
    uy = ry / theta
    uz = rz / theta
    cth = math.cos(theta)
    sth = math.sin(theta)
    vth = 1 - math.cos(theta)

    # column 1
    r11 = ux * ux * vth + cth
    r21 = ux * uy * vth + uz * sth
    r31 = ux * uz * vth - uy * sth
    # column 2
    r12 = ux * uy * vth - uz * sth
    r22 = uy * uy * vth + cth
    r32 = uy * uz * vth + ux * sth
    # column 3
    r13 = ux * uz * vth + uy * sth
    r23 = uy * uz * vth - ux * sth
    r33 = uz * uz * vth + cth

    # elements are represented as an array
    rotmat = [[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]]
    return rotmat


def rotmat2rpy(rotmat):
    # array to matrix
    r11 = rotmat[0][0]
    r12 = rotmat[0][1]
    r13 = rotmat[0][2]

    r21 = rotmat[1][0]
    r22 = rotmat[1][1]
    r23 = rotmat[1][2]

    r31 = rotmat[2][0]
    r32 = rotmat[2][1]
    r33 = rotmat[2][2]
    # rotation matrix to rpy
    beta = atan2(-r31, sqrt(r11 * r11 + r21 * r21))

    if beta > radians(89.99):
        beta = radians(89.99)
        alpha = 0
        gamma = atan2(r12, r22)
    elif beta < - radians(89.99):
        beta = - radians(89.99)
        alpha = 0
        gamma = -atan2(r12, r22)
    else:
        cb = cos(beta)
        alpha = atan2(r21 / cb, r11 / cb)
        gamma = atan2(r32 / cb, r33 / cb)

    rpy = [gamma, beta, alpha]
    return rpy


def rpy2rotmat(rpy):  # tested
    gamma = rpy[0]  # roll
    beta = rpy[1]  # pitch
    alpha = rpy[2]  # yaw

    # trigonometric
    ca = math.cos(alpha)
    cb = math.cos(beta)
    cg = math.cos(gamma)
    sa = math.sin(alpha)
    sb = math.sin(beta)
    sg = math.sin(gamma)

    # column 1
    r11 = ca * cb
    r21 = sa * cb
    r31 = -sb
    # column 2
    r12 = ca * sb * sg - sa * cg
    r22 = sa * sb * sg + ca * cg
    r32 = cb * sg
    # column 3
    r13 = ca * sb * cg + sa * sg
    r23 = sa * sb * cg - ca * sg
    r33 = cb * cg

    # elements are represented as an array
    rotmat = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])

    return rotmat


def rpy2rv(roll, pitch, yaw):
    alpha = yaw
    beta = pitch
    gamma = roll

    ca = cos(alpha)
    cb = cos(beta)
    cg = cos(gamma)
    sa = sin(alpha)
    sb = sin(beta)
    sg = sin(gamma)

    r11 = ca * cb
    r12 = ca * sb * sg - sa * cg
    r13 = ca * sb * cg + sa * sg
    r21 = sa * cb
    r22 = sa * sb * sg + ca * cg
    r23 = sa * sb * cg - ca * sg
    r31 = -sb
    r32 = cb * sg
    r33 = cb * cg

    theta = math.acos((r11 + r22 + r33 - 1) / 2)
    sth = sin(theta)
    kx = (r32 - r23) / (2 * sth)
    ky = (r13 - r31) / (2 * sth)
    kz = (r21 - r12) / (2 * sth)

    rv_1 = theta * kx
    rv_2 = theta * ky
    rv_3 = theta * kz
    return [rv_1, rv_2, rv_3]


def rv2rpy(rx, ry, rz):  # better that getEuler()   tested
    theta = sqrt(rx * rx + ry * ry + rz * rz)
    kx = rx / theta
    ky = ry / theta
    kz = rz / theta
    cth = cos(theta)
    sth = sin(theta)
    vth = 1 - cos(theta)

    r11 = kx * kx * vth + cth
    r12 = kx * ky * vth - kz * sth
    r13 = kx * kz * vth + ky * sth
    r21 = kx * ky * vth + kz * sth
    r22 = ky * ky * vth + cth
    r23 = ky * kz * vth - kx * sth
    r31 = kx * kz * vth - ky * sth
    r32 = ky * kz * vth + kx * sth
    r33 = kz * kz * vth + cth

    beta = atan2(-r31, sqrt((r11 * r11) + (r21 * r21)))

    if beta > radians(89.99):
        beta = radians(89.99)
        alpha = 0
        gamma = atan2(r12, r22)
    elif beta < - radians(89.99):
        beta = -radians(89.99)
        alpha = 0
        gamma = - atan2(r12, r22)
    else:
        cb = cos(beta)
        alpha = atan2(r21 / cb, r11 / cb)
        gamma = atan2(r32 / cb, r33 / cb)

    return [gamma, beta, alpha]


def getEuler(rotationsvector):  # tested **
    Rotation_Vector = np.array([rotationsvector[0], rotationsvector[1], rotationsvector[2]])
    R_T = R.from_rotvec(Rotation_Vector)
    M = R_T.as_matrix()
    RPY = rotmat2rpy(M)
    eulervect = R_T.as_euler('xyz',
                             degrees=False)  # the whole Rotation Matrix of the Pose which is the multiplication of the
    # Rotation Matrices of the Pose R_x_Matrix * R_y_Matrix * R_z_Matrix
    return eulervect


def getSkewSymmetric(m):  # tested
    skew_M = 0.5 * (m - np.transpose(m))
    return skew_M


def getTransformationMatrix(pose):  # tested
    Rotation_Matrix = getRotationsMatrix([pose[3], pose[4], pose[5]])  # the Rotation Matrix
    # derived from the given pose R_x_Matrix * R_y_Matrix * R_z_Matrix
    Position_Vector = np.array([pose[0], pose[1], pose[2]])  # P_v_0 target pose of the TCP (x, y, z)
    H = mr.RpToTrans(Rotation_Matrix, Position_Vector)  # build the finale homogenous transformation matrix
    # to describe the Rotation and the translation according to the given Pose [ Rotation  Translation; 0  1]
    return H


def getRotationsMatrix(rotationsvector):  # tested
    Rotation_Vector = np.array([rotationsvector[0], rotationsvector[1], rotationsvector[2]])
    R_T = R.from_rotvec(Rotation_Vector)
    Rotation_Matrix = R_T.as_matrix()  # the whole Rotation Matrix of the Pose which is the multiplication of the
    # Rotation Matrices of the Pose R_x_Matrix * R_y_Matrix * R_z_Matrix
    return Rotation_Matrix


def get_pose(T):
    [Rt, P] = mr.TransToRp(T)
    r = R.from_matrix(Rt)
    pose = np.hstack([P, r.as_rotvec()])
    pose_scaled = pose_scal(pose)
    return pose_scaled


def pose_scal(pose):
    v = pose
    pi = 3.1416
    l = sqrt(pow(v[3], 2) + pow(v[4], 2) + pow(v[5], 2))
    scale = 1 - 2 * pi / l
    if ((norm(v[3]) >= 0.001 and v[3] < 0.0) or (norm(v[3]) < 0.001 and norm(v[4]) >= 0.001 and v[4] < 0.0) or (
            norm(v[3]) < 0.001 and norm(v[4]) < 0.001 and v[5] < 0.0)):
        tcp_pose = [v[0], v[1], v[2], scale * v[3], scale * v[4], scale * v[5]]
    else:
        tcp_pose = v
    return tcp_pose


def get_Traj(poses, time_f, sampling_time):
    nb_of_points = len(poses)
    traj = [0] * nb_of_points
    trajectory = []
    for i in range(0, nb_of_points - 1):
        traj[i] = get_Traj_PtoP(poses[i], poses[i + 1], time_f, sampling_time)
        if i == 0:
            trajectory = traj[0]
        else:
            trajectory = np.concatenate([trajectory, traj[i]])
    return trajectory


def get_Traj_PtoP(start_p, desired_p, time_f, sampling_time):
    N_samples = int(time_f // sampling_time)
    T_Start = getTransformationMatrix(start_p)  # Homogenous transformation Matrix of the initial Point
    # ------------------------------------------------------------------------------------------------------------------
    T_desired = getTransformationMatrix(desired_p)  # Homogenous transformation Matrix of the Desired Point
    # ------------------------------------generate the Trajectory using MR Library -------------------------------------
    Trajectory = mr.CartesianTrajectory(T_Start, T_desired, time_f, N_samples, 5)
    return Trajectory


def TransformWrenchToBaseFrame(H_0_n, wrench_Values):
    AD_H_0_n = mr.Adjoint(H_0_n)
    # AD_H0n_Transposed = np.transpose(AD_H_0_n)
    AD_H0n_Transposed = np.transpose(AD_H_0_n)
    Wrench_Transposed = AD_H0n_Transposed @ wrench_Values
    wrench_0 = np.transpose(Wrench_Transposed)
    return wrench_0


def TransformWrenchToBaseFrame_unex(T_n_0, M_t, F_t):
    T_0_n = npl.inv(T_n_0)
    H_0_n = T_0_n
    [R_0_n, P_0_n] = mr.TransToRp(H_0_n)
    P_0_n_tilde = mr.VecToso3(P_0_n)  # bis hier alles das gleiche wie Adjoind matrix, wenn man alle komponenete richtig
    # multipliziert

    wrench_values = np.hstack((M_t, F_t))
    Wrench_n = np.matrix(wrench_values)

    [R_n_0, P_n_0] = mr.TransToRp(T_n_0)
    P_n_0_tilde = mr.VecToso3(P_n_0)  # bis hier alles das gleiche wie Adjoind matrix, wenn man alle komponenete richtig
    # multipliziert

    M_0 = R_n_0 @ M_t - R_n_0 @ P_0_n_tilde @ F_t
    F_0 = R_n_0 @ F_t
    wrench_0 = np.hstack((M_0, F_0))
    return wrench_0


def dampingWrench(speed, damping):
    wrench_d = np.asarray(damping) @ np.asarray(speed)
    return wrench_d


def transform_R1toR2(pose):
    rotationMatrix = np.array([[cos(66), sin(66), 0], [-sin(66), cos(66), 0], [0, 0, 1]])
    new_position = rotationMatrix @ [[pose[0], pose[1], pose[1]]]
