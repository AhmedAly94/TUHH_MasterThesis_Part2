import numpy.linalg as npl
from numpy import *
import numpy as np
import modern_robotics as mr
from Functions import getSkewSymmetric, rotmat2rpy, getRotationsMatrix, rotvec2rotmat, getTransformationMatrix, \
    rv2rpy, getEuler

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


def getTransformationMatrix(pose):
    Rotation_Matrix = getRotationsMatrix([pose[3], pose[4], pose[5]])  # the Rotation Matrix
    # derived from the given pose R_x_Matrix * R_y_Matrix * R_z_Matrix
    Position_Vector = [pose[0], pose[1], pose[2]]  # P_v_0 target pose of the TCP (x, y, z)
    H = mr.RpToTrans(Rotation_Matrix, Position_Vector)  # build the finale homogenous transformation matrix
    # to describe the Rotation and the translation according to the given Pose [ Rotation  Translation; 0  1]
    return H


def TransformWrenchToBaseFrame(H_0_n, wrench_Values):
    AD_H_0_n = mr.Adjoint(H_0_n)
    # AD_H0n_Transposed = np.transpose(AD_H_0_n)
    AD_H0n_Transposed = np.transpose(AD_H_0_n)
    Wrench_Transposed = AD_H0n_Transposed @ wrench_Values
    wrench_0 = np.transpose(Wrench_Transposed)
    Wrench = np.squeeze(np.asarray(wrench_0))
    return Wrench


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

    M_0 = R_n_0 @ M_t
    F_0 = R_n_0 @ F_t
    wrench_0 = np.hstack((M_0, F_0))
    return wrench_0


class WrenchForImpedance(object):
    def __init__(self, stiffness):
        self.K_c = None
        self.K_o = None
        self.K_t = None
        self.stiffness = stiffness

    def WrenchCalculation(self, target_v_p, actual_n_p):
        # build the stiffness Matrices K_t: translational stiffness/ K_o: Rotational stiffness/ K_c: coupling stiffness

        stiffness_kt = [self.stiffness[0], self.stiffness[1], self.stiffness[2]]
        self.K_t = np.diag(stiffness_kt)  # translatorische stiffness
        stiffness_ko = [self.stiffness[3], self.stiffness[4], self.stiffness[5]]
        self.K_o = np.diag(stiffness_ko)  # rotatorische stiffness
        self.K_c = np.zeros((3, 3))  # coupling stiffness

        H_v_in_0 = getTransformationMatrix(target_v_p)  # build the finale homogenous transformation matrix
        # to describe the Rotation and the translation according to the Target_Pose [ Rotation  Translation; 0  1]

        H_t_in_0 = getTransformationMatrix(actual_n_p)  # build the finale homogenous matrix to describe the
        # Rotation and the translation according to the actual_Pose [ Rotation  Translation; 0  1]

        G_kt = np.matrix((0.5 * np.trace(self.K_t) * np.identity(3)) - self.K_t)
        G_ko = np.matrix((0.5 * np.trace(self.K_o) * np.identity(3)) - self.K_o)
        G_kc = np.matrix((0.5 * np.trace(self.K_c) * np.identity(3)) - self.K_c)  # == 0 no coupling considered

        H_t_in_v = npl.inv(H_v_in_0) @ H_t_in_0  # tested
        # the relative configuration of the TCP target in actual frame

        """
        this Homogenous transformation matrix helps us to derive the wrench Wt = [mt ft] 
        which is exerted on the manipulator due to the spring and is expressed in 
        the coordinates of the end effector frame
        """
        [R_t_in_v, P_t_in_v] = mr.TransToRp(H_t_in_v)  # Rotation Matrix to describe the pose of TCP in the relative
        # configuration, Position of the TCP in the relative configuration
        P_t_v_tilde = mr.VecToso3(P_t_in_v)  # the [P] in the Adjoint Matrix == [R    0; [P]*R    0]

        H_v_in_t = npl.inv(H_t_in_v)
        [R_v_in_t, P_v_in_t] = mr.TransToRp(H_v_in_t)

        # +++++++++++++++++++++++ build the Force and Torque for the wrench ++++++++++++++++++++++++++++++++++++
        # +++++++++++++++++++++++++++++++ M_t: Torque and F_t: Force +++++++++++++++++++++++++++++++++++++++++++

        M_t_1 = G_ko @ R_t_in_v  # matmul == matrix multiplication // @ as well
        M_t_2 = G_kt @ R_v_in_t @ P_t_v_tilde @ P_t_v_tilde @ R_t_in_v  # matrix multiplication == @

        # mr.so3ToVec is a command in the modern Robotics library, that is an operator that gives the skew-symmetric
        # part of a square matrix. The same as the as() in the algorithm (paper)

        F_t_1_a = mr.so3ToVec(np.array(G_kt @ P_t_v_tilde))  # part of the First part in the Force calculation equation
        # the function mr.so3ToVec is tested +++++++++++++++++++++++++++++
        F_t_1 = R_v_in_t @ F_t_1_a @ R_t_in_v  # First part in the Force calculation equation
        F_t_2 = G_kt @ R_v_in_t @ P_t_v_tilde @ R_t_in_v  # second part in the Force calculation equation

        M_t = - 2 * mr.so3ToVec(getSkewSymmetric(np.array(M_t_1))) - mr.so3ToVec(getSkewSymmetric(np.array(M_t_2)))
        F_t = - F_t_1 - mr.so3ToVec(getSkewSymmetric(np.array(F_t_2)))

        wrench_vector = np.hstack((M_t, F_t))
        wrench_vector_T = np.transpose(np.matrix(wrench_vector))
        Wrench = TransformWrenchToBaseFrame(npl.inv(H_t_in_0), wrench_vector_T)
        Wrench_unex = TransformWrenchToBaseFrame_unex(H_t_in_0, M_t, F_t)
        wrench_0 = [Wrench_unex[3], Wrench_unex[4], Wrench_unex[5], Wrench_unex[0], Wrench_unex[1], Wrench_unex[2]]
        return wrench_0


if __name__ == "__main__":
    wrenchCalculation1 = WrenchForImpedance([500, 500, 500, 20, 20, 20])
    # # all poses are given in the Base frame
    # start_pose = np.array([0.89, 0.4, 0.15, 1.333, -2.509, 1.15])
    # desired_pose_1 = np.array([0.57942, 0.5, 0.62844, 2.049, -2.2261, 1.214])
    # # #
    # wrench = wrenchCalculation1.WrenchCalculation(desired_pose_1, start_pose)
