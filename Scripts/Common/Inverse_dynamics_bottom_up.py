import os
from urdf2casadi.urdfparser import URDFparser
import numpy as np
import math
import time
from Common.Inverse_dynamics_rnea import Inverse_dynamics_rnea
from typing import Tuple, _VariadicGenericAlias

SixTuple = Tuple[float, float, float, float, float, float]


class Inverse_dynamics_force_plate_ur5e(object):
    def __init__(self):
        ur5e: URDFparser = URDFparser()
        path_to_urdf = os.path.dirname(os.path.abspath(__file__)) + "/ur5_mod.urdf"
        ur5e.from_file(path_to_urdf)
        self.rnea: Inverse_dynamics_rnea = Inverse_dynamics_rnea(ur5e)

        self.root = "base_link"
        self.tip = "tool0"
        return

    def calculate_print(self, q: SixTuple, q_dot: SixTuple, q_ddot: SixTuple) -> SixTuple:
        # i_X_p_sym, length = ur5.get_model_calculation(root, tip)
        # i_X_p = i_X_p_sym(q)
        # len = length(q)
        # print(len)
        # print("i_X_p: \n", i_X_p)

        f_force_plate = [2.871, -1.971, 172.724]
        f_ur5e_base = self.__forces_force_plate_to_forces_ur5e_base(f_force_plate)

        m_force_plate = [-17.089, -61.873, -0.089]
        m_ur5e_base = self.__moments_force_plate_to_moments_ur5e_base(f_ur5e_base, m_force_plate)

        f_spatial_ur5e_base = np.concatenate([-1 * m_ur5e_base, -1 * f_ur5e_base])

        # tau_sym_bu = ur5.get_inverse_dynamics_rnea_bottom_up(root, tip)

        f_sym, f_body_inertial = self.rnea.get_forces_bottom_up(self.root, self.tip, gravity=[0, 0, -9.81])
        f_num = f_sym(q, q_dot, q_ddot, f_spatial_ur5e_base)
        f_body_inertial_num = f_body_inertial(q, q_dot, q_ddot, f_spatial_ur5e_base)
        print("Bottom Up forces: \n", f_num)
        print("Body inertials forces: \n", f_body_inertial_num)

        print(f"f_num: {f_num}")

        tau_bottom_up_sym = self.rnea.get_inverse_dynamics_rnea_bottom_up_f(self.root, self.tip)
        tau_bottom_up_num = tau_bottom_up_sym(q, *f_num)
        print("Bottom Up numerical inverse dynamics: \n", tau_bottom_up_num)

        return

    @staticmethod
    def __forces_force_plate_to_forces_ur5e_base(forces: "list[float]") -> "list[float]":
        # R is calculated only once and it is
        R = np.array([[1,  0,  0],
                      [0, -1,  0],
                      [0,  0, -1]])

        forces_transformed = R.dot(forces)
        return forces_transformed

    @staticmethod
    def __moments_force_plate_to_moments_ur5e_base(forces_ur5e_base: "list[float]", moments: "list[float]") -> "list[float]":
        # R is calculated only once and it is
        R = np.array([[1,  0,  0],
                      [0, -1,  0],
                      [0,  0, -1]])

        # GC is position of (center of plate surface) in robot base frame
        GC = [-1.7915 / 1000, -5.1695 / 1000, -(22.58 + 1.4) / 1000]

        OC = [0, 0, -44.78 / 1000]

        GO = GC - R.dot(OC)

        # K is skew-symmetric, it is
        K = np.array([[0,     - GO[2],     GO[1]],
                      [GO[2],       0,   - GO[0]],
                      [-GO[1],   GO[0],         0]])

        moments_transformed = R.dot(moments) + K.dot(forces_ur5e_base)
        return moments_transformed
