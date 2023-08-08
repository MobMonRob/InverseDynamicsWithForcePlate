import os
from urdf2casadi.urdfparser import URDFparser
import numpy as np
from Common.Inverse_dynamics_rnea import Inverse_dynamics_rnea
from typing import Tuple

SixTuple = Tuple[float, float, float, float, float, float]
ThreeTuple = Tuple[float, float, float]


class Inverse_dynamics_force_plate_ur5e(object):
    def __init__(self):
        ur5e: URDFparser = URDFparser()
        path_to_urdf = os.path.dirname(os.path.abspath(__file__)) + "/ur5_mod.urdf"
        ur5e.from_file(path_to_urdf)
        rnea: Inverse_dynamics_rnea = Inverse_dynamics_rnea(ur5e)

        root = "base_link"
        tip = "tool0"

        self.f_sym, self.f_body_inertial_sym = rnea.get_forces_bottom_up(root, tip, gravity=[0, 0, -9.81])
        self.tau_bottom_up_sym = rnea.get_inverse_dynamics_rnea_bottom_up_f(root, tip)
        return

    def calculate_torques(self, q: SixTuple, q_dot: SixTuple, q_ddot: SixTuple, f_force_plate: ThreeTuple, m_force_plate: ThreeTuple) -> SixTuple:
        f_ur5e_base = Inverse_dynamics_force_plate_ur5e.__forces_force_plate_to_forces_ur5e_base(f_force_plate)
        m_ur5e_base = Inverse_dynamics_force_plate_ur5e.__moments_force_plate_to_moments_ur5e_base(f_ur5e_base, m_force_plate)

        # ! Verified direction. AMTI force plate: moments rotate clockwise; urdf model: moments rotate counterclockwise.
        # ! Spatial forces contain the moments first and then the forces.
        f_spatial_ur5e_base = np.concatenate([-1 * m_ur5e_base, f_ur5e_base])

        # print(self.f_body_inertial_sym(q, q_dot, q_ddot, f_spatial_ur5e_base))

        f_num = self.f_sym(q, q_dot, q_ddot, f_spatial_ur5e_base)
        tau_bottom_up_num = self.tau_bottom_up_sym(q, *f_num)
        return tuple(tau_bottom_up_num.T.full()[0])

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
