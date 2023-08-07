import os
from urdf2casadi.urdfparser import URDFparser
from Common.Inverse_dynamics_rnea import Inverse_dynamics_rnea
from typing import Tuple

SixTuple = Tuple[float, float, float, float, float, float]


class Inverse_dynamics_top_down(object):
    def __init__(self):
        ur5e: URDFparser = URDFparser()
        path_to_urdf = os.path.dirname(os.path.abspath(__file__)) + "/ur5_mod.urdf"
        ur5e.from_file(path_to_urdf)
        rnea: Inverse_dynamics_rnea = Inverse_dynamics_rnea(ur5e)

        root = "base_link"
        tip = "tool0"

        self.tau_sym, self.forces_sym = rnea.get_inverse_dynamics_rnea(root, tip, gravity=[0, 0, -9.81])
        return

    def calculate_torques(self, q: SixTuple, q_dot: SixTuple, q_ddot: SixTuple) -> SixTuple:
        tau_num = self.tau_sym(q, q_dot, q_ddot)
        return tuple(tau_num.T.full()[0])

    def calculate_forces(self, q: SixTuple, q_dot: SixTuple, q_ddot: SixTuple) -> SixTuple:
        forces_num = self.forces_sym(q, q_dot, q_ddot)
        return tuple(forces_num.T.full()[0])
