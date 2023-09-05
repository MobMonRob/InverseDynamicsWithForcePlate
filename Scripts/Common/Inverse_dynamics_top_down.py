import os
from urdf2casadi.urdfparser import URDFparser
from Common.Inverse_dynamics_rnea import Inverse_dynamics_rnea
from typing import Tuple

SixTuple = Tuple[float, float, float, float, float, float]
SixTupleTuple = Tuple[SixTuple, SixTuple, SixTuple, SixTuple, SixTuple, SixTuple]


class Inverse_dynamics_top_down(object):
    def __init__(self):
        ur5e: URDFparser = URDFparser()
        path_to_urdf = os.path.dirname(os.path.abspath(__file__)) + "/ur5_mod.urdf"
        ur5e.from_file(path_to_urdf)
        rnea: Inverse_dynamics_rnea = Inverse_dynamics_rnea(ur5e)

        root = "base_link"  # "dummy_link"
        tip = "tool0"

        self.tau_sym, self.forces_sym = rnea.get_inverse_dynamics_rnea(root, tip, gravity=[0, 0, 9.81])
        return

    def calculate_torques(self, q: SixTuple, q_dot: SixTuple, q_ddot: SixTuple) -> SixTuple:
        # tau_num = self.tau_sym([0.0] + list(q), [0.0] + list(q_dot), [0.0] + list(q_ddot))
        tau_num = self.tau_sym(list(q), list(q_dot), list(q_ddot))
        return tuple(tau_num.T.full()[0])  # [1:]

    def calculate_spatial_forces(self, q: SixTuple, q_dot: SixTuple, q_ddot: SixTuple) -> SixTupleTuple:
        # forces_num = self.forces_sym([0.0] + list(q), [0.0] + list(q_dot), [0.0] + list(q_ddot))
        forces_num = self.forces_sym(list(q), list(q_dot), list(q_ddot))
        return tuple(tuple(force.T.full()[0]) for force in forces_num)  # [1:]
