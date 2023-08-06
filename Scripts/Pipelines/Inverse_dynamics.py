import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common.Inverse_dynamics_bottom_up import Inverse_dynamics_force_plate_ur5e
from Common.Inverse_dynamics_top_down import Inverse_dynamics_top_down
import math


def execute():
    print("------------top_down")
    top_down: Inverse_dynamics_top_down = Inverse_dynamics_top_down()
    print("------------bottom_up")
    bottom_up: Inverse_dynamics_force_plate_ur5e = Inverse_dynamics_force_plate_ur5e()

    q: tuple = (math.pi, -2.3345737645286135E-6, -2.3345737645286135E-6, -math.pi / 2, 2.382993625360541E-5, math.pi)
    q_dot: tuple = (0, 0, 0, 0, 0, 0)
    q_ddot: tuple = (0, 0, 0, 0, 0, 0)

    print("------------top_down.calculate_torques")
    top_down_torques = top_down.calculate_torques(q=q, q_dot=q_dot, q_ddot=q_ddot)
    print("top_down_torques: \n", top_down_torques)
    print("------------bottom_up.calculate_print")
    f_force_plate = (2.871, -1.971, 172.724)
    m_force_plate = (-17.089, -61.873, -0.09)
    bottom_up_torques = bottom_up.calculate_torques(q=q, q_dot=q_dot, q_ddot=q_ddot, f_force_plate=f_force_plate, m_force_plate=m_force_plate)
    print("bottom_up_torques: \n", bottom_up_torques)
    return


if __name__ == "__main__":
    execute()
