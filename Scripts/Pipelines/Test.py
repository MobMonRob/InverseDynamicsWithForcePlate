import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common.Inverse_dynamics_rnea import Inverse_dynamics_rnea
from urdf2casadi.urdfparser import URDFparser


def execute():
    ur5e: URDFparser = URDFparser()
    path_to_urdf = os.path.dirname(os.path.abspath(__file__)) + "/../Common/ur5e.urdf"
    ur5e.from_file(path_to_urdf)
    rnea: Inverse_dynamics_rnea = Inverse_dynamics_rnea(ur5e)

    root = "base_link"
    tip = "tool0"

    dummy_tuple = tuple(float(i) for i in range(rnea.get_n_joints(root, tip)))

    i_X_p, Ic = rnea.get_model_calculation(root, tip)
    ics = Ic(dummy_tuple)
    print(list(enumerate(ics)))
    return


if __name__ == "__main__":
    execute()
