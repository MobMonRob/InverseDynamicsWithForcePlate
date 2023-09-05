import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common.Rosbag_extractor import RosbagExtractor, IndexedBagMsgs, BagMsgs, BagMessage
from Common.Ros_msg_types.data_transformation.msg import Joints_spatial_force
from typing import Tuple
import numpy as np
from Pipelines import Inverse_dynamics
from Common.Ros_msg_types.vicon_data_publisher.msg import Force_plate_data
import random
from math import sqrt
from copy import deepcopy
from multiprocessing import Pool
from rospy import Time
from reloading import reloading
from time import sleep
import matplotlib.pyplot as plt
from pathlib import Path
from typing import Callable
import gc
from Pipelines import Inverse_dynamics
from itertools import product


def execute():
    rootDir: str = os.path.abspath(f"{SCRIPT_DIR}/../..")
    dataDir: str = f"{rootDir}/Data/"
    # relativeBagPath: str = f"2023_08_04_ur5e_dynamic/end_position_to_dynamic_random_2023-08-04-19-18-34.bag"
    relativeBagPath: str = f"2023_08_04_ur5e_dynamic/start_position_to_dynamic_random_2023-08-04-19-08-59.bag"
    bagPath: str = f"{dataDir}{relativeBagPath}"
    # bagPath: str = f"{dataDir}2023_08_04_ur5e_static/static_south_2023-08-04-18-20-12.bag"
    plotSaveDir: str = f"{rootDir}/Plots/Time_Series/{relativeBagPath}/"

    topic_fp: str = "/Force_plate_data"
    topic_jp: str = "/Joint_parameters"

    topics: set[str] = set([topic_fp, topic_jp])
    re: RosbagExtractor = RosbagExtractor.fromBag(bagPath=bagPath, topics=topics)
    indexedBagMsgs: IndexedBagMsgs = re.getIndexedBagMsgs()

    msgs_compound_sorted: list[BagMessage] = Inverse_dynamics.create_msgs_compound_sorted(indexedBagMsgs, topic_fp, topic_jp, bagPath, calculate_sma=True)

    timed_jsps: list[Tuple[Time, Joints_spatial_force]] = Inverse_dynamics.create_timed_joints_spatial_force_list(topic_fp, topic_jp, msgs_compound_sorted)

    ###
    components_ylabel: list[str] = (["Drehmoment [Nm]"] * 3) + (["Kraft [N]"] * 3)
    components_name: list[str] = ["mx", "my", "mz", "fx", "fy", "fz"]
    component_range = range(6)
    joint_range = range(6)
    joints_x_components = tuple(product(joint_range, component_range))

    first_time: Time = timed_jsps[0][0]
    times: list[float] = [(timed_jsp[0]-first_time).to_sec() for timed_jsp in timed_jsps]
    # bu und td Reihen. Beide in separaten Datenreihen. Aber im selben Diagramm.
    # joint unf force jeweils in separaten Diagrammen.
    bu_joint_component_to_values: dict[(int, int), list[float]] = {(joint, component): [timed_jsp[1].joints_bottom_up[joint].m_xyz__f_xyz[component] for timed_jsp in timed_jsps]
                                                                   for joint, component in joints_x_components}
    td_joint_component_to_values: dict[(int, int), list[float]] = {(joint, component): [timed_jsp[1].joints_top_down[joint].m_xyz__f_xyz[component] for timed_jsp in timed_jsps]
                                                                   for joint, component in joints_x_components}

    for joint_i, component_i in joints_x_components:
        component_ylabel: str = components_ylabel[component_i]
        component_name: str = components_name[component_i]
        joint_name: str = f"Joint_{joint_i}"
        plot_time_series(plotSaveDir=f"{plotSaveDir}{joint_name}/", description=component_name, ylabel=component_ylabel,
                         times=times, bu=bu_joint_component_to_values[(joint_i, component_i)], td=td_joint_component_to_values[(joint_i, component_i)])

    return


def plot_time_series(plotSaveDir: str, description: str, ylabel: str, times: "list[float]", bu: "list[float]", td: "list[float]"):
    sizeFactor: float = 5  # 8 | Größer <=> Kleinere Schrift
    plt.gcf().set_size_inches(w=2 * sizeFactor, h=1 * sizeFactor)
    plt.gcf().set_dpi(300)
    # plt.rcParams['figure.constrained_layout.use'] = True
    plt.tight_layout(pad=0.0, h_pad=0.0, w_pad=0.0)

    plt.xlabel("Zeit [s]", labelpad=0.0)
    plt.ylabel(ylabel, labelpad=0.0)

    plt.plot(times, td, color="b", label="RNEA", alpha=0.7, linewidth=1.5)
    plt.plot(times, bu, color="r", label="BURNEA", alpha=0.7, linewidth=1.5)

    x_min = np.min(times)
    x_max = np.max(times)
    plt.xlim(left=x_min, right=x_max)

    y_min = np.min((bu, td))
    y_max = np.max((bu, td))
    plt.ylim(bottom=y_min, top=y_max)

    plt.legend(loc="best")
    plt.grid(visible=True, which="both", linestyle=':', color='k', alpha=0.5)

    # create plotSaveDir if not exists
    Path(plotSaveDir).mkdir(parents=True, exist_ok=True)
    plt.savefig(f"{plotSaveDir}{description}.svg", format="svg", transparent=True, pad_inches=0.01, bbox_inches="tight")
    plt.savefig(f"{plotSaveDir}{description}.png", format="png", transparent=True, pad_inches=0.01, bbox_inches="tight")

    plt.ioff()
    plt.close()


if __name__ == "__main__":
    execute()
