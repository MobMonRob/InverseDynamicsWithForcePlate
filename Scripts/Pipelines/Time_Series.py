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
from Common.Ros_msg_types.ur_robot_data_acquisition.msg import Joint_parameters
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
from math import pi


def execute():
    rootDir: str = os.path.abspath(f"{SCRIPT_DIR}/../..")
    dataDir: str = f"{rootDir}/Data/"

    relativeBagPaths: str = ["2023_08_04_ur5e_dynamic/start_position_to_dynamic_random_2023-08-04-19-08-59.bag", "2023_08_04_ur5e_static/static_west_2023-08-04-18-21-52.bag"]

    for relativeBagPath in relativeBagPaths:
        sizeFactor = None
        if "static" in relativeBagPath:
            sizeFactor = 2.5

        bagPath: str = f"{dataDir}{relativeBagPath}"
        # bagPath: str = f"{dataDir}2023_08_04_ur5e_static/static_south_2023-08-04-18-20-12.bag"
        plotSaveDir: str = f"{rootDir}/Plots/Time_Series/{relativeBagPath}/"

        topic_fp: str = "/Force_plate_data"
        topic_jp: str = "/Joint_parameters"

        topics: set[str] = set([topic_fp, topic_jp])
        re: RosbagExtractor = RosbagExtractor.fromBag(bagPath=bagPath, topics=topics)
        indexedBagMsgs: IndexedBagMsgs = re.getIndexedBagMsgs()

        msgs_compound_sorted: list[BagMessage] = Inverse_dynamics.create_msgs_compound_sorted(indexedBagMsgs, topic_fp, topic_jp, bagPath, calculate_sma=True)

        timed_jsfs: list[Tuple[Time, Joints_spatial_force]] = Inverse_dynamics.create_timed_joints_spatial_force_list(topic_fp, topic_jp, msgs_compound_sorted)

        # Start inverse dynamics

        components_ylabel: list[str] = (["Drehmoment [Nm]"] * 3) + (["Kraft [N]"] * 3)
        components_name: list[str] = ["mx", "my", "mz", "fx", "fy", "fz"]
        component_range = range(6)
        joint_range = range(6)
        joints_x_components = tuple(product(joint_range, component_range))

        first_time: Time = timed_jsfs[0][0]
        times: list[float] = [(timed_jsp[0]-first_time).to_sec() for timed_jsp in timed_jsfs]
        # bu und td Reihen. Beide in separaten Datenreihen. Aber im selben Diagramm.
        # joint unf force jeweils in separaten Diagrammen.
        bu_joint_component_to_values: dict[(int, int), list[float]] = {(joint, component): [timed_jsp[1].joints_bottom_up[joint].m_xyz__f_xyz[component] for timed_jsp in timed_jsfs]
                                                                       for joint, component in joints_x_components}
        td_joint_component_to_values: dict[(int, int), list[float]] = {(joint, component): [timed_jsp[1].joints_top_down[joint].m_xyz__f_xyz[component] for timed_jsp in timed_jsfs]
                                                                       for joint, component in joints_x_components}

        colors = ["b", "r"]
        labels = ["RNEA", "BURNEA"]
        for joint_i, component_i in joints_x_components:
            component_ylabel: str = components_ylabel[component_i]
            component_name: str = components_name[component_i]
            joint_name: str = f"Joint_{joint_i}"
            values = [td_joint_component_to_values[(joint_i, component_i)], bu_joint_component_to_values[(joint_i, component_i)]]

            plot_time_series(plotSaveDir=f"{plotSaveDir}inverse_dynamics/{joint_name}/", description=component_name,
                             ylabel=component_ylabel, times=times, values_list=values, colors=colors, labels=labels, sizeFactor=sizeFactor)

        # End inverse dynamics
        # Start norm

        bu_joint_to_ms = [[np.linalg.norm(timed_jsp[1].joints_bottom_up[joint].m_xyz__f_xyz[0:3]) for timed_jsp in timed_jsfs] for joint in joint_range]
        bu_joint_to_fs = [[np.linalg.norm(timed_jsp[1].joints_bottom_up[joint].m_xyz__f_xyz[3:6]) for timed_jsp in timed_jsfs] for joint in joint_range]
        td_joint_to_ms = [[np.linalg.norm(timed_jsp[1].joints_top_down[joint].m_xyz__f_xyz[0:3]) for timed_jsp in timed_jsfs] for joint in joint_range]
        td_joint_to_fs = [[np.linalg.norm(timed_jsp[1].joints_top_down[joint].m_xyz__f_xyz[3:6]) for timed_jsp in timed_jsfs] for joint in joint_range]

        joint_to_butd_to_ms = list(zip(bu_joint_to_ms, td_joint_to_ms))
        joint_to_butd_to_fs = list(zip(bu_joint_to_fs, td_joint_to_fs))

        joint_labels: list[str] = [f"Joint {i}" for i in range(6)]

        for label, (bu, td) in zip(joint_labels, joint_to_butd_to_ms):
            plot_time_series(plotSaveDir=f"{plotSaveDir}norm/{label}/", description="m", ylabel="2-Norm M [Nm]",
                             times=times, values_list=[td, bu], colors=["b", "r"], labels=["RNEA", "BURNEA"], sizeFactor=sizeFactor)
        for label, (bu, td) in zip(joint_labels, joint_to_butd_to_fs):
            plot_time_series(plotSaveDir=f"{plotSaveDir}norm/{label}/", description="f", ylabel="2-Norm F [N]",
                             times=times, values_list=[td, bu], colors=["b", "r"], labels=["RNEA", "BURNEA"], sizeFactor=sizeFactor)

        # End norm
        # Start joint positions

        bagMsgs_jp: BagMsgs = indexedBagMsgs.get_msgs(topic=topic_jp, bagPath=bagPath)
        msgs_jp: list[BagMessage] = bagMsgs_jp.msgs
        jp_first_time: Time = msgs_jp[0].timestamp
        jp_times: list[float] = [(timestamp-jp_first_time).to_sec() for topic, message, timestamp in msgs_jp]
        joints_to_positions: list[list[float]] = [[message.actual_joint_positions[i] for topic, message, timestamp in msgs_jp] for i in range(6)]

        colors: list[str] = ["b", "g", "r", "c", "m", "y"]
        joint_labels: list[str] = [f"Joint {i}" for i in range(6)]

        for values, color, label in zip(joints_to_positions, colors, joint_labels):
            plot_time_series(plotSaveDir=f"{plotSaveDir}joint_positions/", description=label,
                             ylabel="Position [rad]", times=jp_times, values_list=[values], colors=[color], labels=[label], y_max=pi, y_min=-pi, sizeFactor=sizeFactor)

        # End joint positions

    return


def plot_time_series(plotSaveDir: str, description: str, ylabel: str, times: "list[float]", values_list: "list[list[float]]", colors: "list[str]", labels: "list[str]", y_max=None, y_min=None, sizeFactor: float = None):

    if sizeFactor == None:
        sizeFactor: float = 5  # 8 | Größer <=> Kleinere Schrift
    plt.gcf().set_size_inches(w=2 * sizeFactor, h=1 * sizeFactor)
    plt.gcf().set_dpi(300)
    # plt.rcParams['figure.constrained_layout.use'] = True
    plt.tight_layout(pad=0.0, h_pad=0.0, w_pad=0.0)

    plt.xlabel("Zeit [s]", labelpad=0.0)
    plt.ylabel(ylabel, labelpad=0.0)

    linewidth = 1.5 * 5.0 / sizeFactor
    for values, color, label in zip(values_list, colors, labels):
        plt.plot(times, values, color=color, label=label, alpha=0.5, linewidth=linewidth)

    x_min = np.min(times)
    x_max = np.max(times)
    plt.xlim(left=x_min, right=x_max)

    if y_min == None:
        y_min = np.min(values_list)
    if y_max == None:
        y_max = np.max(values_list)

    gap = ((linewidth/1.5) * (y_max - y_min) * 0.02)
    bottom = y_min - gap
    top = y_max + gap
    plt.ylim(bottom=bottom, top=top)

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
