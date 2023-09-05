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


def execute():
    rootDir: str = os.path.abspath(f"{SCRIPT_DIR}/../..")
    dataDir: str = f"{rootDir}/Data/"
    # relativeBagPath: str = f"2023_08_04_ur5e_dynamic/end_position_to_dynamic_random_2023-08-04-19-18-34.bag"
    relativeBagPath: str = f"2023_08_04_ur5e_dynamic/start_position_to_dynamic_random_2023-08-04-19-08-59.bag"
    bagPath: str = f"{dataDir}{relativeBagPath}"
    # bagPath: str = f"{dataDir}2023_08_04_ur5e_static/static_south_2023-08-04-18-20-12.bag"
    plotSaveDir: str = f"{rootDir}/Plots/Monte_Carlo/{relativeBagPath}"

    # "/Force_plate_data",
    topics_fp: list[str] = ["/Force_plate_data_sma"]
    topic_jp: str = "/Joint_parameters"

    topics: set[str] = set([*topics_fp, topic_jp])
    re: RosbagExtractor = RosbagExtractor.fromBag(bagPath=bagPath, topics=topics)
    indexedBagMsgs: IndexedBagMsgs = re.getIndexedBagMsgs()

    for topic_fp in topics_fp:
        plotSaveDir_with_topic: str = f"{plotSaveDir}{topic_fp}/"

        bagMsgs_fp: BagMsgs = indexedBagMsgs.get_msgs(topic=topic_fp, bagPath=bagPath)
        msgs_fp: list[BagMessage] = bagMsgs_fp.msgs

        bagMsgs_jp: BagMsgs = indexedBagMsgs.get_msgs(topic=topic_jp, bagPath=bagPath)
        msgs_jp: list[BagMessage] = bagMsgs_jp.msgs

        msgs_compound: list[BagMessage] = list()
        msgs_compound.extend(msgs_fp)
        msgs_compound.extend(msgs_jp)
        compound_enumerated = list(enumerate(msgs_compound))
        msgs_compound_enumerated_sorted: list[Tuple[int, BagMessage]] = sorted(compound_enumerated, key=lambda x: x[1].timestamp)

        permutation: list[int] = [i for i, msg in msgs_compound_enumerated_sorted]
        msgs_compound_sorted: list[BagMessage] = [msg for i, msg in msgs_compound_enumerated_sorted]

        timed_jsps: list[Tuple[Time, Joints_spatial_force]] = Inverse_dynamics.create_timed_joints_spatial_force_list(topic_fp, topic_jp, msgs_compound_sorted)

        #! For saving: at least 100.
        #! 1000 leads to out of memory issues in the current implementation.
        #! Fast and easy fix idea: do only in batches of 100. Then calculate min and max per batch before resuming.
        monte_carlo_set_count: int = 200
        max_norm = 10

        for name, func in [("rand_f", randomize_fp_f), ("rand_m", randomize_fp_m)]:
            params = [(topic_jp, topic_fp, msgs_fp, msgs_jp, permutation, func, max_norm, i) for i in range(monte_carlo_set_count)]

            mc_sets_to_timed_jsps: list[list[Tuple[Time, Joints_spatial_force]]]
            with Pool() as pool:
                mc_sets_to_timed_jsps = pool.starmap(monte_carlo_set, params)

            plotSaveDir2 = f"{plotSaveDir_with_topic}/{name}/runs_{monte_carlo_set_count}/"
            plot(mc_sets_to_timed_jsps, timed_jsps, plotSaveDir=plotSaveDir2)

            print("finished plotting")
            del mc_sets_to_timed_jsps
            print("finised deleting")
            number: int = gc.collect()
            print(f"finished gc: {number}")

    sleep(3)
    print("finished")

    return


def plot(mc_sets_to_timed_jsps: "list[list[Tuple[Time, Joints_spatial_force]]]", timed_jsps: "list[Tuple[Time, Joints_spatial_force]]", plotSaveDir: str):
    joint_range = range(6)

    ylabels = (["Drehmoment [Nm]"] * 3) + (["Kraft [N]"] * 3)
    for (component_index, component_name), ylabel in zip(enumerate(["mx", "my", "mz", "fx", "fy", "fz"]), ylabels):
        first_time: Time = timed_jsps[0][0]
        times: list[float] = [(timed_jsp[0]-first_time).to_sec() for timed_jsp in timed_jsps]
        joints_to_component: list[list[float]] = [[timed_jsp[1].joints_bottom_up[joint].m_xyz__f_xyz[component_index]
                                                   for timed_jsp in timed_jsps] for joint in joint_range]

        joints_to_mc_set_to_component: list[list[list[float]]] = [[[timed_jsp[1].joints_bottom_up[joint].m_xyz__f_xyz[component_index]
                                                                    for timed_jsp in mc_set] for mc_set in mc_sets_to_timed_jsps] for joint in joint_range]

        joints_to_max_component: list[list[float]] = [np.max(mc_set_to_mzs, axis=0) for mc_set_to_mzs in joints_to_mc_set_to_component]

        joints_to_min_component: list[list[float]] = [np.min(mc_set_to_mzs, axis=0) for mc_set_to_mzs in joints_to_mc_set_to_component]

        adjusted_plotSaveDir: str = f"{plotSaveDir}{component_name}/"

        for joint in joint_range:
            description: str = f"joint_{joint}"
            mzs = joints_to_component[joint]
            min_mzs = joints_to_min_component[joint]
            max_mzs = joints_to_max_component[joint]

            plot_mc_time_series(adjusted_plotSaveDir, description, ylabel, times, mzs, min_mzs, max_mzs)

    return


def plot_mc_time_series(plotSaveDir: str, description: str, ylabel: str, times: "list[float]", component, min_component: "list[float]", max_component: "list[float]"):
    sizeFactor: float = 5  # 8 | Größer <=> Kleinere Schrift
    plt.gcf().set_size_inches(w=sqrt(2) * sizeFactor, h=1 * sizeFactor)
    plt.gcf().set_dpi(300)
    # plt.rcParams['figure.constrained_layout.use'] = True
    plt.tight_layout(pad=0.0, h_pad=0.0, w_pad=0.0)

    plt.xlabel("Zeit [s]", labelpad=0.0)
    plt.ylabel(ylabel, labelpad=0.0)

    plt.plot(times, component, color="b")
    plt.fill_between(times, min_component, max_component, color='r', alpha=0.5)

    plt.grid(visible=True, which="both", linestyle=':', color='k', alpha=0.5)

    # create plotSaveDir if not exists
    Path(plotSaveDir).mkdir(parents=True, exist_ok=True)
    plt.savefig(f"{plotSaveDir}{description}.svg", format="svg", transparent=True, pad_inches=0.01, bbox_inches="tight")
    plt.savefig(f"{plotSaveDir}{description}.png", format="png", transparent=True, pad_inches=0.01, bbox_inches="tight")

    plt.ioff()
    plt.close()


def monte_carlo_set(topic_jp: str, topic_fp: str, msgs_fp: "list[BagMessage]", msgs_jp: "list[BagMessage]", permutation: "list[int]", func: "Callable[[list[BagMessage], float], list[BagMessage]]", max_norm: float, i) -> "list[Tuple[Time, Joints_spatial_force]]":
    randomized_fp: list[BagMessage] = func(msgs_fp=msgs_fp, max_norm=max_norm)

    randomized_msgs_compound: list[BagMessage] = list()
    randomized_msgs_compound.extend(randomized_fp)
    randomized_msgs_compound.extend(msgs_jp)
    randomized_msgs_compound_sorted: list[BagMessage] = [randomized_msgs_compound[i] for i in permutation]

    timed_joints_spatial_force_list: list[Tuple[Time, Joints_spatial_force]] = Inverse_dynamics.create_timed_joints_spatial_force_list(topic_fp, topic_jp, randomized_msgs_compound_sorted)

    print(f"run: {i}")

    return timed_joints_spatial_force_list


def randomize_fp_f(msgs_fp: "list[BagMessage]", max_norm: float) -> "list[BagMessage]":
    component_for_norm_of_max_norm: float = max_norm / sqrt(3)
    rand_from: float = -component_for_norm_of_max_norm
    rand_to: float = component_for_norm_of_max_norm

    randomized_fp: list[BagMessage] = list()

    for topic, message, timestamp in msgs_fp:
        msg: Force_plate_data = deepcopy(message)
        msg.fx_N += random.uniform(rand_from, rand_to)
        msg.fy_N += random.uniform(rand_from, rand_to)
        msg.fz_N += random.uniform(rand_from, rand_to)
        bag_msg: BagMessage = BagMessage(topic=topic, message=msg, timestamp=timestamp)
        randomized_fp.append(bag_msg)

    return randomized_fp


def randomize_fp_m(msgs_fp: "list[BagMessage]", max_norm: float) -> "list[BagMessage]":
    component_for_norm_of_max_norm: float = max_norm / sqrt(3)
    rand_from: float = -component_for_norm_of_max_norm
    rand_to: float = component_for_norm_of_max_norm

    randomized_fp: list[BagMessage] = list()

    for topic, message, timestamp in msgs_fp:
        msg: Force_plate_data = deepcopy(message)
        msg.mx_Nm += random.uniform(rand_from, rand_to)
        msg.my_Nm += random.uniform(rand_from, rand_to)
        msg.mz_Nm += random.uniform(rand_from, rand_to)
        bag_msg: BagMessage = BagMessage(topic=topic, message=msg, timestamp=timestamp)
        randomized_fp.append(bag_msg)

    return randomized_fp


if __name__ == "__main__":
    execute()
