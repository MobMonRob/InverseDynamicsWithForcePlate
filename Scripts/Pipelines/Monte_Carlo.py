import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common.Rosbag_extractor import RosbagExtractor, IndexedBagMsgs, BagMsgs, BagMessage
from Common.Inverse_dynamics_node import Inverse_dynamics_node
from itertools import cycle, product
from Common.Ros_msg_types.data_transformation.msg import Joints_spatial_force, Spatial_force
from typing import Tuple
import pandas as pd
from pandas import DataFrame
import numpy as np
from Pipelines.Inverse_dynamics import create_msgs_compound_sorted, create_timed_joints_spatial_force_list
from Common.Ros_msg_types.vicon_data_publisher.msg import Force_plate_data
import random
from math import sqrt
from copy import deepcopy
from multiprocessing import Pool
from rospy import Time


def execute():
    rootDir: str = os.path.abspath(f"{SCRIPT_DIR}/../..")
    dataDir: str = f"{rootDir}/Data/"
    # relativeBagPath: str = f"2023_08_04_ur5e_dynamic/end_position_to_dynamic_random_2023-08-04-19-18-34.bag"
    relativeBagPath: str = f"2023_08_04_ur5e_dynamic/start_position_to_dynamic_random_2023-08-04-19-08-59.bag"
    bagPath: str = f"{dataDir}{relativeBagPath}"
    # bagPath: str = f"{dataDir}2023_08_04_ur5e_static/static_south_2023-08-04-18-20-12.bag"
    plotSaveDir: str = f"{rootDir}/Plots/Monte_Carlo/{relativeBagPath}"

    topics_fp: list[str] = ["/Force_plate_data", "/Force_plate_data_sma"]
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

        randomized_msgs_compound: list[BagMessage] = list()
        randomized_msgs_compound.extend(msgs_fp)
        randomized_msgs_compound.extend(msgs_jp)
        compound_enumerated = list(enumerate(randomized_msgs_compound))
        msgs_compound_enumerated_sorted: list[BagMessage] = sorted(compound_enumerated, key=lambda x: x[1].timestamp)

        permutation = [i for i, msg in msgs_compound_enumerated_sorted]
        randomized_msgs_compound_sorted = [msg for i, msg in msgs_compound_enumerated_sorted]

        print(f"len: {len(randomized_msgs_compound_sorted)}")

        monte_carlo_set_count: int = 10

        params = [(topic_jp, topic_fp, msgs_fp, msgs_jp, permutation, i) for i in range(monte_carlo_set_count)]

        output: list[list[Tuple[Time, Joints_spatial_force]]]
        with Pool() as pool:
            output = pool.starmap(monte_carlo_set, params)

        for message_list in output:
            print(message_list[0][1].joints_bottom_up[0].m_xyz__f_xyz[3])

    return


def monte_carlo_set(topic_jp, topic_fp, msgs_fp, msgs_jp, permutation, i) -> "list[Tuple[Time, Joints_spatial_force]]":
    randomized_fp: list[BagMessage] = randomize_fp(msgs_fp=msgs_fp, max_norm=10)

    randomized_msgs_compound: list[BagMessage] = list()
    randomized_msgs_compound.extend(randomized_fp)
    randomized_msgs_compound.extend(msgs_jp)
    randomized_msgs_compound_sorted: list[BagMessage] = [randomized_msgs_compound[i] for i in permutation]

    timed_joints_spatial_force_list: list[Tuple[Time, Joints_spatial_force]] = create_timed_joints_spatial_force_list(topic_fp, topic_jp, randomized_msgs_compound_sorted)

    print(f"run: {i}")

    return timed_joints_spatial_force_list


def randomize_fp(msgs_fp: "list[BagMessage]", max_norm: float) -> "list[BagMessage]":
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


if __name__ == "__main__":
    execute()
