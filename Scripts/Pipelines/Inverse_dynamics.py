import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common.Rosbag_extractor import RosbagExtractor, IndexedBagMsgs, BagMsgs, BagMessage
from Common.Inverse_dynamics_node import Inverse_dynamics_node
from Common.Bland_Altman_Plot import BAP_config, BAP_set, BAP_legend, generate_bland_altman_plot
from Common.Simple_moving_average import SimpleMovingAverageOnObjects
from Common.Ros_msg_types.vicon_data_publisher.msg._Force_plate_data import Force_plate_data
from itertools import cycle, product
from Common.Ros_msg_types.data_transformation.msg import Joints_spatial_force, Spatial_force
from typing import Tuple
import pandas as pd
from pandas import DataFrame
import numpy as np
from numpy.linalg import norm
from rospy import Time
from Common.one_euro_filter import OneEuroFilterOnObjects
import matplotlib


def execute():
    rootDir: str = os.path.abspath(f"{SCRIPT_DIR}/../..")
    dataDir: str = f"{rootDir}/Data/"
    # relativeBagPath: str = f"2023_08_04_ur5e_dynamic/end_position_to_dynamic_random_2023-08-04-19-18-34.bag"
    relativeBagPath: str = f"2023_08_04_ur5e_dynamic/start_position_to_dynamic_random_2023-08-04-19-08-59.bag"
    bagPath: str = f"{dataDir}{relativeBagPath}"
    # bagPath: str = f"{dataDir}2023_08_04_ur5e_static/static_south_2023-08-04-18-20-12.bag"
    plotSaveDir: str = f"{rootDir}/Plots/Inverse_dynamics/{relativeBagPath}"

    topics_fp: list[str] = ["/Force_plate_data", "/Force_plate_data_sma"]
    topic_jp: str = "/Joint_parameters"

    topics: set[str] = set([*topics_fp, topic_jp])
    re: RosbagExtractor = RosbagExtractor.fromBag(bagPath=bagPath, topics=topics)
    indexedBagMsgs: IndexedBagMsgs = re.getIndexedBagMsgs()

    for topic_fp in topics_fp:
        plotSaveDir_with_topic: str = f"{plotSaveDir}{topic_fp}/"

        # Workaround to calculate sma with window size 100 afterwards.
        calculate_sma: bool = False
        if topic_fp == "/Force_plate_data_sma":
            topic_fp = "/Force_plate_data"
            calculate_sma = True
        msgs_compound_sorted: list[BagMessage] = create_msgs_compound_sorted(indexedBagMsgs, topic_fp, topic_jp, bagPath, calculate_sma)

        joints_spatial_force_list: list[Joints_spatial_force] = [sf for time, sf in create_timed_joints_spatial_force_list(topic_fp, topic_jp, msgs_compound_sorted)]

        force_to_joint_plot(joints_spatial_force_list, plotSaveDir_with_topic)

        bu_df, td_df = create_bu_td_forces_joints_to_values(joints_spatial_force_list)
        norm_to_joint_plot(bu_df, td_df, plotSaveDir_with_topic)

    return


def create_bu_td_forces_joints_to_values(joints_spatial_force_list: "list[Joints_spatial_force]") -> Tuple[pd.DataFrame, pd.DataFrame]:
    # Alternative wäre, Subklasse von dict zu machen, die ein Schlüssel einfügt, wenn nicht vorhanden.
    bu_forces_joints_to_values: dict[(int, int), list[float]] = {(i, j): [] for i, j in product(range(6), range(6))}
    td_forces_joints_to_values: dict[(int, int), list[float]] = {(i, j): [] for i, j in product(range(6), range(6))}
    for joints_spatial_force in joints_spatial_force_list:
        joints_forces_bottom_up: list[Spatial_force] = joints_spatial_force.joints_bottom_up
        joints_forces_top_down = joints_spatial_force.joints_top_down
        for joint_index in range(6):
            joint_forces_bottom_up: list[float] = joints_forces_bottom_up[joint_index].m_xyz__f_xyz
            joint_forces_top_down: list[float] = joints_forces_top_down[joint_index].m_xyz__f_xyz
            for force_component_index in range(6):
                key: Tuple[int, int] = (force_component_index, joint_index)

                joint_force_bottom_up: float = joint_forces_bottom_up[force_component_index]
                joint_force_top_down: float = joint_forces_top_down[force_component_index]

                bu_forces_joints_to_values[key].append(joint_force_bottom_up)
                td_forces_joints_to_values[key].append(joint_force_top_down)

    # Alternative, von columns nach multiindex: columns machen und dann pivot_table
    # Andersherum, von multiindex nach columns: unstack
    size = len(joints_spatial_force_list)
    bu = np.concatenate(list(bu_forces_joints_to_values.values()))
    td = np.concatenate(list(td_forces_joints_to_values.values()))
    mi = pd.MultiIndex.from_product(iterables=[range(6), range(6), range(size)], names=["spatial_force", "joint", "index"])
    bu_df = pd.DataFrame(data=bu, index=mi, columns=["value"])
    td_df = pd.DataFrame(data=td, index=mi, columns=["value"])
    return bu_df, td_df


def norm_m_f(df: DataFrame) -> DataFrame:
    df = df.unstack(level="spatial_force")
    df = df.apply(func=lambda row: (norm(row[0:3]), norm(row[3:6])), axis="columns", result_type="expand")
    df.columns = ["m", "f"]
    return df


def norm_to_joint_plot(bu_df: DataFrame, td_df: DataFrame, plotSaveDir: str):
    # bu_forces_joints_to_values

    # Alternativ hätte man das auch mit einem ros node machen können.
    # Bzw. über den Kern des nodes drüber iterieren, ähnlich wie in create_joints_spatial_force_list.
    # Aber das war gute pandas Übung.
    bu_df_normed_m_f = norm_m_f(bu_df)
    td_df_normed_m_f = norm_m_f(td_df)

    bu_df_normed_m_f = bu_df_normed_m_f.unstack(level="joint")
    td_df_normed_m_f = td_df_normed_m_f.unstack(level="joint")

    bu_joint_to_normed_m_values: list[list[float]] = [bu_df_normed_m_f[("m", i)].values.tolist() for i in range(6)]
    bu_joint_to_normed_f_values: list[list[float]] = [bu_df_normed_m_f[("f", i)].values.tolist() for i in range(6)]
    td_joint_to_normed_m_values: list[list[float]] = [td_df_normed_m_f[("m", i)].values.tolist() for i in range(6)]
    td_joint_to_normed_f_values: list[list[float]] = [td_df_normed_m_f[("f", i)].values.tolist() for i in range(6)]

    sets_m: list[BAP_set] = [BAP_set(x1=bu_joint_to_normed_m_values[i], x2=td_joint_to_normed_m_values[i]) for i in range(6)]
    sets_f: list[BAP_set] = [BAP_set(x1=bu_joint_to_normed_f_values[i], x2=td_joint_to_normed_f_values[i]) for i in range(6)]

    colors_joints = list()
    colors_joints.extend(["b", "r", "g", "m", "c", "darkorange"])

    dataName1 = f"BURNEA"
    dataName2 = f"RNEA"
    plotSaveDir = f"{plotSaveDir}norm_to_joint/"

    config_m: BAP_config = BAP_config(sets=sets_m, colors=iter(colors_joints), dataName1=dataName1, dataName2=dataName2,
                                      units="Nm", additionalComment=f"(M für alle Gelenke)", plotSaveDir=plotSaveDir)
    config_f: BAP_config = BAP_config(sets=sets_f, colors=iter(colors_joints), dataName1=dataName1, dataName2=dataName2,
                                      units="N", additionalComment=f"(F für alle Gelenke)", plotSaveDir=plotSaveDir)

    color_to_label = {c: i+1 for (i, c) in enumerate(colors_joints)}
    legend: BAP_legend = BAP_legend(title="Gelenke", color_to_label=color_to_label)

    # Plotten
    generate_bland_altman_plot(config=config_m, showplot=False, plot_outliers=False, legend=legend)
    generate_bland_altman_plot(config=config_f, showplot=False, plot_outliers=False, legend=legend)

    return


def create_msgs_compound_sorted(indexedBagMsgs: IndexedBagMsgs, topic_fp: str, topic_jp: str, bagPath: str, calculate_sma: bool = False) -> "list[BagMessage]":
    window_size = 100

    # Start: msgs_compound_sorted
    sma: SimpleMovingAverageOnObjects[Force_plate_data] = SimpleMovingAverageOnObjects[Force_plate_data](window_size=window_size, sample=Force_plate_data())

    # one_euro: OneEuroFilterOnObjects[Force_plate_data] = OneEuroFilterOnObjects[Force_plate_data](sample=Force_plate_data())

    bagMsgs_fp: BagMsgs = indexedBagMsgs.get_msgs(topic=topic_fp, bagPath=bagPath)
    bagMsgs_jp: BagMsgs = indexedBagMsgs.get_msgs(topic=topic_jp, bagPath=bagPath)

    msgs_fp: list[BagMessage] = bagMsgs_fp.msgs
    msgs_jp: list[BagMessage] = bagMsgs_jp.msgs

    if calculate_sma == True:
        msgs_fp = [BagMessage(topic=topic, message=sma.process(message), timestamp=timestamp) for topic, message, timestamp in msgs_fp]
        # msgs_fp = [BagMessage(topic=topic, message=one_euro.process(message), timestamp=timestamp) for topic, message, timestamp in msgs_fp]
        msgs_fp = msgs_fp[window_size:len(msgs_fp)]
        msgs_jp = msgs_jp[window_size:len(msgs_jp)]

    msgs_compound: list[BagMessage] = list()
    msgs_compound.extend(msgs_fp)
    msgs_compound.extend(msgs_jp)

    msgs_compound_sorted: list[BagMessage] = sorted(msgs_compound, key=lambda x: x.timestamp)
    # End: msgs_compound_sorted

    return msgs_compound_sorted


def create_timed_joints_spatial_force_list(topic_fp: str, topic_jp: str, msgs_compound_sorted: "list[BagMessage]") -> "list[Tuple[Time, Joints_spatial_force]]":
    # Start: joints_spatial_force_list
    iv_node: Inverse_dynamics_node = Inverse_dynamics_node()
    timed_joints_spatial_force_list: list[(Time, Joints_spatial_force)] = list()

    for topic, message, timestamp in msgs_compound_sorted:
        if topic == topic_fp:
            iv_node.force_plate_data(fpd=message, time=timestamp)
            continue
        elif topic == topic_jp:
            joints_spatial_force: Joints_spatial_force = iv_node.joint_parameters(jp=message, time=timestamp)
            if joints_spatial_force == None:
                continue
            else:
                timed_joints_spatial_force_list.append((timestamp, joints_spatial_force))
    # End: joints_spatial_force_list

    return timed_joints_spatial_force_list


def force_to_joint(joints_spatial_force_list: "list[Joints_spatial_force]", plotSaveDir: str) -> "Tuple[list[list[list[float]]], list[list[list[float]]]]":
    # m{x, y, z}: Nm
    # f{x, y, z}: N
    # Joint{i}: (mx, my, mz, fx, mf, fz)
    # Joints = (Joint0 .. Joint5)
    # Joints_list: [Joints, ...]

    # print(joints_spatial_force_list[0].joints_bottom_up[0].m_xyz__f_xyz[0])

    # End: joints_spatial_force_list

    # Start: force_to_joint

    # ein Plot pro Kraftkomponente
    # in sets: alle joints
    # in BAP_set eine Kraftkomponente eines Joints

    bottom_up__force_components__to__joints__to__datapoints: list[list[list[float]]] = [[[] for joint in range(6)] for force in range(6)]
    top_down__force_components__to__joints__to__datapoints: list[list[list[float]]] = [[[] for joint in range(6)] for force in range(6)]

    for joints_spatial_force in joints_spatial_force_list:
        joints_forces_bottom_up: list[Spatial_force] = joints_spatial_force.joints_bottom_up
        joints_forces_top_down = joints_spatial_force.joints_top_down
        for joint_index in range(6):
            joint_forces_bottom_up: list[float] = joints_forces_bottom_up[joint_index].m_xyz__f_xyz
            joint_forces_top_down: list[float] = joints_forces_top_down[joint_index].m_xyz__f_xyz
            for force_component_index in range(6):
                joint_force_bottom_up: float = joint_forces_bottom_up[force_component_index]
                joint_force_top_down: float = joint_forces_top_down[force_component_index]

                bottom_up__force_components__to__joints__to__datapoints[force_component_index][joint_index].append(joint_force_bottom_up)

                top_down__force_components__to__joints__to__datapoints[force_component_index][joint_index].append(joint_force_top_down)

    # End: force_to_joint

    return bottom_up__force_components__to__joints__to__datapoints, top_down__force_components__to__joints__to__datapoints


def force_to_joint_plot(joints_spatial_force_list: "list[Joints_spatial_force]", plotSaveDir: str):
    bottom_up__force_components__to__joints__to__datapoints, top_down__force_components__to__joints__to__datapoints = force_to_joint(joints_spatial_force_list, plotSaveDir)

    force_component__to__sets_joints_to_datapoints: list[list[BAP_set]] = list()
    for force_component_index in range(6):
        sets_joint_to_datapoints: list[BAP_set] = list()
        bottom_up__joints__to__datapoints = bottom_up__force_components__to__joints__to__datapoints[force_component_index]
        top_down__joints__to__datapoints = top_down__force_components__to__joints__to__datapoints[force_component_index]
        for joint_index in range(6):
            bottom_up__joint__to__datapoints = bottom_up__joints__to__datapoints[joint_index]
            top_down__joint__to__datapoints = top_down__joints__to__datapoints[joint_index]
            sets_joint_to_datapoints.append(BAP_set(x1=bottom_up__joint__to__datapoints, x2=top_down__joint__to__datapoints))
        force_component__to__sets_joints_to_datapoints.append(sets_joint_to_datapoints)

    colors_joints = list()
    colors_joints.extend(["b", "r", "g", "m", "c", "darkorange"])

    forces_names: list[str] = ["Mx", "My", "Mz", "Fx", "Fy", "Fz"]
    forces_units: list[str] = ["Nm", "Nm", "Nm", "N", "N", "N"]

    plotSaveDir = f"{plotSaveDir}force_to_joint/"

    color_to_label = {c: i+1 for (i, c) in enumerate(colors_joints)}
    legend: BAP_legend = BAP_legend(title="Gelenke", color_to_label=color_to_label)

    for force_component_index in range(6):
        force_name: str = forces_names[force_component_index]
        force_units: str = forces_units[force_component_index]
        sets_joint_to_datapoints = force_component__to__sets_joints_to_datapoints[force_component_index]

        dataName1 = f"BURNEA"
        dataName2 = f"RNEA"
        units = f"{force_units}"
        config: BAP_config = BAP_config(sets=sets_joint_to_datapoints, colors=iter(colors_joints), dataName1=dataName1, dataName2=dataName2,
                                        units=units, additionalComment=f"({force_name} für alle Gelenke)", plotSaveDir=plotSaveDir)

        # Plotten
        generate_bland_altman_plot(config=config, showplot=False, plot_outliers=False, legend=legend)

    return


if __name__ == "__main__":
    execute()
