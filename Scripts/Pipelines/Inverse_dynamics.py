import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common.Rosbag_extractor import RosbagExtractor, IndexedBagMsgs, BagMsgs, BagMessage
from Common.Inverse_dynamics_node import Inverse_dynamics_node
from Common.Bland_Altman_Plot import BAP_config, BAP_set, generate_bland_altman_plot
from itertools import cycle
from Common.Ros_msg_types.data_transformation.msg import Joints_spatial_force


def execute():
    rootDir: str = os.path.abspath(f"{SCRIPT_DIR}/../..")
    dataDir: str = f"{rootDir}/Data/"
    bagPath: str = f"{dataDir}2023_08_04_ur5e_dynamic/start_position_to_dynamic_random_2023-08-04-19-08-59.bag"
    # bagPath: str = f"{dataDir}2023_08_04_ur5e_static/static_south_2023-08-04-18-20-12.bag"
    plotSaveDir: str = f"{rootDir}/Plots/"

    # TODO: sma austauschen durch 1euro
    topic_fp: str = "/Force_plate_data"
    topic_jp: str = "/Joint_parameters"
    topics: set[str] = set([topic_fp, topic_jp])

    re: RosbagExtractor = RosbagExtractor.fromBag(bagPath=bagPath, topics=topics)

    bagPaths: list[str] = re.bagPaths
    indexedBagMsgs: IndexedBagMsgs = re.getIndexedBagMsgs()

    bagMsgs_fp: BagMsgs = indexedBagMsgs.get_msgs(topic=topic_fp, bagPath=bagPath)
    bagMsgs_jp: BagMsgs = indexedBagMsgs.get_msgs(topic=topic_jp, bagPath=bagPath)

    msgs_fp: list[BagMessage] = bagMsgs_fp.msgs
    msgs_jp: list[BagMessage] = bagMsgs_jp.msgs

    msgs_compound: list[BagMessage] = list()
    msgs_compound.extend(msgs_fp)
    msgs_compound.extend(msgs_jp)

    msgs_compound_sorted: list[BagMessage] = sorted(msgs_compound, key=lambda x: x.timestamp)

    iv_node: Inverse_dynamics_node = Inverse_dynamics_node()
    joints_spatial_force_list: list[Joints_spatial_force] = list()

    for topic, message, timestamp in msgs_compound_sorted:
        if topic == topic_fp:
            iv_node.force_plate_data(fpd=message, time=timestamp)
            continue
        elif topic == topic_jp:
            joints_spatial_force: Joints_spatial_force = iv_node.joint_parameters(jp=message, time=timestamp)
            if joints_spatial_force == None:
                continue
            else:
                joints_spatial_force_list.append(joints_spatial_force)

    # print(joints_spatial_force_list[0].joints_bottom_up[0].m_xyz__f_xyz[0])

    m_xyz__f_xyz__to__joints_bottom_up_0: list[list[float]] = [[] for i in range(6)]
    m_xyz__f_xyz__to__joints_top_down_0: list[list[float]] = [[] for i in range(6)]
    joint: int = 0
    for joints_spatial_force in joints_spatial_force_list:
        bottom_up_0 = joints_spatial_force.joints_bottom_up[joint].m_xyz__f_xyz
        top_down_0 = joints_spatial_force.joints_top_down[joint].m_xyz__f_xyz

        for i in range(6):
            m_xyz__f_xyz__to__joints_bottom_up_0[i].append(bottom_up_0[i])
            m_xyz__f_xyz__to__joints_top_down_0[i].append(top_down_0[i])

    # in sets: alle m_xyz__f_xyz
    # in BAP_set alle joints[0] aller Elemente aus joints_spatial_force_list
    sets: list[BAP_set] = list()
    for i in range(6):
        sets.append(BAP_set(x1=m_xyz__f_xyz__to__joints_bottom_up_0[i], x2=m_xyz__f_xyz__to__joints_top_down_0[i]))

    colors = list()
    colors.extend(["b", "g", "r", "c", "m", "y"])

    while True:
        # m{x, y, z}: Nm
        # f{x, y, z}: N
        # Joint{i}: (mx, my, mz, fx, mf, fz)
        # Joints = (Joint0 .. Joint5)
        # Joints_list: [Joints, ...]
        dataName1 = "Spatial Kraft bottom_up, Gelenk 0"
        dataName2 = "Spatial Kraft top_down, Gelenk 0"
        units = "[Nm / N]"
        config: BAP_config = BAP_config(sets=sets, colors=iter(colors), dataName1=dataName1, dataName2=dataName2, units=units, additionalComment="", plotSaveDir=plotSaveDir)

        # Plotten
        generate_bland_altman_plot(config=config, showplot=True)

    return


if __name__ == "__main__":
    execute()
