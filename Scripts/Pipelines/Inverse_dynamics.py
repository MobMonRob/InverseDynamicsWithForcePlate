import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common.Rosbag_extractor import RosbagExtractor, IndexedBagMsgs, BagMsgs, BagMessage
from Common.Inverse_dynamics_node import Inverse_dynamics_node
from Common.Bland_Altman_Plot import BAP_config, BAP_set, generate_bland_altman_plot
from itertools import cycle
from Common.Ros_msg_types.data_transformation.msg import Joints_spatial_force, Spatial_force


def execute():
    rootDir: str = os.path.abspath(f"{SCRIPT_DIR}/../..")
    dataDir: str = f"{rootDir}/Data/"
    relativeBagPath: str = f"2023_08_04_ur5e_dynamic/end_position_to_dynamic_random_2023-08-04-19-18-34.bag"
    bagPath = f"{dataDir}{relativeBagPath}"
    # bagPath: str = f"{dataDir}2023_08_04_ur5e_static/static_south_2023-08-04-18-20-12.bag"
    plotSaveDir: str = f"{rootDir}/Plots/Inverse_dynamics/force_to_joint/{relativeBagPath}/"

    # TODO: sma austauschen durch 1euro
    topic_fp: str = "/Force_plate_data"
    topic_jp: str = "/Joint_parameters"
    topics: set[str] = set([topic_fp, topic_jp])

    re: RosbagExtractor = RosbagExtractor.fromBag(bagPath=bagPath, topics=topics)

    # Start: msgs_compound_sorted
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
    # End: msgs_compound_sorted

    # Start: joints_spatial_force_list
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

    # m{x, y, z}: Nm
    # f{x, y, z}: N
    # Joint{i}: (mx, my, mz, fx, mf, fz)
    # Joints = (Joint0 .. Joint5)
    # Joints_list: [Joints, ...]

    # print(joints_spatial_force_list[0].joints_bottom_up[0].m_xyz__f_xyz[0])

    # End: joints_spatial_force_list

    # # Start: Joint0 m_xyz__f_xyz
    # m_xyz__f_xyz__to__joints_bottom_up_0: list[list[float]] = [[] for i in range(6)]
    # m_xyz__f_xyz__to__joints_top_down_0: list[list[float]] = [[] for i in range(6)]
    # joint: int = 0
    # for joints_spatial_force in joints_spatial_force_list:
    #     bottom_up_0 = joints_spatial_force.joints_bottom_up[joint].m_xyz__f_xyz
    #     top_down_0 = joints_spatial_force.joints_top_down[joint].m_xyz__f_xyz

    #     for i in range(6):
    #         m_xyz__f_xyz__to__joints_bottom_up_0[i].append(bottom_up_0[i])
    #         m_xyz__f_xyz__to__joints_top_down_0[i].append(top_down_0[i])

    # # in sets: alle m_xyz__f_xyz
    # # in BAP_set alle joints[0] aller Elemente aus joints_spatial_force_list
    # sets: list[BAP_set] = list()
    # for i in range(6):
    #     sets.append(BAP_set(x1=m_xyz__f_xyz__to__joints_bottom_up_0[i], x2=m_xyz__f_xyz__to__joints_top_down_0[i]))
    # # End: Joint0 m_xyz__f_xyz

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

    # End: force_to_joint

    colors_joints = list()
    colors_joints.extend(["b", "g", "r", "c", "m", "y"])

    forces_names: list[str] = ["mx", "my", "mz", "fx", "fy", "fz"]
    forces_units: list[str] = ["Nm", "Nm", "Nm", "N", "N", "N"]

    for force_component_index in range(6):
        force_name: str = forces_names[force_component_index]
        force_units: str = forces_units[force_component_index]
        sets_joint_to_datapoints = force_component__to__sets_joints_to_datapoints[force_component_index]

        dataName1 = f"bottom_up"
        dataName2 = f"top_down"
        units = f"[{force_units}]"
        config: BAP_config = BAP_config(sets=sets_joint_to_datapoints, colors=iter(colors_joints), dataName1=dataName1, dataName2=dataName2,
                                        units=units, additionalComment=f"({force_name} für alle Gelenke)", plotSaveDir=plotSaveDir)

        # Plotten
        generate_bland_altman_plot(config=config, showplot=False)

    return


if __name__ == "__main__":
    execute()
