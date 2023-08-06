import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common.Inverse_dynamics_bottom_up import Inverse_dynamics_force_plate_ur5e
from Common.Inverse_dynamics_top_down import Inverse_dynamics_top_down
import math
from Common.Rosbag_extractor import RosbagExtractor, IndexedBagMsgs, IndexedMsgsKey, BagMsgs
from Common.Ros_msg_types.ur_robot_data_acquisition.msg._Joint_parameters import Joint_parameters
import rospy
from Common.Inverse_dynamics_bottom_up import SixTuple
import pandas as pd


def execute():
    dirPath: str = "/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/Data/2023_08_04_ur5e_dynamic/"
    bagPath: str = f"{dirPath}start_position_to_dynamic_random_2023-08-04-19-08-59.bag"

    # sma ist hier womöglich zu langsam...
    topic_fp: str = "/Force_plate_data_sma"
    topic_jp: str = "/Joint_parameters"
    topics: set[str] = set([topic_fp, topic_jp])

    # _t: genpy.Time = _t
    re: RosbagExtractor = RosbagExtractor.fromBag(bagPath=bagPath, topics=topics)
    bagPaths: list[str] = re.bagPaths
    indexedBagMsgs: IndexedBagMsgs = re.getIndexedBagMsgs()

    bagMsgs_fp: BagMsgs = indexedBagMsgs.get_msgs(topic=topic_fp, bagPath=bagPath)
    bagMsgs_jp: BagMsgs = indexedBagMsgs.get_msgs(topic=topic_jp, bagPath=bagPath)

    _, _, timestamp0 = bagMsgs_jp.msgs[0]
    timestamp0: rospy.Time

    fieldNames: list[str] = list()
    for i in range(0, 6, 1):
        fieldNames.append(f"actual_joint_position_{i}")

    rows: list[list] = list()
    index: list[int] = list()
    for _topic, _message, _timestamp in bagMsgs_jp.msgs:
        row: list = list()

        _message: Joint_parameters
        _timestamp: rospy.Time
        index.append(_timestamp.to_nsec() - timestamp0.to_nsec())

        for i in range(0, 6, 1):
            row.append(_message.actual_joint_positions[i])

        rows.append(row)

    df: pd.DataFrame = pd.DataFrame(data=rows, columns=fieldNames, index=index)
    print(df)

    return

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
