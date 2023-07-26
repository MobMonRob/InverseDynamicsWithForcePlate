import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common import Rosbag_extractor
from Common.Ros_msg_types.vicon_data_publisher.msg._Force_plate_data import Force_plate_data


def test():
    path: str = "/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/Data/2023-07-25-16-30-34_duenne_Meissel.bag"
    topic: str = "/Force_plate_data"
    msgs: list[Force_plate_data] = Rosbag_extractor.getMsgList(path, topic)
    for msg in msgs:
        print(msg.frameNumber)


if __name__ == "__main__":
    test()
