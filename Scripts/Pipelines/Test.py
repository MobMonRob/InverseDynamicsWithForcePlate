import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common import Rosbag_extractor
from Common.Ros_msg_types.vicon_data_publisher.msg._Force_plate_data import Force_plate_data
from Common.Ros_msg_types.vicon_data_publisher.msg._Marker_global_translation import Marker_global_translation
from Common import FrameNumber_filter


def test():
    dirPath: str = "/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/Data/20Punkte_24.07.2023_1/"
    bagPath: str = f"{dirPath}2023-07-24-16-37-12.bag"
    topic_fp: str = "/Force_plate_data"
    topic_mgt: str = "/Marker_global_translation"
    topics: set[str] = set([topic_fp, topic_mgt])

    Rosbag_extractor.printInfo(bagPath)

    compound_topics_to_msgs: dict[str, list] = Rosbag_extractor.getTopicsToMsgsFromDir(dirPath, topics)
    # compound_topics_to_msgs: dict[str, list] = Rosbag_extractor.getTopicsToMsgsFromBag(bagPath, topics)

    compound_topics_to_frameNumbers_to_msgs: dict[str, dict[int, list]] = FrameNumber_filter.groupOnFrameNumber(compound_topics_to_msgs)
    FrameNumber_filter.removeMsgsWithIndividualFrameNumbers(compound_topics_to_frameNumbers_to_msgs)


if __name__ == "__main__":
    test()
