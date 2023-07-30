from rosbag import Bag
import os
from pathlib import Path
from Common import Valid_msgs_filter
from Common.Ros_msg_types.vicon_data_publisher.msg._Force_plate_data import Force_plate_data
from Common.Ros_msg_types.vicon_data_publisher.msg._Marker_global_translation import Marker_global_translation
from Common import Utils


class RosbagExtractor():
    def __init__(self):
        self.topics: set[str] = []
        self.bagPaths: list[str] = []
        self.topicsToTypes: dict[str, str] = dict()
        self.topics_to_frameNumber_to_msgs: dict[str, dict[int, list]] = dict()


    @staticmethod
    def fromBag(bagPath: str, topics: "set[str]") -> "RosbagExtractor": 
        re: RosbagExtractor = RosbagExtractor()
        re.topics: set[str] = topics
        re.bagPaths: list[str] = [bagPath]
        re.topicsToTypes: dict[str, str] = RosbagExtractor.__getTopicsToTypes(bagPath=bagPath, topics=topics)
        topics_to_msgs: dict[str, list] = RosbagExtractor.__getTopicsToMsgsFromBag(bagPath=bagPath, _topics=topics)
        re.topics_to_frameNumber_to_msgs = Utils.groupMsgsOnFrameNumber_topic(topics_to_msgs)
        re.__removeInvalidMessages()
        return re


    @staticmethod
    def fromDir(dirPath: str, topics: "set[str]") -> "RosbagExtractor": 
        re: RosbagExtractor = RosbagExtractor()
        re.topics: set[str] = topics
        re.bagPaths: list[str] = RosbagExtractor.__getSortedBagPathList(dirPath=dirPath)
        re.topicsToTypes: dict[str, str] = RosbagExtractor.__getTopicsToTypes(bagPath=re.bagPaths[0], topics=topics)
        topics_to_msgs: dict[str, list] = RosbagExtractor.__getTopicsToMsgsFromBags(bagPathList=re.bagPaths, topics=topics)
        re.topics_to_frameNumber_to_msgs = Utils.groupMsgsOnFrameNumber_topic(topics_to_msgs)
        re.__removeInvalidMessages()
        return re


    @staticmethod
    def printInfo(bagPath: str):
        with Bag(bagPath) as bag:
            topicsInfo = bag.get_type_and_topic_info()[1]
            for topic in topicsInfo:
                print(f"{Path(__file__).stem}: topic \"{topic}\": {topicsInfo[topic]}")


    def getframeNumberToMsgs(self, topic: str) -> "dict[int, list]":
        return self.topics_to_frameNumber_to_msgs[topic]


    def __removeInvalidMessages(self):
        # TODO: typ-spezifische validierungen einfügen
        for topic, frameNumber_to_msgs in self.topics_to_frameNumber_to_msgs.items():
            Valid_msgs_filter.removeIncompleteFrameNumberGroups(frameNumber_to_msgs)
            # type: str = self.topicsToTypes[topic]
            # if (type == Force_plate_data._type):
            #     pass
            # elif (type == Marker_global_translation._type):
            #     pass
            # else:
            #     raise RuntimeError(f"{Path(__file__).stem}: topic \"{topic}\" is of unknown type \"{type}\".")
        return


    @staticmethod
    def __getTopicsToTypes(bagPath: str, topics: "set[str]") -> "dict[str, str]":
        """
        Assumes that the types in all bags are the same.
        """
        with Bag(bagPath) as bag:
            topicsInfo = bag.get_type_and_topic_info()[1]
            topicsToTypes: dict[str, str] = {topic : topicsInfo[topic].msg_type for topic in topicsInfo if topic in topics}
            return topicsToTypes


    @staticmethod
    def __getTopicsToMsgsFromBag(bagPath: str, _topics: "set[str]") -> "dict[str, list]":
        topics_to_msgs: dict[str, list] = {topic: list() for topic in _topics}

        with Bag(bagPath) as bag:
            for _topic, _msg, _t in bag.read_messages(topics=_topics):
                topics_to_msgs.get(_topic).append(_msg)

        for topic, msgs in topics_to_msgs.items():
            print(f"{Path(__file__).stem}: Loaded {len(msgs)} messages of topic \"{topic}\" from bag \"{bagPath}\"")

        return topics_to_msgs


    @staticmethod
    def __getSortedBagPathList(dirPath: str) -> "list[str]":
        bagPathList = []

        with os.scandir(dirPath) as it:
            for entry in it:
                if entry.name.endswith(".bag") and entry.is_file():
                    bagPathList.append(entry.path)

        # inplace sort
        bagPathList.sort()

        return bagPathList


    @staticmethod
    def __getTopicsToMsgsFromBags(bagPathList: "list[str]", topics: "set[str]") -> "dict[str, list]":
        compound_topics_to_msgs: dict[str, list] = {topic: list() for topic in topics}

        for bagPath in bagPathList:
            topics_to_msgs: dict[str, list] = RosbagExtractor.__getTopicsToMsgsFromBag(bagPath, topics)
            for topic, msgs in topics_to_msgs.items():
                compound_topics_to_msgs.get(topic).extend(msgs)

        for topic, msgs in compound_topics_to_msgs.items():
            print(f"{Path(__file__).stem}: Loaded {len(msgs)} messages of topic \"{topic}\" from {len(bagPathList)} bag files.")

        return compound_topics_to_msgs
