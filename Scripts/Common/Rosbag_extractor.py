from rosbag import Bag
from rosbag.bag import BagMessage
import os
from pathlib import Path
from Common import Valid_msgs_filter
from Common import Utils
from dataclasses import dataclass
from typing import TypeVar, Generic, Union, Set
import collections


class RosbagExtractor():
    def getFrameNumberToRosMsgs(self, topic: str) -> "dict[int, list]":
        bagPaths_to_msgs: dict[str, Msgs[BagMessage]] = self.indexedMsgs.get_bagPaths_to_msgs(topic=topic)
        ros_msgs: list = list()
        for msgs in bagPaths_to_msgs.values():
            ros_msgs.extend([bagMessage.message for bagMessage in msgs.msgs])
        frameNumber_to_msgs: dict[int, list] = Utils.groupMsgsOnFrameNumber(ros_msgs)
        Valid_msgs_filter.removeIncompleteFrameNumberGroups(frameNumber_to_msgs)
        return frameNumber_to_msgs

    def getIndexedBagMsgs(self) -> "IndexedBagMsgs":
        return self.indexedMsgs

    def getFrameNumberToRosMsgs(self, bagPath: str, topic: str) -> "dict[int, list]":
        msgs: BagMsgs = self.indexedMsgs.get_msgs(topic=topic, bagPath=bagPath)
        ros_msgs: list = [bagMessge.message for bagMessge in msgs.msgs]
        frameNumber_to_msgs: dict[int, list] = Utils.groupMsgsOnFrameNumber(ros_msgs)
        Valid_msgs_filter.removeIncompleteFrameNumberGroups(frameNumber_to_msgs)
        return frameNumber_to_msgs

    def __init__(self):
        self.topics: set[str] = []
        self.bagPaths: list[str] = []
        self.topicsToTypes: dict[str, str] = dict()
        self.indexedMsgs: IndexedBagMsgs = IndexedBagMsgs()

    @staticmethod
    def fromBag(bagPath: str, topics: "set[str]") -> "RosbagExtractor":
        re: RosbagExtractor = RosbagExtractor()
        re.topics: set[str] = topics
        re.bagPaths: list[str] = [bagPath]
        re.topicsToTypes: dict[str, str] = RosbagExtractor.__getTopicsToTypes(bagPath=bagPath, topics=topics)
        re.indexedMsgs: IndexedBagMsgs = RosbagExtractor.__getIndexedMsgsFromBag(bagPath=bagPath, _topics=topics)
        return re

    @staticmethod
    def fromDir(dirPath: str, topics: "set[str]") -> "RosbagExtractor":
        re: RosbagExtractor = RosbagExtractor()
        re.topics: set[str] = topics
        re.bagPaths: list[str] = RosbagExtractor.__getSortedBagPathList(dirPath=dirPath)
        re.topicsToTypes: dict[str, str] = RosbagExtractor.__getTopicsToTypes(bagPath=re.bagPaths[0], topics=topics)
        re.indexedMsgs: IndexedBagMsgs = RosbagExtractor.__getIndexedMsgsFromBags(bagPathList=re.bagPaths, topics=topics)
        return re

    @staticmethod
    def printInfo(bagPath: str):
        with Bag(bagPath) as bag:
            topicsInfo = bag.get_type_and_topic_info()[1]
            for topic in topicsInfo:
                print(f"{Path(__file__).stem}: topic \"{topic}\": {topicsInfo[topic]}")

    @staticmethod
    def __getTopicsToTypes(bagPath: str, topics: "set[str]") -> "dict[str, str]":
        """
        Assumes that the types in all bags are the same.
        """
        with Bag(bagPath) as bag:
            topicsInfo = bag.get_type_and_topic_info()[1]
            topicsToTypes: dict[str, str] = {topic: topicsInfo[topic].msg_type for topic in topicsInfo if topic in topics}
            return topicsToTypes

    @staticmethod
    def __getIndexedMsgsFromBag(bagPath: str, _topics: "set[str]") -> "IndexedBagMsgs":
        topics_to_msgs: dict[str, Msgs[BagMessage]] = {topic: Msgs(topic=topic, bagPath=bagPath, msgs=list()) for topic in _topics}

        with Bag(bagPath) as bag:
            for bagMessage in bag.read_messages(topics=_topics):
                topics_to_msgs.get(bagMessage.topic).msgs.append(bagMessage)

        for topic, msgs in topics_to_msgs.items():
            print(f"{Path(__file__).stem}: Loaded {len(msgs.msgs)} messages of topic \"{topic}\" from bag \"{bagPath}\"")

        indexedMsgs: IndexedBagMsgs = IndexedBagMsgs()
        indexedMsgs.add_msgs_list(list(topics_to_msgs.values()))

        return indexedMsgs

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
    def __getIndexedMsgsFromBags(bagPathList: "list[str]", topics: "set[str]") -> "IndexedBagMsgs":
        compound_indexedMsgs: IndexedBagMsgs = IndexedBagMsgs()

        for bagPath in bagPathList:
            indexedMsgs: IndexedBagMsgs = RosbagExtractor.__getIndexedMsgsFromBag(bagPath, topics)
            compound_indexedMsgs.add_indexedMsgs(indexedMsgs)

        return compound_indexedMsgs


T = TypeVar('T')


@dataclass
class Msgs(Generic[T]):
    topic: str
    bagPath: str
    msgs: "list[T]"


BagMsgs = Msgs[BagMessage]

IndexedMsgsKey = collections.namedtuple("IndexedMsgsKey", "topic, bagPath")


class IndexedMsgs(Generic[T]):
    def __init__(self):
        self.__topics_and_bagPaths_to_msgs: "dict[IndexedMsgsKey, Msgs[T]]" = dict()
        return

    def add_msgs(self, msgs: Msgs[T]):
        key = IndexedMsgs.__getKey(msgs)
        self.__ensure_key_not_present(key)
        self.__topics_and_bagPaths_to_msgs[key] = msgs
        return

    def add_msgs_list(self, msgs_list: "list[Msgs[T]]"):
        for msgs in msgs_list:
            key = IndexedMsgs.__getKey(msgs)
            self.__ensure_key_not_present(key)
        for msgs in msgs_list:
            key = IndexedMsgs.__getKey(msgs)
            self.__topics_and_bagPaths_to_msgs[key] = msgs
        return

    @staticmethod
    def __getKey(msgs: Msgs[T]) -> "IndexedMsgsKey":
        return IndexedMsgsKey(msgs.topic, msgs.bagPath)

    def add_indexedMsgs(self, other: "IndexedMsgs[T]"):
        for key in other.__topics_and_bagPaths_to_msgs.keys():
            self.__ensure_key_not_present(key)
        for key, msgs in other.__topics_and_bagPaths_to_msgs.items():
            self.__topics_and_bagPaths_to_msgs[key] = msgs
        return

    def __ensure_key_not_present(self, key: "IndexedMsgsKey"):
        if self.__topics_and_bagPaths_to_msgs.get(key, None) != None:
            raise RuntimeError(f"Key already present: \"{key}\"")
        return

    def get_msgs(self, topic: str, bagPath: str) -> Union["Msgs[T]", None]:
        return self.__topics_and_bagPaths_to_msgs.get((topic, bagPath), None)

    def get_keys(self) -> "set[IndexedMsgsKey]":
        return self.__topics_and_bagPaths_to_msgs.keys()

    def get_bagPaths_to_msgs(self, topic: str) -> "dict[str, Msgs[T]]":
        bagPaths_to_msgs: dict[str, Msgs[T]] = {_bagPath: msgs for (_topic, _bagPath), msgs in self.__topics_and_bagPaths_to_msgs.items() if _topic == topic}
        return bagPaths_to_msgs

    def get_topics_to_msgs(self, bagPath: str) -> "dict[str, Msgs[T]]":
        topics_to_msgs: dict[str, Msgs[T]] = {_topic: msgs for (_topic, _bagPath), msgs in self.__topics_and_bagPaths_to_msgs.items() if _bagPath == bagPath}
        return topics_to_msgs


IndexedBagMsgs = IndexedMsgs[BagMessage]
