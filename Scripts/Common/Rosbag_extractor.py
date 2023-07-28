import rosbag
from rosbag import Bag
import os
from pathlib import Path


def printInfo(bagPath: str):
    with rosbag.Bag(bagPath) as bag:
        topicsInfo = bag.get_type_and_topic_info()[1]
        for topic in topicsInfo:
            print(f"{Path(__file__).stem}: topic \"{topic}\": {topicsInfo[topic]}")


def getTopicsToMsgsFromBag(bagPath: str, _topics: "set[str]") -> "dict[str, list]":
    topics_to_msgs: dict[str, list] = dict()
    for topic in _topics:
        _list = []
        topics_to_msgs.update({topic: _list})

    with rosbag.Bag(bagPath) as bag:
        for _topic, _msg, _t in bag.read_messages(topics=_topics):
            topics_to_msgs.get(_topic).append(_msg)

    for topic, msgs in topics_to_msgs.items():
        print(f"{Path(__file__).stem}: Loaded {len(msgs)} messages of topic \"{topic}\" from bag \"{bagPath}\"")

    return topics_to_msgs


def getSortedBagPathList(dirPath: str) -> "list[str]":
    bagPathList = []

    with os.scandir(dirPath) as it:
        for entry in it:
            if entry.name.endswith(".bag") and entry.is_file():
                bagPathList.append(entry.path)

    # inplace sort
    bagPathList.sort()

    return bagPathList


def getTopicsToMsgsFromDir(dirPath: str, topics: "set[str]") -> "dict[str, list]":
    compound_topics_to_msgs: dict[str, list] = dict()
    for topic in topics:
        _list = []
        compound_topics_to_msgs.update({topic: _list})

    bagPathList: list[str] = getSortedBagPathList(dirPath)

    for bagPath in bagPathList:
        topics_to_msgs: dict[str, list] = getTopicsToMsgsFromBag(bagPath, topics)
        for topic, msgs in topics_to_msgs.items():
            compound_topics_to_msgs.get(topic).extend(msgs)

    for topic, msgs in compound_topics_to_msgs.items():
        print(f"{Path(__file__).stem}: Loaded {len(msgs)} messages of topic \"{topic}\" from {len(bagPathList)} files within dir \"{dirPath}\"")

    return compound_topics_to_msgs
