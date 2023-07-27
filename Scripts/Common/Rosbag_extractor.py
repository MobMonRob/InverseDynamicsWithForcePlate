import rosbag
from rosbag import Bag
import os


def printInfo(bagPath: str):
    with rosbag.Bag(bagPath) as bag:
        topicsInfo = bag.get_type_and_topic_info()[1]
        for topic in topicsInfo:
            print(topicsInfo[topic])


def getMsgListFromBag(bagPath: str, topic: str) -> list:
    msgList = []

    with rosbag.Bag(bagPath) as bag:
        for _topic, _msg, _t in bag.read_messages(topics=[topic]):
            msgList.append(_msg)

    print(f"Rosbag_extractor: Loaded {len(msgList)} messages of topic \"{topic}\" from bag \"{bagPath}\"")

    return msgList


def getSortedBagPathList(dirPath: str) -> "list[str]":
    bagPathList = []

    with os.scandir(dirPath) as it:
        for entry in it:
            if entry.name.endswith(".bag") and entry.is_file():
                bagPathList.append(entry.path)

    # inplace sort
    bagPathList.sort()

    return bagPathList


def getMsgListFromBagDir(dirPath: str, topic: str) -> list:
    bagPathList: list[str] = getSortedBagPathList(dirPath)

    compoundMsgList: list = []

    for bagPath in bagPathList:
        msgList: list = getMsgListFromBag(bagPath, topic)
        compoundMsgList.extend(msgList)
    
    print(f"Rosbag_extractor: Loaded {len(compoundMsgList)} messages of topic \"{topic}\" from {len(bagPathList)} files within dir \"{dirPath}\"")
    
    return compoundMsgList
