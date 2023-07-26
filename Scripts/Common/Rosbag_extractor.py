import rosbag
from rosbag import Bag


def printInfo(path: str, topic: str):
    bag: Bag = rosbag.Bag(path)

    topicsInfo = bag.get_type_and_topic_info()[1]
    print(topicsInfo[topic])

    bag.close()


def getMsgList(path: str, topic: str) -> list:
    # print(f"path: \"{path}\"")
    # print(f"topic: \"{topic}\"")

    bag: Bag = rosbag.Bag(path)

    msgList = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        msgList.append(msg)
    bag.close()
    return msgList
