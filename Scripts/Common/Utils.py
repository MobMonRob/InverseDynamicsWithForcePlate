from typing import TypeVar, Generic
import pandas as pd
from Common.geometry_classes import Point3D
from collections import defaultdict


T = TypeVar('T')


def mergeFrameNumberToList(frameNumbers_to_list: "dict[int, list[T]]") -> "list[T]":
    merged_msgs: list[T] = []
    for msgs in frameNumbers_to_list.values():
        merged_msgs.extend(msgs)
    return merged_msgs


def groupMsgsOnFrameNumber_topic(topics_to_msgs: "dict[str, list]") -> "dict[str, dict[int, list]]":
    topics_to_frameNumbers_to_msgs: dict[str, dict[int, list]] = dict()
    for topic, msgs in topics_to_msgs.items():
        grouped_msgs: dict[int, list] = groupMsgsOnFrameNumber(msgs)
        topics_to_frameNumbers_to_msgs.update({topic: grouped_msgs})

    return topics_to_frameNumbers_to_msgs


def groupMsgsOnFrameNumber(msgs: "list") -> "dict[int, list]":
    """
    same dict key <=> same frameNumber
    """
    frameNumber_to_list: dict[int, list] = defaultdict(None)
    for msg in msgs:
        msg_list = frameNumber_to_list.get(msg.frameNumber)
        if msg_list == None:
            msg_list = []
            msg_list.append(msg)
            frameNumber_to_list.update({msg.frameNumber: msg_list})
        else:
            msg_list.append(msg)

    return frameNumber_to_list


def listToDataFrame(theList: "list[T]") -> pd.DataFrame:
    fieldNames: list[str] = theList[0].__slots__
    values = [[getattr(element, fieldName) for fieldName in fieldNames] for element in theList]
    df: pd.DataFrame = pd.DataFrame(values, columns=fieldNames)
    return df

