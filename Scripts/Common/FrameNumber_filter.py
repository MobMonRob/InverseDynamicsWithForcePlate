from Common.Ros_msg_types.vicon_data_publisher.msg._Force_plate_data import Force_plate_data
from Common.Ros_msg_types.vicon_data_publisher.msg._Marker_global_translation import Marker_global_translation
from collections import defaultdict
from typing import TypeVar, Generic

def filter(msgs_fp: "list[Force_plate_data]", msgs_mgt: "list[Marker_global_translation]"):
    # same dict key <=> same frameNumber (different subsampleNumber)
    dict_fp: dict[int, list[Force_plate_data]] = groupOnFrameNumber(msgs_fp)

    for msg in dict_fp.values():
        print(len(msg))
        break
    
    print(len(dict_fp))
    
    # same dict key <=> same frameNumber (different markerNumber)
    dict_mgt: dict[int, list[Marker_global_translation]] = groupOnFrameNumber(msgs_mgt)


T = TypeVar('T')
# same dict key <=> same frameNumber
def groupOnFrameNumber(msgs: "list[T]") -> "dict[int, list[T]]" :
    frameNumber_to_list: dict[int, list[T]] = defaultdict(None)
    for msg in msgs:
        msg_list = frameNumber_to_list.get(msg.frameNumber)
        if msg_list == None:
            msg_list = []
            msg_list.append(msg)
            frameNumber_to_list.update({msg.frameNumber: msg_list})
        else:
            msg_list.append(msg)
    
    return frameNumber_to_list

