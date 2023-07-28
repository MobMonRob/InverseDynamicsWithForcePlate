from collections import defaultdict


def removeInvalidMsgs(topics_to_frameNumbers_to_msgs: "dict[str, dict[int, list]]"):
    # Seqence matters here!
    __removeIncompleteFrameNumberGroups(topics_to_frameNumbers_to_msgs)
    __removeMsgsWithIndividualFrameNumbers(topics_to_frameNumbers_to_msgs)


def __removeIncompleteFrameNumberGroups(topics_to_frameNumbers_to_msgs: "dict[str, dict[int, list]]"):
    for topic, frameNumbers_to_msgs in topics_to_frameNumbers_to_msgs.items():
        first_length: int = len(next(iter(frameNumbers_to_msgs.values())))
        for frameNumber, msgs in frameNumbers_to_msgs.items():
            if len(msgs) != first_length:
                frameNumbers_to_msgs.pop(frameNumber)
                print(f"FrameNumber_filter: Removed frameNumber {frameNumber} from topic \"{topic}\"")


def __removeMsgsWithIndividualFrameNumbers(topics_to_frameNumbers_to_msgs: "dict[str, dict[int, list]]"):
    sets: list[set[int]] = []
    for frameNumbers_to_msgs in topics_to_frameNumbers_to_msgs.values():
        sets.append(set(frameNumbers_to_msgs.keys()))

    symmetric_difference: set[int] = set.symmetric_difference(*sets)
    print(f"FrameNumber_filter: symmetric_difference = {symmetric_difference}")

    for frameNumbers_to_msgs in topics_to_frameNumbers_to_msgs.values():
        for frameNumber in symmetric_difference:
            frameNumbers_to_msgs.pop(frameNumber, None)


def groupOnFrameNumber(topics_to_msgs: "dict[str, list]") -> "dict[str, dict[int, list]]":
    topics_to_frameNumbers_to_msgs: dict[str, dict[int, list]] = dict()
    for topic, msgs in topics_to_msgs.items():
        grouped_msgs: dict[int, list] = __groupOnFrameNumber(msgs)
        topics_to_frameNumbers_to_msgs.update({topic: grouped_msgs})

    return topics_to_frameNumbers_to_msgs


def __groupOnFrameNumber(msgs: "list") -> "dict[int, list]":
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
