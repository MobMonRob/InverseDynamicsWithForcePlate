from pathlib import Path
from typing import Any
from varname import nameof
import pandas as pd
from Common.Ros_msg_types.vicon_data_publisher.msg._Marker_global_translation import Marker_global_translation


def removeIncompleteFrameNumberGroups(frameNumbers_to_msgs: "dict[int, list]"):
    max_length: int = max([len(list_) for list_ in frameNumbers_to_msgs.values()])
    incompleteFrameNumbers: list[int] = []
    for frameNumber, msgs in frameNumbers_to_msgs.items():
        if len(msgs) != max_length:
            incompleteFrameNumbers.append(frameNumber)
            print(f"{Path(__file__).stem}: Remove incomplete frameNumber {frameNumber}: len(msg)<{len(msgs)}> != max_length<{max_length}>")
    for frameNumber in incompleteFrameNumbers:
        frameNumbers_to_msgs.pop(frameNumber)


def removeFramesNotOcurringEverywhere(framesDictList: "list[dict[int, Any]]", hint: str = ""):
    """
    The last call to this function should be as late as possible.
    """
    frameNotOcurringEverywhere: set[int] = calculateFramesNotOccuringEverywhere(framesDictList=framesDictList, hint=hint)

    for frameDict in framesDictList:
        for frameNumber in frameNotOcurringEverywhere:
            frameDict.pop(frameNumber, None)

    return


def calculateFramesNotOccuringEverywhere(framesDictList: "list[dict[int, Any]]", hint: str) -> "set[int]":
    frameSets: list[set[int]] = [set(frameDict.keys()) for frameDict in framesDictList]

    # symmetric difference is not idempotent => set.symmetric_difference(frameSets) is wrong.

    union: set[int] = set.union(*frameSets)
    intersection: set[int] = set.intersection(*frameSets)
    frameNotOcurringEverywhere: set[int] = union.difference(intersection)

    if len(hint) > 0:
        hint = f"(hint: {hint}) "
    print(f"{Path(__file__).stem}: frameNotOcurringEverywhere {hint}(size: {len(frameNotOcurringEverywhere)}) = {frameNotOcurringEverywhere}")

    return frameNotOcurringEverywhere


def removeInvalidMarkerFrames(df: pd.DataFrame) -> pd.DataFrame:
    frameNumberColumn: str = f"{nameof(Marker_global_translation.frameNumber)}"
    occludedNumberColumn: str = f"{nameof(Marker_global_translation.occluded)}"
    validGroupLength: int

    # Calculate valid group length
    # Assuming, first group ist correct
    frameNumberGroup = df.groupby(frameNumberColumn)
    for name_of_group, contents_of_group in frameNumberGroup:
        validGroupLength = len(contents_of_group.index)
        break

    # Filter occluded == False
    filtered = df[df[occludedNumberColumn] == False]

    frameNumberGroup = filtered.groupby(frameNumberColumn)
    validGroupLengthFiltered = filtered[frameNumberGroup[frameNumberColumn].transform("size") == validGroupLength]

    return validGroupLengthFiltered
