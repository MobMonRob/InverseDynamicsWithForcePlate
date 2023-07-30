from pathlib import Path
from typing import Any
from varname import nameof
import pandas as pd
from Common.Ros_msg_types.vicon_data_publisher.msg._Marker_global_translation import Marker_global_translation


def removeIncompleteFrameNumberGroups(frameNumbers_to_msgs: "dict[int, list]"):
    """
    Assumes, first_length is a correct one.
    """
    first_length: int = len(next(iter(frameNumbers_to_msgs.values())))
    incompleteFrameNumbers: list[int] = []
    for frameNumber, msgs in frameNumbers_to_msgs.items():
        if len(msgs) != first_length:
            incompleteFrameNumbers.append(frameNumber)
    for frameNumber in incompleteFrameNumbers:
            frameNumbers_to_msgs.pop(frameNumber)
            print(f"{Path(__file__).stem}: Removed incomplete frameNumber {frameNumber}")


def removeFramesNotOcurringEverywhere(framesDictList: "list[dict[int, Any]]"):
    """
    The last call to this function should be as late as possible.
    """
    frameNotOcurringEverywhere: set[int] = calculateFramesNotOccuringEverywhere(framesDictList=framesDictList)

    for frameDict in framesDictList:
        for frameNumber in frameNotOcurringEverywhere:
            frameDict.pop(frameNumber, None)

    return


def calculateFramesNotOccuringEverywhere(framesDictList: "list[dict[int, Any]]") -> "set[int]":
    frameSets: list[set[int]] = [set(frameDict.keys()) for frameDict in framesDictList]

    # symmetric difference is not idempotent => set.symmetric_difference(frameSets) is wrong.
    
    union: set[int] = set.union(*frameSets)
    intersection: set[int] = set.intersection(*frameSets)
    frameNotOcurringEverywhere: set[int] = union.difference(intersection)
    print(f"{Path(__file__).stem}: frameNotOcurringEverywhere (size: {len(frameNotOcurringEverywhere)}) = {frameNotOcurringEverywhere}")

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

