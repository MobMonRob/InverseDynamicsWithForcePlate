import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common import Rosbag_extractor
from Common.Ros_msg_types.vicon_data_publisher.msg._Force_plate_data import Force_plate_data
from Common.Ros_msg_types.vicon_data_publisher.msg._Marker_global_translation import Marker_global_translation
from Common import Valid_msgs_filter
import statistics
from Common.Simple_moving_average import SimpleMovingAverageOnObjects
from Common import ForcePlate_CoP
from Common.geometry_classes import Point2D, Point3D, Line3D
import pandas as pd
from typing import TypeVar, Generic
from varname import nameof


def execute():
    pass
    # TODO: Vorgehen: Dateien neu erstellen und Code kopieren. Nicht alte Dateien kopieren.
    # TODO: Common scripts in Unterordner von force_plate_data_transceiver verlinken.
    """
    // 1. CoP der Kraftmessplatte einlesen
        // 1.1 Force_plate_data aus robag holen
        // // 1.1.2 Force_plate_sma berechnen
        // 1.2 Alle Subsamples mitteln
        // 1.4 CoP_force_plate_sma berechnen und zurückgeben
    2. CoP der Überschneidung einlesen
        // 1.1 Aus Marker_global_translation die Datensätze zu den Markern mit den Nummern 4,5,6,7 holen, Mittelwerte berechnen (fold_marker_data.py)
        1.2 Für jeden Frame find_line_plane_intersection.py ausführen auf den Mittelpunkten der Marker 4,5,6,7. Die Gleichung der Ebene ist in
            find_line_plane_intersection.py hardgecodet. Ausgabe: Punkte mit x, y, z (Überscheidung der Gerade mit der Ebene)
    3. BAD machen CoP force plate VS. CoP Überschneidung
    """

    dirPath: str = "/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/Data/20Punkte_24.07.2023_1/"
    bagPath: str = f"{dirPath}2023-07-24-16-37-12.bag"
    topic_fp: str = "/Force_plate_data_sma"
    topic_mgt: str = "/Marker_global_translation"
    topics: set[str] = set([topic_fp, topic_mgt])

    Rosbag_extractor.printInfo(bagPath)

    compound_topics_to_msgs: dict[str, list] = Rosbag_extractor.getTopicsToMsgsFromBag(bagPath, topics)
    # compound_topics_to_msgs: dict[str, list] = Rosbag_extractor.getTopicsToMsgsFromDir(dirPath, topics)
    compound_topics_to_frameNumbers_to_msgs: dict[str, dict[int, list]] = Valid_msgs_filter.groupMsgsOnFrameNumber_topic(compound_topics_to_msgs)
    Valid_msgs_filter.removeInvalidMsgs(compound_topics_to_frameNumbers_to_msgs)

    # 1. CoP der Kraftmessplatte einlesen
    # frameNumbers_to_forcePlateData: dict[int, list[Force_plate_data]] = compound_topics_to_frameNumbers_to_msgs[topic_fp]
    # forcePlateData_mean: list[Force_plate_data] = forcePlataData_mean_subsampleLists(frameNumbers_to_forcePlateData)

    # frameNumber_to_coP_force_plate_corner: dict[int, Point2D] = dict()
    # for forcePlataData in forcePlateData_mean:
    #     coP_middle: Point2D = ForcePlate_CoP.get_CoP_force_plate_middle(forcePlataData)
    #     coP_corner: Point2D = ForcePlate_CoP.get_CoP_middle_to_corner(coP_middle)
    #     frameNumber_to_coP_force_plate_corner[forcePlataData.frameNumber] = coP_corner
    
    # 2. CoP der Überschneidung einlesen
    frameNumbers_to_markerGlobalTranslation: dict[int, list[Marker_global_translation]] = compound_topics_to_frameNumbers_to_msgs[topic_mgt]
    msgs_mgt: list[Marker_global_translation] = []
    for msgs in frameNumbers_to_markerGlobalTranslation.values():
        msgs_mgt.extend(msgs)
    
    validMarkers: pd.DataFrame = removeInvalidFrames(listToDataFrame(msgs_mgt))
    firstMarkerPair: pd.DataFrame = averageMarkers(validMarkers, tuple([4, 5]))
    secondMarkerPair: pd.DataFrame = averageMarkers(validMarkers, tuple([6, 7]))
    firstMarkerDict = dataFrame__to__frameNumbers_to_point3D(firstMarkerPair)
    secondMarkerDict = dataFrame__to__frameNumbers_to_point3D(secondMarkerPair)
    # valid assumption: firstMarkerDict and secondMarkerDict contain same keys
    frameNumbers_to_lines: dict[int, Line3D] = dict()
    for frameNumber in firstMarkerDict.keys():
        firstPoint: Point3D = firstMarkerDict.get(frameNumber)
        secondPoint: Point3D = secondMarkerDict.get(frameNumber)
        line = Line3D(firstPoint, secondPoint)
        frameNumbers_to_lines[frameNumber] = line
    
    for line in frameNumbers_to_lines.values():
        print(line.xTermParametrized)
        break


    return


def dataFrame__to__frameNumbers_to_point3D(df: pd.DataFrame) -> "dict[int, Point3D]":
    return{frameNumber: Point3D(**kwargs) for frameNumber, kwargs in df.to_dict(orient="index").items()}


def removeInvalidFrames(df: pd.DataFrame) -> pd.DataFrame:
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

def averageMarkers(df: pd.DataFrame, markerNumbers: "tuple[int]") -> pd.DataFrame:
    frameNumberColumn: str = f"{nameof(Marker_global_translation.frameNumber)}"
    markerNumberColumn: str = f"{nameof(Marker_global_translation.markerNumber)}"
    occludedNumberColumn: str = f"{nameof(Marker_global_translation.occluded)}"

    # Filter markers of interest
    filtered = df[df[markerNumberColumn].isin(markerNumbers)]
    
    # Drop unneeded columns
    toAverage = filtered.drop(columns=[markerNumberColumn, occludedNumberColumn])

    # Calculate averages per frameNumber
    frameNumberGroup = toAverage.groupby(frameNumberColumn)
    averages = frameNumberGroup.mean()

    return averages

T = TypeVar('T')
def listToDataFrame(theList: "list[T]") -> pd.DataFrame:
    fieldNames: list[str] = theList[0].__slots__
    values = [[getattr(element, fieldName) for fieldName in fieldNames] for element in theList]
    df: pd.DataFrame = pd.DataFrame(values, columns=fieldNames)
    return df

def forcePlataData_mean_subsampleLists(frameNumbers_to_forcePlateData: "dict[int, list[Force_plate_data]]") -> "list[Force_plate_data]":
    forcePlateData_mean: list[Force_plate_data] = []

    for frameNumber, subsampleList in frameNumbers_to_forcePlateData.items():
        mean: Force_plate_data = Force_plate_data()
        for fieldName in Force_plate_data.__slots__:
            field_values_list: list[float] = [getattr(forcePlateData, fieldName) for forcePlateData in subsampleList]
            field_mean: float = statistics.fmean(field_values_list)
            setattr(mean, fieldName, field_mean)
        mean.frameNumber = frameNumber
        mean.subsampleNumber = None
        forcePlateData_mean.append(mean)
    
    return forcePlateData_mean


def sma(frameNumbers_to_forcePlateData_mean: "dict[int, Force_plate_data]") -> "dict[int, Force_plate_data]":
    sma: SimpleMovingAverageOnObjects[Force_plate_data] = SimpleMovingAverageOnObjects[Force_plate_data](1000, Force_plate_data())
    
    frameNumbers_to_forcePlateData_sma: dict[int, Force_plate_data] = dict()
    for frameNumber, forcePlateData in frameNumbers_to_forcePlateData_mean.items():
        forcePlataData_sma: Force_plate_data = sma.process(forcePlateData)
        forcePlataData_sma.frameNumber = frameNumber
        frameNumbers_to_forcePlateData_sma[frameNumber] = forcePlataData_sma

    return sma


if __name__ == "__main__":
    execute()

