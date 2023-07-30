import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common import Rosbag_extractor
from Common.Ros_msg_types.vicon_data_publisher.msg._Force_plate_data import Force_plate_data
from Common.Ros_msg_types.vicon_data_publisher.msg._Marker_global_translation import Marker_global_translation
from Common import Valid_msgs_filter
from Common.geometry_classes import Point2D, Point3D, Line3D, Plane3D
import pandas as pd
from typing import TypeVar, Generic
from varname import nameof
from Common.Line_plane_intersection import LinePlaneIntersection
from Common import Bland_Altman_Plot
from Common import CoPs_force_plate


def execute():
    # TODO: Vorgehen: Dateien neu erstellen und Code kopieren. Nicht alte Dateien kopieren.
    # TODO: Common scripts in Unterordner von force_plate_data_transceiver verlinken.
    # TODO: Die Validierung auf den Frames besser machen. Vor jedem Topic das Topic einzeln validieren. Dann vor Verwendung von beiden, beide validieren.

    dirPath: str = "/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/Data/20Punkte_24.07.2023_1/"
    bagPath: str = f"{dirPath}2023-07-24-16-37-12.bag"
    plotSaveDir: str = "/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/Scripts/Pipelines/Plots/"

    topic_fp: str = "/Force_plate_data_sma"
    topic_mgt: str = "/Marker_global_translation"
    topics: set[str] = set([topic_fp, topic_mgt])

    Rosbag_extractor.printInfo(bagPath)

    compound_topics_to_msgs: dict[str, list] = Rosbag_extractor.getTopicsToMsgsFromBag(bagPath, topics)
    # compound_topics_to_msgs: dict[str, list] = Rosbag_extractor.getTopicsToMsgsFromDir(dirPath, topics)

    compound_topics_to_frameNumbers_to_msgs: dict[str, dict[int, list]] = Valid_msgs_filter.groupMsgsOnFrameNumber_topic(compound_topics_to_msgs)
    Valid_msgs_filter.removeInvalidMsgs(compound_topics_to_frameNumbers_to_msgs)

    # 1. CoP der Kraftmessplatte einlesen
    frameNumbers_to_forcePlateData: dict[int, list[Force_plate_data]] = compound_topics_to_frameNumbers_to_msgs[topic_fp]
    frameNumber_to_coP_force_plate_corner: dict[int, Point2D] = CoPs_force_plate.calculate_CoPs(frameNumbers_to_forcePlateData)
    
    # 2. CoP der Überschneidung einlesen
    # Beachte: Die validen frameNumbers können unterschiedlich gewesen sein für beide topics.
    frameNumbers_to_markerGlobalTranslation: dict[int, list[Marker_global_translation]] = compound_topics_to_frameNumbers_to_msgs[topic_mgt]
    msgs_mgt: list[Marker_global_translation] = []
    for msgs in frameNumbers_to_markerGlobalTranslation.values():
        msgs_mgt.extend(msgs)
    
    validMarkers: pd.DataFrame = removeInvalidMarkerFrames(listToDataFrame(msgs_mgt))
    markerPair1: pd.DataFrame = averageMarkers(validMarkers, tuple([4, 5]))
    markerPair2: pd.DataFrame = averageMarkers(validMarkers, tuple([6, 7]))
    frameNumber_to_middlePoint1: dict[int, Point3D] = dataFrame__to__frameNumbers_to_point3D(markerPair1)
    frameNumber_to_middlePoint2: dict[int, Point3D] = dataFrame__to__frameNumbers_to_point3D(markerPair2)
    # valid assumption: firstMarkerDict and secondMarkerDict contain exactly the same keys
    frameNumbers_to_lines: dict[int, Line3D] = dict()
    for frameNumber in frameNumber_to_middlePoint1.keys():
        middlePoint1: Point3D = frameNumber_to_middlePoint1.get(frameNumber)
        middlePoint2: Point3D = frameNumber_to_middlePoint2.get(frameNumber)
        line = Line3D(middlePoint1, middlePoint2)
        frameNumbers_to_lines[frameNumber] = line
    
    plane: Plane3D = Plane3D.vicon_main_plane()
    intersector: LinePlaneIntersection = LinePlaneIntersection()
    intersections: dict[str, Point3D] = dict()
    for frameNumber, line in frameNumbers_to_lines.items():
        intersection: Point3D = intersector.intersect(line=line, plane=plane)
        intersections[frameNumber] = intersection
    # TODO: Ros node für intersection machen. Dann kann man im plotjuggler sehen wie sehr die einzelnen CoPs von der Kraftmessplatte und von Vicon an einem gemessenen Punkt schwanken. Das was mehr schwankt, ist womöglich ungenauer, denn der Meissel verändert seine Position nicht. Es kann natürlich aber sein, dass man einen konstanten Versatz hat von einer Methode zu dem tatsächlichen Punkt, den man so nicht feststellen kann. In Bland-Altman Diagramm sieht man auch generell nicht, was wie sehr schwankt.
    # TODO: Die systematische Abweichung in der y-Achse für alle Rosbags könnte daran liegen, dass man an wenig y-Stellen misst. Denn bei der x-Achse ist der Mittelwert der Differenzen fast Null.
    # TODO: Ausserdem: Wenn man die Kraftmessplatte mit dem Viereck vergleicht, dann bekommt man auch eine Idee, wie genau die Kraftmessplatte eigentlich ist.
    # TODO: Normalerweise wird Bland-Altman Plot benutzt, um Methode gegen Goldstandard zu vergleichen. Hier allerdings sind beide Methoden ungenau. Und auch noch nicht klar, wie sehr ungenau. Die Kraftmessplatte ist ungenau auch, weil Gewicht recht klein (Gewicht angeben und welches die in der Dokumentation schreiben, man mindestens braucht).
    
    # plot x
    CoP_force_plate_list: list[float] = []
    CoP_intersection_list: list[float] = []
    for frameNumber in intersections.keys():
        CoP_force_plate: Point2D = frameNumber_to_coP_force_plate_corner.get(frameNumber, None)
        if CoP_force_plate == None:
            continue
        CoP_intersection: Point3D = intersections.get(frameNumber)
        CoP_force_plate_list.append(CoP_force_plate.x)
        CoP_intersection_list.append(CoP_intersection.x_m)
    
    CoP_force_plate_list = Bland_Altman_Plot.scale(CoP_force_plate_list, 1000)
    CoP_intersection_list = Bland_Altman_Plot.scale(CoP_intersection_list, 1000)

    Bland_Altman_Plot.generate_bland_altman_plot(data1=CoP_force_plate_list, data2=CoP_intersection_list, dataName1="CoP Kraftmessplatte", dataName2="CoP Überschneidung", units1="[mm]", units2="[mm]", saveDir=plotSaveDir, additionalComment="(x-Achse)")

    # plot y
    CoP_force_plate_list: list[float] = []
    CoP_intersection_list: list[float] = []
    for frameNumber in intersections.keys():
        CoP_force_plate: Point2D = frameNumber_to_coP_force_plate_corner.get(frameNumber, None)
        if CoP_force_plate == None:
            continue
        CoP_intersection: Point3D = intersections.get(frameNumber)
        CoP_force_plate_list.append(CoP_force_plate.y)
        CoP_intersection_list.append(CoP_intersection.y_m)

    CoP_force_plate_list = Bland_Altman_Plot.scale(CoP_force_plate_list, 1000)
    CoP_intersection_list = Bland_Altman_Plot.scale(CoP_intersection_list, 1000)

    Bland_Altman_Plot.generate_bland_altman_plot(data1=CoP_force_plate_list, data2=CoP_intersection_list, dataName1="CoP Kraftmessplatte", dataName2="CoP Überschneidung", units1="[mm]", units2="[mm]", saveDir=plotSaveDir, additionalComment="(y-Achse)")

    return


def dataFrame__to__frameNumbers_to_point3D(df: pd.DataFrame) -> "dict[int, Point3D]":
    return{frameNumber: Point3D(**kwargs) for frameNumber, kwargs in df.to_dict(orient="index").items()}


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


if __name__ == "__main__":
    execute()

