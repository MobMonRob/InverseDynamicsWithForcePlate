import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common.Rosbag_extractor import RosbagExtractor
from Common.Ros_msg_types.vicon_data_publisher.msg._Force_plate_data import Force_plate_data
from Common.Ros_msg_types.vicon_data_publisher.msg._Marker_global_translation import Marker_global_translation
from Common import Valid_msgs_filter
from Common.geometry_classes import Point2D, Point3D, Line3D, Plane3D
import pandas as pd
from varname import nameof
from Common.Line_plane_intersection import LinePlaneIntersection
from Common import Bland_Altman_Plot
from Common import CoPs_force_plate
from Common import Utils


def execute():
    dirPath: str = "/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/Data/20Punkte_24.07.2023_1/"
    bagPath: str = f"{dirPath}2023-07-24-16-37-12.bag"
    plotSaveDir: str = "/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/Scripts/Pipelines/Plots/"

    topic_fp: str = "/Force_plate_data_sma"
    topic_mgt: str = "/Marker_global_translation"
    topics: set[str] = set([topic_fp, topic_mgt])

    re: RosbagExtractor = RosbagExtractor.fromBag(bagPath=bagPath, topics=topics)
    # re: RosbagExtractor = RosbagExtractor.fromDir(dirPath=dirPath, topics=topics)

    # 1. CoPs der Kraftmessplatte berechnen
    frameNumbers_to_forcePlateData: dict[int, list[Force_plate_data]] = re.getframeNumberToMsgs(topic_fp)
    frameNumber_to_coP_force_plate_corner: dict[int, Point2D] = CoPs_force_plate.calculate_CoPs(frameNumbers_to_forcePlateData)
    
    # 2. CoPs der Überschneidung berechnen
    frameNumber_to_markerGlobalTranslation: dict[int, list[Marker_global_translation]] = re.getframeNumberToMsgs(topic_mgt)
    msgs_mgt: list[Marker_global_translation] = Utils.mergeFrameNumberToList(frameNumbers_to_list=frameNumber_to_markerGlobalTranslation)
    validMarkers: pd.DataFrame = Valid_msgs_filter.removeInvalidMarkerFrames(Utils.listToDataFrame(msgs_mgt))
    frameNumber_to_intersections: dict[str, Point3D] = calculate_intersections_CoPs(validMarkers)
    
    # 3. Validieren
    Valid_msgs_filter.removeFramesNotOcurringEverywhere([frameNumber_to_coP_force_plate_corner, frameNumber_to_intersections])

    # 4. Plotten
    plot_x_and_y(frameNumber_to_coP_force_plate_corner=frameNumber_to_coP_force_plate_corner, frameNumber_to_intersections=frameNumber_to_intersections, plotSaveDir=plotSaveDir)

    return


def plot_x_and_y(frameNumber_to_coP_force_plate_corner: "dict[int, Point2D]", frameNumber_to_intersections: "dict[str, Point3D]", plotSaveDir: str):
    # plot x
    CoP_force_plate_list: list[float] = []
    CoP_intersection_list: list[float] = []
    for frameNumber in frameNumber_to_intersections.keys():
        CoP_force_plate: Point2D = frameNumber_to_coP_force_plate_corner.get(frameNumber)
        CoP_intersection: Point3D = frameNumber_to_intersections.get(frameNumber)
        CoP_force_plate_list.append(CoP_force_plate.x)
        CoP_intersection_list.append(CoP_intersection.x_m)
    
    CoP_force_plate_list = Bland_Altman_Plot.scale(CoP_force_plate_list, 1000)
    CoP_intersection_list = Bland_Altman_Plot.scale(CoP_intersection_list, 1000)

    Bland_Altman_Plot.generate_bland_altman_plot(data1=CoP_force_plate_list, data2=CoP_intersection_list, dataName1="CoP Kraftmessplatte", dataName2="CoP Überschneidung", units="[mm]", saveDir=plotSaveDir, additionalComment="(x-Achse)")

    # plot y
    CoP_force_plate_list: list[float] = []
    CoP_intersection_list: list[float] = []
    for frameNumber in frameNumber_to_intersections.keys():
        CoP_force_plate: Point2D = frameNumber_to_coP_force_plate_corner.get(frameNumber)
        CoP_intersection: Point3D = frameNumber_to_intersections.get(frameNumber)
        CoP_force_plate_list.append(CoP_force_plate.y)
        CoP_intersection_list.append(CoP_intersection.y_m)

    CoP_force_plate_list = Bland_Altman_Plot.scale(CoP_force_plate_list, 1000)
    CoP_intersection_list = Bland_Altman_Plot.scale(CoP_intersection_list, 1000)

    Bland_Altman_Plot.generate_bland_altman_plot(data1=CoP_force_plate_list, data2=CoP_intersection_list, dataName1="CoP Kraftmessplatte", dataName2="CoP Überschneidung", units="[mm]", saveDir=plotSaveDir, additionalComment="(y-Achse)")

    return


def calculate_intersections_CoPs(validMarkers: pd.DataFrame) -> "dict[str, Point3D]":
    markerPair1: pd.DataFrame = averageMarkers(validMarkers, tuple([4, 5]))
    markerPair2: pd.DataFrame = averageMarkers(validMarkers, tuple([6, 7]))
    frameNumber_to_middlePoint1: dict[int, Point3D] = Utils.dataFrame__to__frameNumbers_to_point3D(markerPair1)
    frameNumber_to_middlePoint2: dict[int, Point3D] = Utils.dataFrame__to__frameNumbers_to_point3D(markerPair2)
    # valid assumption: firstMarkerDict and secondMarkerDict contain exactly the same keys
    frameNumbers_to_lines: dict[int, Line3D] = dict()
    for frameNumber in frameNumber_to_middlePoint1.keys():
        middlePoint1: Point3D = frameNumber_to_middlePoint1.get(frameNumber)
        middlePoint2: Point3D = frameNumber_to_middlePoint2.get(frameNumber)
        line = Line3D(middlePoint1, middlePoint2)
        frameNumbers_to_lines[frameNumber] = line
    
    plane: Plane3D = Plane3D.vicon_main_plane()
    intersector: LinePlaneIntersection = LinePlaneIntersection()
    frameNumber_to_intersections: dict[str, Point3D] = dict()
    for frameNumber, line in frameNumbers_to_lines.items():
        intersection: Point3D = intersector.intersect(line=line, plane=plane)
        frameNumber_to_intersections[frameNumber] = intersection
    
    return frameNumber_to_intersections


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


if __name__ == "__main__":
    execute()

