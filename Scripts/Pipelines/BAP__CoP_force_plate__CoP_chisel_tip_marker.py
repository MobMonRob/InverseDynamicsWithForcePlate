import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common.Rosbag_extractor import RosbagExtractor
from Common.Ros_msg_types.vicon_data_publisher.msg._Force_plate_data import Force_plate_data
from Common.Ros_msg_types.vicon_data_publisher.msg._Marker_global_translation import Marker_global_translation
from Common import Valid_msgs_filter
from Common.geometry_classes import Point2D, Point3D
import pandas as pd
from varname import nameof
from Common import Bland_Altman_Plot
from Common import CoPs_force_plate
from Common import Utils


#! Nur aussagekräftig, wenn ausschließlich Werte im rosbag, wo der Meißel auf dem Punkt sitzt.
def execute():
    dirPath: str = "/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/Data/20Punkte_24.07.2023_1/"
    bagPath: str = f"{dirPath}2023-07-24-16-37-12.bag"
    plotSaveDir: str = "/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/Scripts/Pipelines/Plots/"

    topic_fp: str = "/Force_plate_data_sma"
    topic_mgt: str = "/Marker_global_translation"
    topics: set[str] = set([topic_fp, topic_mgt])

    # re: RosbagExtractor = RosbagExtractor.fromBag(bagPath=bagPath, topics=topics)
    re: RosbagExtractor = RosbagExtractor.fromDir(dirPath=dirPath, topics=topics)

    # 1. CoPs der Kraftmessplatte berechnen
    frameNumbers_to_forcePlateData: dict[int, list[Force_plate_data]] = re.getframeNumberToMsgs(topic_fp)
    forcePlateData_mean: list[Force_plate_data] = CoPs_force_plate.forcePlataData_mean_subsampleLists(frameNumbers_to_forcePlateData)
    frameNumber_to_CoP_force_plate_corner: dict[int, Point2D] = CoPs_force_plate.calculate_CoPs(forcePlateData_mean)
    
    # 2. CoPs des marker tips
    frameNumber_to_markerGlobalTranslation: dict[int, list[Marker_global_translation]] = re.getframeNumberToMsgs(topic_mgt)
    msgs_mgt: list[Marker_global_translation] = Utils.mergeFrameNumberToList(frameNumbers_to_list=frameNumber_to_markerGlobalTranslation)
    validMarkers: pd.DataFrame = Valid_msgs_filter.removeInvalidMarkerFrames(Utils.listToDataFrame(msgs_mgt))
    frameNumber_to_marker_tip: dict[str, Point3D] = calculateMarkerTips(validMarkers)
    
    # 3. Validieren
    Valid_msgs_filter.removeFramesNotOcurringEverywhere([frameNumber_to_CoP_force_plate_corner, frameNumber_to_marker_tip])

    # 4. Plotten
    Bland_Altman_Plot.plot_x_and_y(frameNumber_to_CoP_force_plate_corner=frameNumber_to_CoP_force_plate_corner, frameNumber_to_CoP_marker=frameNumber_to_marker_tip, marker_CoP_name="CoP Marker an der Spitze", plotSaveDir=plotSaveDir)

    return


def calculateMarkerTips(validMarkers: pd.DataFrame) -> "dict[str, Point3D]":
    frameNumberColumn: str = f"{nameof(Marker_global_translation.frameNumber)}"
    markerNumberColumn: str = f"{nameof(Marker_global_translation.markerNumber)}"
    occludedNumberColumn: str = f"{nameof(Marker_global_translation.occluded)}"

    # Filter markers of interest
    df = validMarkers[validMarkers[markerNumberColumn].isin([8])]
    
    # Drop unneeded columns
    df = df.drop(columns=[markerNumberColumn, occludedNumberColumn])

    frameNumber_to_tip: dict[int, Point3D] = {row.frameNumber : Point3D(x_m=row.x_m, y_m=row.y_m, z_m=row.z_m) for index, row in df.iterrows()}

    return frameNumber_to_tip


if __name__ == "__main__":
    execute()

