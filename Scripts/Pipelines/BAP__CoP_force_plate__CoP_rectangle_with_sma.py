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
    dirPath: str = "/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/Data/2023_07_26/"
    bagPath: str = f"{dirPath}2023-07-26-16-16-28_Rechteck.bag"
    plotSaveDir: str = "/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/Scripts/Pipelines/Plots/"

    topic_fp: str = "/Force_plate_data_sma"
    topics: set[str] = set([topic_fp])

    re: RosbagExtractor = RosbagExtractor.fromBag(bagPath=bagPath, topics=topics)
    # re: RosbagExtractor = RosbagExtractor.fromDir(dirPath=dirPath, topics=topics)

    # 1. CoPs der Kraftmessplatte berechnen
    frameNumbers_to_forcePlateData: dict[int, list[Force_plate_data]] = re.getFrameNumberToRosMsgs(topic_fp)
    forcePlateData_mean: list[Force_plate_data] = CoPs_force_plate.forcePlataData_mean_subsampleLists(frameNumbers_to_forcePlateData)
    frameNumber_to_CoP_force_plate_corner: dict[int, Point2D] = CoPs_force_plate.calculate_CoPs(forcePlateData_mean)

    # 2. CoPs des rechtecks
    CoP_rectangle: Point3D = Point3D(x_m=509.9782/1000, y_m=202.2414/1000, z_m=(69.7475-7-2.02)/1000)
    frameNumber_to_rectangle: dict[str, Point3D] = {frameNumber: CoP_rectangle for frameNumber in frameNumber_to_CoP_force_plate_corner.keys()}

    # 3. Validieren
    # Valid_msgs_filter.removeFramesNotOcurringEverywhere([frameNumber_to_CoP_force_plate_corner, frameNumber_to_marker_tip])

    # 4. Plotten
    Bland_Altman_Plot.plot_x_and_y(frameNumber_to_CoP_force_plate_corner=frameNumber_to_CoP_force_plate_corner,
                                   frameNumber_to_CoP_marker=frameNumber_to_rectangle, marker_CoP_name="CoP Rechteck mit sma", plotSaveDir=plotSaveDir)

    return


if __name__ == "__main__":
    execute()
