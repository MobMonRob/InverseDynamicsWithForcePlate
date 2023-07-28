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
from Common.geometry_classes import Point2D


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
        1.1 Aus Marker_global_translation die Datensätze zu den Markern mit den Nummern 4,5,6,7 holen, mergen (fold_marker_data.py)
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

    frameNumbers_to_forcePlateData: dict[int, list[Force_plate_data]] = compound_topics_to_frameNumbers_to_msgs[topic_fp]
    forcePlateData_mean: list[Force_plate_data] = forcePlataData_mean_subsampleLists(frameNumbers_to_forcePlateData)

    frameNumber_to_coP_force_plate_corner: dict[int, Point2D] = dict()
    for forcePlataData in forcePlateData_mean:
        coP_middle: Point2D = ForcePlate_CoP.get_CoP_force_plate_middle(forcePlataData)
        coP_corner: Point2D = ForcePlate_CoP.get_CoP_middle_to_corner(coP_middle)
        frameNumber_to_coP_force_plate_corner[forcePlataData.frameNumber] = coP_corner

    return


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

