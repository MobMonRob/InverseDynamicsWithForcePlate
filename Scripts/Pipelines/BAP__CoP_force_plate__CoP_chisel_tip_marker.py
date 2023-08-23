import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common.Rosbag_extractor import RosbagExtractor, IndexedBagMsgs, BagMsgs
from Common.Ros_msg_types.vicon_data_publisher.msg._Force_plate_data import Force_plate_data
from Common.Ros_msg_types.vicon_data_publisher.msg._Marker_global_translation import Marker_global_translation
from Common import Valid_msgs_filter
from Common.geometry_classes import Point2D, Point3D
import pandas as pd
from varname import nameof
from Common.Bland_Altman_Plot import BAP_config, BAP_set, generate_bland_altman_plot, prepare_CoP_data
from Common import CoPs_force_plate
from Common import Utils


#! Nur aussagekräftig, wenn ausschließlich Werte im rosbag, wo der Meißel auf dem Punkt sitzt.
class BAP__CoP_force_plate__CoP_chisel_tip_marker:
    def __init__(self) -> None:
        rootDir: str = os.path.abspath(f"{SCRIPT_DIR}/../..")
        self.dataDir: str = f"{rootDir}/Data/2023_07_24 - 20 Punkte/"
        self.bagPath: str = f"{self.dataDir}2023-07-24-16-37-12.bag"
        self.plotSaveDir: str = f"{rootDir}/Plots/"

        self.topic_fp: str = "/Force_plate_data_sma"
        self.topic_mgt: str = "/Marker_global_translation"
        self.topics: set[str] = set([self.topic_fp, self.topic_mgt])

        # self.re: RosbagExtractor = RosbagExtractor.fromBag(bagPath=self.bagPath, topics=self.topics)
        self.re: RosbagExtractor = RosbagExtractor.fromDir(dirPath=self.dataDir, topics=self.topics)
        return

    def execute(self):
        sets_x: list[BAP_set] = list()
        sets_y: list[BAP_set] = list()
        for bagPath in self.re.bagPaths:
            frameNumber_to_CoP_force_plate_corner, frameNumber_to_marker_tip = self.__process_bag(bagPath)
            x, y = prepare_CoP_data(frameNumber_to_CoP_force_plate_corner, frameNumber_to_marker_tip)
            sets_x.append(x)
            sets_y.append(y)

        # colors = iter(cm.rainbow(np.linspace(0, 1, len(sets))))
        # colors = itertools.cycle(["r", "b"])
        colors = list()
        colors.extend(["r"] * 5)
        colors.extend(["b"] * 5)
        colors.extend(["r"] * 4)
        colors.extend(["b"] * 4)
        colors.extend(["g"] * 2)

        # while True:
        dataName1 = "CoP Kraftmessplatte"
        dataName2 = "CoP Marker an der Spitze"
        units = "[mm]"
        config_x: BAP_config = BAP_config(sets=sets_x, colors=iter(colors), dataName1=dataName1, dataName2=dataName2, units=units, additionalComment="(x-Achse)", plotSaveDir=self.plotSaveDir)
        config_y: BAP_config = BAP_config(sets=sets_y, colors=iter(colors), dataName1=dataName1, dataName2=dataName2, units=units, additionalComment="(y-Achse)", plotSaveDir=self.plotSaveDir)

        # Plotten
        generate_bland_altman_plot(config=config_x)
        generate_bland_altman_plot(config=config_y)

        return

    def __process_bag(self, bagPath: str):
        # 1. CoPs der Kraftmessplatte berechnen
        frameNumbers_to_forcePlateData: dict[int, list[Force_plate_data]] = self.re.getFrameNumberToRosMsgs(bagPath=bagPath, topic=self.topic_fp)
        forcePlateData_mean: list[Force_plate_data] = CoPs_force_plate.forcePlataData_mean_subsampleLists(frameNumbers_to_forcePlateData)
        frameNumber_to_CoP_force_plate_corner: dict[int, Point2D] = CoPs_force_plate.calculate_CoPs(forcePlateData_mean)

        # 2. CoPs des marker tips
        frameNumber_to_markerGlobalTranslation: dict[int, list[Marker_global_translation]] = self.re.getFrameNumberToRosMsgs(bagPath=bagPath, topic=self.topic_mgt)
        msgs_mgt: list[Marker_global_translation] = Utils.mergeFrameNumberToList(frameNumbers_to_list=frameNumber_to_markerGlobalTranslation)
        validMarkers: pd.DataFrame = Valid_msgs_filter.removeInvalidMarkerFrames(Utils.listToDataFrame(msgs_mgt))
        frameNumber_to_marker_tip: dict[int, Point3D] = BAP__CoP_force_plate__CoP_chisel_tip_marker.__calculateMarkerTips(validMarkers)

        # 3. Validieren
        Valid_msgs_filter.removeFramesNotOcurringEverywhere(framesDictList=[frameNumber_to_CoP_force_plate_corner, frameNumber_to_marker_tip], hint=bagPath)

        return frameNumber_to_CoP_force_plate_corner, frameNumber_to_marker_tip

    @staticmethod
    def __calculateMarkerTips(validMarkers: pd.DataFrame) -> "dict[int, Point3D]":
        frameNumberColumn: str = f"{nameof(Marker_global_translation.frameNumber)}"
        markerNumberColumn: str = f"{nameof(Marker_global_translation.markerNumber)}"
        occludedNumberColumn: str = f"{nameof(Marker_global_translation.occluded)}"

        # Filter markers of interest
        df = validMarkers[validMarkers[markerNumberColumn].isin([8])]

        # Drop unneeded columns
        df = df.drop(columns=[markerNumberColumn, occludedNumberColumn])

        frameNumber_to_tip: dict[int, Point3D] = {row.frameNumber: Point3D(x_m=row.x_m, y_m=row.y_m, z_m=row.z_m) for index, row in df.iterrows()}

        return frameNumber_to_tip


if __name__ == "__main__":
    BAP__CoP_force_plate__CoP_chisel_tip_marker().execute()
