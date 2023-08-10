import sys  # nopep8
import os  # nopep8
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # nopep8
sys.path.append(os.path.dirname(SCRIPT_DIR))  # nopep8
from Common.Rosbag_extractor import RosbagExtractor
from Common.Ros_msg_types.vicon_data_publisher.msg._Force_plate_data import Force_plate_data
from Common.geometry_classes import Point2D, Point3D
from Common.Bland_Altman_Plot import BAP_config, BAP_set, generate_bland_altman_plot, prepare_CoP_data
from Common import CoPs_force_plate


#! Nur aussagekräftig, wenn ausschließlich Werte im rosbag, wo der Meißel auf dem Punkt sitzt.
class BAP__CoP_force_plate__CoP_rectangle_without_sma:
    def __init__(self) -> None:
        rootDir: str = os.path.abspath(f"{SCRIPT_DIR}/../..")
        self.dataDir: str = f"{rootDir}/Data/2023_07_26/"
        self.bagPath: str = f"{self.dataDir}2023-07-26-16-16-28_Rechteck.bag"
        self.plotSaveDir: str = f"{rootDir}/Plots/"

        self.topic_fp: str = "/Force_plate_data"
        self.topics: set[str] = set([self.topic_fp])

        self.re: RosbagExtractor = RosbagExtractor.fromBag(bagPath=self.bagPath, topics=self.topics)
        return

    def execute(self):
        sets_x: list[BAP_set] = list()
        sets_y: list[BAP_set] = list()
        for bagPath in self.re.bagPaths:
            frameNumber_to_CoP_force_plate_corner, frameNumber_to_rectangle = self.__process_bag(bagPath)
            x, y = prepare_CoP_data(frameNumber_to_CoP_force_plate_corner, frameNumber_to_rectangle)
            sets_x.append(x)
            sets_y.append(y)

        colors = list()
        colors.extend(["r"])

        while True:
            dataName1 = "CoP Kraftmessplatte"
            dataName2 = "CoP Rechteck ohne sma"
            units = "[mm]"
            config_x: BAP_config = BAP_config(sets=sets_x, colors=iter(colors), dataName1=dataName1, dataName2=dataName2, units=units, additionalComment="(x-Achse)", plotSaveDir=self.plotSaveDir)
            config_y: BAP_config = BAP_config(sets=sets_y, colors=iter(colors), dataName1=dataName1, dataName2=dataName2, units=units, additionalComment="(y-Achse)", plotSaveDir=self.plotSaveDir)

            # Plotten
            generate_bland_altman_plot(config=config_x, showplot=True)
            generate_bland_altman_plot(config=config_y, showplot=False)

        return

    def __process_bag(self, bagPath: str):
        # 1. CoPs der Kraftmessplatte berechnen
        frameNumbers_to_forcePlateData: dict[int, list[Force_plate_data]] = self.re.getFrameNumberToRosMsgs(bagPath=bagPath, topic=self.topic_fp)
        forcePlateData_mean: list[Force_plate_data] = CoPs_force_plate.forcePlataData_mean_subsampleLists(frameNumbers_to_forcePlateData)
        frameNumber_to_CoP_force_plate_corner: dict[int, Point2D] = CoPs_force_plate.calculate_CoPs(forcePlateData_mean)

        # 2. CoPs des rechtecks
        CoP_rectangle: Point3D = Point3D(x_m=509.9782/1000, y_m=202.2414/1000, z_m=(69.7475-7-2.02)/1000)
        frameNumber_to_rectangle: dict[str, Point3D] = {frameNumber: CoP_rectangle for frameNumber in frameNumber_to_CoP_force_plate_corner.keys()}

        # 3. Validieren
        # Valid_msgs_filter.removeFramesNotOcurringEverywhere([frameNumber_to_CoP_force_plate_corner, frameNumber_to_rectangle])

        return frameNumber_to_CoP_force_plate_corner, frameNumber_to_rectangle


if __name__ == "__main__":
    BAP__CoP_force_plate__CoP_rectangle_without_sma().execute()
