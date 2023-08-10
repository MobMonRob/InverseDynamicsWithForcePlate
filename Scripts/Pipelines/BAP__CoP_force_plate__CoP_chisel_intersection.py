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
from Common.Bland_Altman_Plot_2 import BAP_config, BAP_set, generate_bland_altman_plot, prepare_CoP_data
from Common import CoPs_force_plate
from Common import Utils


#! Nur aussagekräftig, wenn ausschließlich Werte im rosbag, wo der Meißel auf dem Punkt sitzt.
class BAP__CoP_force_plate__CoP_chisel_intersection:
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
            frameNumber_to_CoP_force_plate_corner, frameNumber_to_intersections = self.__process_bag(bagPath)
            x, y = prepare_CoP_data(frameNumber_to_CoP_force_plate_corner, frameNumber_to_intersections)
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

        while True:
            dataName1 = "CoP Kraftmessplatte"
            dataName2 = "CoP Überschneidung"
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

        # 2. CoPs der Überschneidung berechnen
        frameNumber_to_markerGlobalTranslation: dict[int, list[Marker_global_translation]] = self.re.getFrameNumberToRosMsgs(bagPath=bagPath, topic=self.topic_mgt)
        msgs_mgt: list[Marker_global_translation] = Utils.mergeFrameNumberToList(frameNumbers_to_list=frameNumber_to_markerGlobalTranslation)
        validMarkers: pd.DataFrame = Valid_msgs_filter.removeInvalidMarkerFrames(Utils.listToDataFrame(msgs_mgt))
        frameNumber_to_intersections: dict[str, Point3D] = BAP__CoP_force_plate__CoP_chisel_intersection.calculate_intersections_CoPs(validMarkers)

        # 3. Validieren
        Valid_msgs_filter.removeFramesNotOcurringEverywhere([frameNumber_to_CoP_force_plate_corner, frameNumber_to_intersections])

        return frameNumber_to_CoP_force_plate_corner, frameNumber_to_intersections

    @staticmethod
    def calculate_intersections_CoPs(validMarkers: pd.DataFrame) -> "dict[str, Point3D]":
        markerPair1: pd.DataFrame = BAP__CoP_force_plate__CoP_chisel_intersection.averageMarkers(validMarkers, tuple([4, 5]))
        markerPair2: pd.DataFrame = BAP__CoP_force_plate__CoP_chisel_intersection.averageMarkers(validMarkers, tuple([6, 7]))
        frameNumber_to_middlePoint1: dict[int, Point3D] = BAP__CoP_force_plate__CoP_chisel_intersection.dataFrame__to__frameNumbers_to_point3D(markerPair1)
        frameNumber_to_middlePoint2: dict[int, Point3D] = BAP__CoP_force_plate__CoP_chisel_intersection.dataFrame__to__frameNumbers_to_point3D(markerPair2)
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

    @staticmethod
    def dataFrame__to__frameNumbers_to_point3D(df: pd.DataFrame) -> "dict[int, Point3D]":
        return {frameNumber: Point3D(**kwargs) for frameNumber, kwargs in df.to_dict(orient="index").items()}

    @staticmethod
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
    BAP__CoP_force_plate__CoP_chisel_intersection().execute()
