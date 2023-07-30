from Common.Ros_msg_types.vicon_data_publisher.msg._Force_plate_data import Force_plate_data
from Common import CoP_force_plate
from Common.geometry_classes import Point2D
import statistics


def calculate_CoPs(frameNumbers_to_forcePlateData: "dict[int, list[Force_plate_data]]") -> "dict[int, Point2D] ":
    forcePlateData_mean: list[Force_plate_data] = __forcePlataData_mean_subsampleLists(frameNumbers_to_forcePlateData)

    frameNumber_to_coP_force_plate_corner: dict[int, Point2D] = dict()
    for forcePlataData in forcePlateData_mean:
        coP_middle: Point2D = CoP_force_plate.get_CoP_force_plate_middle(forcePlataData)
        coP_corner: Point2D = CoP_force_plate.get_CoP_middle_to_corner(coP_middle)
        frameNumber_to_coP_force_plate_corner[forcePlataData.frameNumber] = coP_corner
    
    return frameNumber_to_coP_force_plate_corner


def __forcePlataData_mean_subsampleLists(frameNumbers_to_forcePlateData: "dict[int, list[Force_plate_data]]") -> "list[Force_plate_data]":
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

