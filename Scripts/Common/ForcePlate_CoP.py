from Common.geometry_classes import Point2D
from Common.Ros_msg_types.vicon_data_publisher.msg._Force_plate_data import Force_plate_data

# * All parameter units are in the International System of Units (SI)!

class AMTI_BMS400600():
    # Offsets for AMTI BMS400600
    a = 0
    b = 0
    z0 = 0

    # Force plate width
    width = 0.4

    #Force plate hight
    length = 0.6


def get_CoP_force_plate_middle(fp: Force_plate_data) -> Point2D:
    # Height of metal plate on the force plate
    h = 0.04478

    CoP_x = ((-fp.my_Nm + fp.fx_N * (AMTI_BMS400600.z0 + h)) / fp.fz_N) - AMTI_BMS400600.a
    CoP_y = ((fp.mx_Nm + fp.fy_N * (AMTI_BMS400600.z0 + h)) / fp.fz_N) - AMTI_BMS400600.b

    return Point2D(CoP_x, CoP_y)


def get_CoP_middle_to_corner(CoP_middle: Point2D) -> Point2D:
    # For fixed rotational transformation matrix from local force plate frame to global frame in the plate corner:
    #     (0 1  0)
    # R = (1 0  0)
    #     (0 0 -1)

    CoPx_g = AMTI_BMS400600.length / 2 + CoP_middle.y
    CoPy_g = AMTI_BMS400600.width / 2 + CoP_middle.x

    return Point2D(CoPx_g, CoPy_g)


def get_torsional_moment_force_plate_middle(fp: Force_plate_data) -> float:
    CoP: Point2D = get_CoP_force_plate_middle(fp)

    # Torsional (or frictional) moment in [Newtons*Meters]
    # This moment is assumed to have no components along the X and Y plate axes
    Tz = fp.mz_Nm + fp.fx_N * (CoP.x + AMTI_BMS400600.b) - fp.fy_N * (CoP.y + AMTI_BMS400600.a)
    return Tz


def get_torsional_moment_force_plate_corner(Tz: float) -> float:
    return -Tz
