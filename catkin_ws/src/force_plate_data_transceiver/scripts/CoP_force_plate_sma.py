#!/usr/bin/env python

import sys  # nopep8
import os  # nopep8
sys.path.append(
    "/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/catkin_ws/src/force_plate_data_transceiver/scripts/")  # nopep8

from geometry.geometry_classes import Point3D
from geometry.calculate_cop import get_cop_as_arguments
import rospy
from force_plate_data_transceiver.msg import CoP_position
from vicon_data_publisher.msg import Force_plate_data


publisher = None

#############################################


def callback(data: Force_plate_data):
    point3d: Point3D = get_cop_as_arguments(
        data.fx_N, data.fy_N, data.fz_N, data.mx_Nm, data.my_Nm, data.mz_Nm)
    msg: CoP_position = CoP_position()
    msg.frameNumber = data.frameNumber
    msg.x_m = point3d.x
    msg.y_m = point3d.y
    msg.z_m = point3d.z
    if (abs(point3d.x) > 1):
        msg.x_m = -1
    if (abs(point3d.y) > 1):
        msg.y_m = -1
    if (abs(point3d.z) > 1):
        msg.z_m = -1
    publisher.publish(msg)


#############################################
def transceiver():
    name = 'CoP_force_plate_sma'
    rospy.init_node(name, anonymous=True)

    global publisher
    publisher = rospy.Publisher(name, CoP_position, queue_size=1000)

    rospy.Subscriber("Force_plate_data_sma", Force_plate_data, callback)

    rospy.loginfo(f"{name} started.")

    rospy.spin()


#############################################
if __name__ == '__main__':
    transceiver()
