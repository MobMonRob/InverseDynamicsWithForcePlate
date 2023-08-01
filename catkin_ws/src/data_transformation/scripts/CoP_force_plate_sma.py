#!/usr/bin/env python

import rospy
from Common.geometry_classes import Point3D, Point2D
from Common import CoP_force_plate
from data_transformation.msg import CoP_position
from vicon_data_publisher.msg import Force_plate_data
from pathlib import Path


publisher = None

#############################################


def callback(data: Force_plate_data):
    middle = CoP_force_plate.get_CoP_force_plate_middle(data)
    corner = CoP_force_plate.get_CoP_middle_to_corner(middle)

    msg: CoP_position = CoP_position()
    msg.frameNumber = data.frameNumber
    msg.x_m = corner.x
    msg.y_m = corner.y
    msg.z_m = 0
    if (abs(corner.x) > 1):
        msg.x_m = -1
    if (abs(corner.y) > 1):
        msg.y_m = -1

    publisher.publish(msg)


#############################################
def transceiver():
    rospy.init_node(f"{Path(__file__).stem}", anonymous=True)

    global publisher
    publisher = rospy.Publisher(f"{Path(__file__).stem}", CoP_position, queue_size=1000)

    rospy.Subscriber("Force_plate_data_sma", Force_plate_data, callback)

    rospy.loginfo(f"{Path(__file__).stem}: started.")

    rospy.spin()


#############################################
if __name__ == '__main__':
    transceiver()
