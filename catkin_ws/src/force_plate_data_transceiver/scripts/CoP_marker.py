#!/usr/bin/env python

#! TODO: Refactor

import rospy
from vicon_data_publisher.msg import Marker_global_translation
from force_plate_data_transceiver.msg import CoP_position

import sys, os
sys.path.append("/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/catkin_ws/src/force_plate_data_transceiver/scripts/")
from geometry.find_line_plane_intersection import find_intersection, substitute_variables
from geometry.define_line import get_line_as_arguments
from geometry.define_plane import define_plane
from geometry.geometry_classes import Point3D
import numpy as np

from sympy.solvers import solve
from sympy import Symbol

publisher = None

marker4 = -1
marker5 = -1
marker6 = -1
marker7 = -1

plane = define_plane(np.array([-0.019 / 1000, 0.124 / 1000, -0.002 / 1000]), np.array([0.6, -0.037 / 1000, -0.037 / 1000]), np.array([-0.102 / 1000, 0.434847, 0.1321 / 1000]))

#############################################

def callback(data: Marker_global_translation):
    # if (data.frameNumber % 10 > 0):
    #     return

    global marker4, marker5, marker6, marker7, plane

    if data.markerNumber == 4 and marker4 == -1:
        marker4 = data
    elif data.markerNumber == 5 and marker5 == -1:
        marker5 = data
    elif data.markerNumber == 6 and marker6 == -1:
        marker6 = data
    elif data.markerNumber == 7 and marker7 == -1:
        marker7 = data
    
    if ((marker4 == -1) or (marker5 == -1) or (marker6 == -1) or (marker7 == -1)):
        return
    
    point1: Point3D = Point3D((marker4.x_m + marker5.x_m) / 2, (marker4.y_m + marker5.y_m) / 2, (marker4.z_m + marker5.z_m) / 2)
    point2: Point3D = Point3D((marker6.x_m + marker7.x_m) / 2, (marker6.y_m + marker7.y_m) / 2, (marker6.z_m + marker7.z_m) / 2)

    marker4 = -1
    marker5 = -1
    marker6 = -1
    marker7 = -1

    line = get_line_as_arguments(point1, point2)

    intersection_point = find_intersection(line, plane)

    # if ((intersection_point.x > 0.150) and (intersection_point.y > 0.300)):
    #     rospy.loginfo(f"{point1.x}, {point1.y}, {point1.z}")
    #     rospy.loginfo(f"{point2.x}, {point2.y}, {point2.z}")

    msg: CoP_position = CoP_position()
    msg.frameNumber = data.frameNumber
    msg.x_m = intersection_point.x
    msg.y_m = intersection_point.y
    msg.z_m = intersection_point.z

    publisher.publish(msg)


#############################################
def transceiver():
    name = 'CoP_marker'

    rospy.init_node(name, anonymous=True)

    global publisher
    publisher = rospy.Publisher(name, CoP_position, queue_size = 1000)
    
    rospy.Subscriber("Marker_global_translation", Marker_global_translation, callback)

    rospy.loginfo(f"{name} started.")

    rospy.spin()

#############################################
if __name__ == '__main__':
    transceiver()
