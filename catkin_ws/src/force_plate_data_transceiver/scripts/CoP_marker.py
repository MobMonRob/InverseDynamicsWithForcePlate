#!/usr/bin/env python

import rospy
from vicon_data_publisher.msg import Marker_global_translation
from force_plate_data_transceiver.msg import CoP_position

import sys, os
sys.path.append("/home/deralbert/BA/InverseDynamicsWithForcePlate/catkin_ws/src/force_plate_data_transceiver/scripts/")
from geometry.find_line_plane_intersection import find_intersection, substitute_variables
from geometry.define_line import get_line_as_arguments, Point3D
from geometry.define_plane import define_plane
import numpy as np

from sympy.solvers import solve
from sympy import Symbol

publisher = None

marker0 = -1
marker1 = -1
marker2 = -1
marker3 = -1

plane = define_plane(np.array([0, 0, 0]), np.array([0.6, 0, 0]), np.array([0, 0.4, 0]))

#############################################

def callback(data: Marker_global_translation):
    # if (data.frameNumber % 10 > 0):
    #     return

    global marker0, marker1, marker2, marker3, plane

    if data.markerNumber == 0 and marker0 == -1:
        marker0 = data
    elif data.markerNumber == 1 and marker1 == -1:
        marker1 = data
    elif data.markerNumber == 2 and marker2 == -1:
        marker2 = data
    elif data.markerNumber == 3 and marker3 == -1:
        marker3 = data
    
    if ((marker0 == -1) or (marker1 == -1) or (marker2 == -1) or (marker3 == -1)):
        return
    
    point1: Point3D = Point3D((marker0.x_m + marker1.x_m) / 2, (marker0.y_m + marker1.y_m) / 2, (marker0.z_m + marker1.z_m) / 2)
    point2: Point3D = Point3D((marker2.x_m + marker3.x_m) / 2, (marker2.y_m + marker3.y_m) / 2, (marker2.z_m + marker3.z_m) / 2)

    marker0 = -1
    marker1 = -1
    marker2 = -1
    marker3 = -1

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