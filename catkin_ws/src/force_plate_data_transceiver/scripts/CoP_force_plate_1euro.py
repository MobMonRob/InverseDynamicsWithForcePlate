#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from vicon_data_publisher.msg import Force_plate_data

import sys, os
sys.path.append("/home/deralbert/BA/InverseDynamicsWithForcePlate/catkin_ws/src/force_plate_data_transceiver/scripts/")
from geometry.define_line import Point3D
from geometry.calculate_cop import get_cop_as_arguments

publisher = None

#############################################

def callback(data: Force_plate_data):
    # FrameNumber. -> Evtl. überall zusätzlich ROS Header für Plotjuggler. Zuweisen nicht vergessen am Anfang.
    point3d: Point3D = get_cop_as_arguments(data.fx_N, data.fy_N, data.fz_N, data.mx_Nm, data.my_Nm, data.mz_Nm)
    msg: Point = Point()
    msg.x = point3d.x
    msg.y = point3d.y
    msg.z = point3d.z
    if (abs(point3d.x) > 1):
        msg.x = -1
    if (abs(point3d.y) > 1):
        msg.y = -1
    if (abs(point3d.z) > 1):
        msg.z = -1
    publisher.publish(msg)


#############################################
def transceiver():
    name = 'CoP_force_plate_1euro'
    rospy.init_node(name, anonymous=True)

    global publisher
    publisher = rospy.Publisher(name, Point, queue_size = 1000)
    
    rospy.Subscriber("Force_plate_data_1euro_filter", Force_plate_data, callback)

    rospy.loginfo(f"{name} started.")

    rospy.spin()

#############################################
if __name__ == '__main__':
    transceiver()
