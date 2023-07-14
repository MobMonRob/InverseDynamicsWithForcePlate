#!/usr/bin/env python

import rospy
from vicon_data_publisher.msg import Marker_global_translation
from queue import Queue
import copy

publisher = None

#############################################

def callback(data):
    if (data.markerNumber == 0):
        publisher.publish(data)

#############################################
def transceiver():
    rospy.init_node('Marker_data_filter', anonymous=True)

    global publisher
    publisher = rospy.Publisher('Marker_global_translation_filter', Marker_global_translation, queue_size = 1000)
    rospy.Subscriber("Marker_global_translation", Marker_global_translation, callback)

    rospy.loginfo(f"Marker_data_filter started.")

    rospy.spin()

#############################################
if __name__ == '__main__':
    transceiver()
