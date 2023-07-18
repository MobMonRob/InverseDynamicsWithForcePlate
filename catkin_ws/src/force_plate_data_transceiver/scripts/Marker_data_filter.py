#!/usr/bin/env python

import rospy
from vicon_data_publisher.msg import Marker_global_translation
from queue import Queue
import copy

publisher0 = None
publisher1 = None
publisher2 = None
publisher3 = None

publisher9 = None

#############################################

def callback(data: Marker_global_translation):
    if (data.markerNumber == 0):
        publisher0.publish(data)
    elif (data.markerNumber == 1):
        publisher1.publish(data)
    elif (data.markerNumber == 2):
        publisher2.publish(data)
    elif (data.markerNumber == 3):
        publisher3.publish(data)
    if (data.markerNumber == 9):
        publisher9.publish(data)

#############################################
def transceiver():
    rospy.init_node('Marker_data_filter', anonymous=True)

    global publisher0
    global publisher1
    global publisher2
    global publisher3

    global publisher9

    publisher0 = rospy.Publisher('Marker_global_translation_filter_0', Marker_global_translation, queue_size = 1000)
    publisher1 = rospy.Publisher('Marker_global_translation_filter_1', Marker_global_translation, queue_size = 1000)
    publisher2 = rospy.Publisher('Marker_global_translation_filter_2', Marker_global_translation, queue_size = 1000)
    publisher3 = rospy.Publisher('Marker_global_translation_filter_3', Marker_global_translation, queue_size = 1000)

    publisher9 = rospy.Publisher('Marker_global_translation_filter_9', Marker_global_translation, queue_size = 1000)
    
    rospy.Subscriber("Marker_global_translation", Marker_global_translation, callback)

    rospy.loginfo(f"Marker_data_filter started.")

    rospy.spin()

#############################################
if __name__ == '__main__':
    transceiver()
