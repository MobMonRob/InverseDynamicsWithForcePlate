#!/usr/bin/env python

import rospy
from vicon_data_publisher.msg import Marker_global_translation
from queue import Queue
from pathlib import Path

publisher1: rospy.Publisher
publisher3: rospy.Publisher
publisher4: rospy.Publisher

publisher9: rospy.Publisher

#############################################


def callback(data: Marker_global_translation):
    if (data.markerNumber == 0):
        publisher1.publish(data)
    elif (data.markerNumber == 2):
        publisher3.publish(data)
    elif (data.markerNumber == 3):
        publisher4.publish(data)
    # if (data.markerNumber == 8):
    #     publisher9.publish(data)

#############################################


def transceiver():
    rospy.init_node(f"{Path(__file__).stem}", anonymous=True)

    global publisher1
    global publisher3
    global publisher4
    global publisher3

    global publisher9

    publisher1 = rospy.Publisher(
        'Marker_global_translation_filter_1', Marker_global_translation, queue_size=1000)
    publisher3 = rospy.Publisher(
        'Marker_global_translation_filter_3', Marker_global_translation, queue_size=1000)
    publisher4 = rospy.Publisher(
        'Marker_global_translation_filter_4', Marker_global_translation, queue_size=1000)

    # publisher9 = rospy.Publisher(
    #     'Marker_global_translation_filter_9', Marker_global_translation, queue_size=1000)

    rospy.Subscriber("Marker_global_translation",
                     Marker_global_translation, callback)

    rospy.loginfo(f"{Path(__file__).stem}: started.")

    rospy.spin()


#############################################
if __name__ == '__main__':
    transceiver()
