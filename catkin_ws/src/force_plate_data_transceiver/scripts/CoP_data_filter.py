#!/usr/bin/env python

import rospy
from vicon_data_publisher.msg import Marker_global_translation
from vicon_data_publisher.msg import Force_plate_data

publisher_force_plate = None
publisher_marker = None
subscriber_force_plate = None
subscriber_marker = None

list_force_plate = []
list_marker = []

first_frame_force_plate = -1
first_frame_marker = -1

second_frame_reached_force_plate = False
#second_frame_reached_marker = False

second_loop = False
matching_condition = 50 # [Newtons]
first_frame_condition_matched = -1
last_frame_condition_matched = -1

#############################################

def publish_mode():
    subscriber_force_plate.unregister()
    subscriber_marker.unregister()

    force_plate_start_index = first_frame_condition_matched - first_frame_force_plate
    force_plate_to_index = last_frame_condition_matched - first_frame_force_plate + 1
    filtered_list_force_plate = list_force_plate[force_plate_start_index:force_plate_to_index]
    for force_plate_msg in filtered_list_force_plate:
        publisher_force_plate.publish(force_plate_msg)

    marker_start_index = first_frame_condition_matched - first_frame_marker
    marker_to_index = last_frame_condition_matched - first_frame_marker + 1
    filtered_list_marker = list_marker[marker_start_index:marker_to_index]
    for marker_msg in filtered_list_marker:
        publisher_marker.publish(marker_msg)
    
    rospy.loginfo(f"CoP_data_filter finished.")

def callback_force_plate(data):
    global second_loop
    global first_frame_force_plate
    global first_frame_condition_matched
    global last_frame_condition_matched
    global second_frame_reached_force_plate

    if ((data.frameNumber == first_frame_force_plate) and (second_frame_reached_force_plate == True)):
        second_loop = True
        publish_mode()
        return

    if (first_frame_force_plate == -1):
        first_frame_force_plate = data.frameNumber
    
    if ((second_frame_reached_force_plate == False) and (data.frameNumber > first_frame_force_plate)):
        second_frame_reached_force_plate = True
    
    list_force_plate.append(data)

    if ((data.fz_N > matching_condition) and (first_frame_condition_matched == -1)):
        first_frame_condition_matched = data.frameNumber
    
    if (data.fz_N > matching_condition):
        last_frame_condition_matched = data.frameNumber


def callback_marker(data):
    global first_frame_marker

    if (first_frame_marker == -1):
        first_frame_marker = data.frameNumber

    if (second_loop == True):
        return

    list_marker.append(data)


#############################################
def transceiver():
    rospy.init_node('CoP_data_filter', anonymous=True)

    global publisher_force_plate
    global publisher_marker
    global subscriber_force_plate
    global subscriber_marker

    publisher_force_plate = rospy.Publisher('CoP_force_plate_filter', Force_plate_data, queue_size = 1000)
    publisher_marker = rospy.Publisher('CoP_marker_filter', Marker_global_translation, queue_size = 1000)
    
    subscriber_force_plate = rospy.Subscriber("Force_plate_data_sma", Force_plate_data, callback_force_plate)
    subscriber_marker = rospy.Subscriber("Marker_global_translation", Marker_global_translation, callback_marker)

    rospy.loginfo(f"CoP_data_filter started.")

    rospy.spin()

#############################################
if __name__ == '__main__':
    transceiver()
