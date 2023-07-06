#!/usr/bin/env python

import rospy
from force_plate_data_publisher.msg import Force_plate_data;

from queue import Queue

window_size = 1000
queue = Queue(window_size)
current_count = 0
average = Force_plate_data()

publisher = None

offset_x = 0
offset_y = 0
offset_z = 0

#############################################
def callback(data):
    global average
    global offset_x
    global offset_y
    global offset_z

    # Massen bestimmen
    data.f_x = data.f_x
    data.f_y = data.f_y
    data.f_z = data.f_z / 9.81

    if (queue.qsize() < window_size):
        rospy.loginfo(f"{queue.qsize()}")
        queue.put(data)
        average.f_x = average.f_x + data.f_x / window_size
        average.f_y = average.f_y + data.f_y / window_size
        average.f_z = average.f_z + data.f_z / window_size
        average.m_x = average.m_x + data.m_x / window_size
        average.m_y = average.m_y + data.m_y / window_size
        average.m_z = average.m_z + data.m_z / window_size
        offset_x = average.f_x
        offset_y = average.f_y
        offset_z = average.f_z
        return

    # Offset von Data abziehen
    data.f_x = data.f_x -offset_x
    data.f_y = data.f_y -offset_y
    data.f_z = data.f_z -offset_z
    # Hier nochmal 100 abwarten, bis die alten alle raus sind.

    old = queue.get()
    queue.put(data)

    average.f_x = average.f_x + (data.f_x - old.f_x) / window_size
    average.f_y = average.f_y + (data.f_y - old.f_y) / window_size
    average.f_z = average.f_z + (data.f_z - old.f_z) / window_size
    average.m_x = average.m_x + (data.m_x - old.m_x) / window_size
    average.m_y = average.m_y + (data.m_y - old.m_y) / window_size
    average.m_z = average.m_z + (data.m_z - old.m_z) / window_size

    publisher.publish(average)

    #rospy.loginfo(f"\n{rospy.get_caller_id()}\nI heard\n{average}\n")

#############################################
def transceiver():
    rospy.init_node('force_plate_data_transceiver', anonymous=True)

    global publisher
    publisher = rospy.Publisher('Force_plate_data_sma', Force_plate_data, queue_size = 1000)
    rospy.Subscriber("Force_plate_data", Force_plate_data, callback)

    rospy.loginfo(f"Tranceiver started.")

    rospy.spin()

#############################################
if __name__ == '__main__':
    transceiver() 
