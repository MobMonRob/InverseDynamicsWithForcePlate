#!/usr/bin/env python

import rospy
from force_plate_data_publisher.msg import Force_plate_data;

from queue import Queue

window_size = 8
queue = Queue(window_size)
current_count = 0
average = Force_plate_data()

#############################################
def callback(data):
    if (queue.qsize() < window_size):
        queue.put(data)
        average.f_x = average.f_x + data.f_x / window_size
        average.f_y = average.f_y + data.f_y / window_size
        average.f_z = average.f_z + data.f_z / window_size
        average.m_x = average.m_x + data.m_x / window_size
        average.m_y = average.m_y + data.m_y / window_size
        average.m_z = average.m_z + data.m_z / window_size
        return
    
    old = queue.get()
    queue.put(data)

    average.f_x = average.f_x + (data.f_x - old.f_x) / window_size
    average.f_y = average.f_y + (data.f_y - old.f_y) / window_size
    average.f_z = average.f_z + (data.f_z - old.f_z) / window_size
    average.m_x = average.m_x + (data.m_x - old.m_x) / window_size
    average.m_y = average.m_y + (data.m_y - old.m_y) / window_size
    average.m_z = average.m_z + (data.m_z - old.m_z) / window_size

    rospy.loginfo(f"\n{rospy.get_caller_id()}\nI heard\n{average}\n")

#############################################
def subscriber():
    rospy.init_node('Force_plate_data_subscriber', anonymous=True)

    rospy.Subscriber("Force_plate_data", Force_plate_data, callback)

    rospy.spin()

#############################################
if __name__ == '__main__':
    subscriber() 
