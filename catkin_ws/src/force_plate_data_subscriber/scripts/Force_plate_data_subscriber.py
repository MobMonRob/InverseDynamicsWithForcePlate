#!/usr/bin/env python

import rospy
from force_plate_data_publisher.msg import Force_plate_data;

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.f_x)
    
def listener():
    rospy.init_node('Force_plate_data_subscriber', anonymous=True)

    rospy.Subscriber("Force_plate_data", Force_plate_data, callback)

    rospy.spin()

if __name__ == '__main__':
    listener() 
