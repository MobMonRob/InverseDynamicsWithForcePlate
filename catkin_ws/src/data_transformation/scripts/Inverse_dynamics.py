#!/usr/bin/env python

import rospy
from data_transformation.msg import Joints_spatial_force
from vicon_data_publisher.msg import Force_plate_data
from ur_robot_data_acquisition.msg import Joint_parameters
from pathlib import Path
from rospy import Time
from Common.Inverse_dynamics_node import Inverse_dynamics_node

publisher: rospy.Publisher = None
iv_node: Inverse_dynamics_node = Inverse_dynamics_node()

#############################################


# Higher frequency
def callback_force_plate_data(fpd: Force_plate_data):
    global iv_node
    iv_node.force_plate_data(fpd, Time.now())
    return


#############################################


# Lower frequency
def callback_joint_parameters(jp: Joint_parameters):
    global iv_node
    global publisher
    joints_spatial_force: Joints_spatial_force = iv_node.joint_parameters(jp, Time.now())
    if joints_spatial_force == None:
        return
    publisher.publish(joints_spatial_force)


#############################################


def execute():
    rospy.init_node(f"{Path(__file__).stem}", anonymous=True)

    global publisher
    publisher = rospy.Publisher(f"{Path(__file__).stem}", Joints_spatial_force, queue_size=1000)

    # sma reacts too slowly here.
    rospy.Subscriber("Force_plate_data_1euro_filter", Force_plate_data, callback_force_plate_data)
    rospy.Subscriber("Joint_parameters", Joint_parameters, callback_joint_parameters)

    rospy.loginfo(f"{Path(__file__).stem}: started.")

    rospy.spin()


#############################################
if __name__ == '__main__':
    execute()
