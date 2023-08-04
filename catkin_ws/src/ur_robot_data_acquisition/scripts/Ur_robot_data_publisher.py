#!/usr/bin/env python

import rospy
from ur_robot_data_acquisition.msg._Joint_parameters import Joint_parameters
import copy
from pathlib import Path
from rtde_receive import RTDEReceiveInterface as RTDEReceive

publisher = None
receiver: RTDEReceive

#############################################


def acquire_and_publish():
    t_start = receiver.initPeriod()
    actual_q_list: list[float] = receiver.getActualQ()
    actual_qd_list: list[float] = receiver.getActualQd()
    joint_parameters: Joint_parameters = Joint_parameters(actual_q_list, actual_qd_list)
    publisher.publish(joint_parameters)
    receiver.waitPeriod(t_start)
    return


#############################################
def main():
    rospy.init_node(f"{Path(__file__).stem}", anonymous=True)

    global publisher
    publisher = rospy.Publisher("Joint_parameters", Joint_parameters, queue_size=1000)

    global receiver
    receiver = RTDEReceive(hostname="192.168.12.1", frequency=500.0, variables=["actual_q", "actual_qd"])

    rospy.loginfo(f"{Path(__file__).stem}: started.")

    while not rospy.is_shutdown():
        acquire_and_publish()


#############################################
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
