#!/usr/bin/env python

import rospy
from ur_robot_data_acquisition.msg import Actual_q
import copy
from pathlib import Path
from rtde_receive import RTDEReceiveInterface as RTDEReceive

publisher = None
receiver: RTDEReceive

#############################################

def acquire_and_publish():
    t_start = receiver.initPeriod()
    actual_q_list: list[float] = receiver.getActualQ()
    actual_q: Actual_q = Actual_q(actual_q_list)
    publisher.publish(actual_q)
    receiver.waitPeriod(t_start)
    return


#############################################
def main():
    rospy.init_node(f"{Path(__file__).stem}", anonymous=True)

    global publisher
    publisher = rospy.Publisher('Actual_q', Actual_q, queue_size = 1000)

    global receiver
    receiver = RTDEReceive(hostname="192.168.12.1", frequency=500.0, variables=["actual_q"])

    rospy.loginfo(f"{Path(__file__).stem}: started.")

    while not rospy.is_shutdown():
        acquire_and_publish()


#############################################
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
