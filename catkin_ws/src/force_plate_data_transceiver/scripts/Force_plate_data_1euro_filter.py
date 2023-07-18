#!/usr/bin/env python

import rospy
from vicon_data_publisher.msg import Force_plate_data

import sys
sys.path.append("/home/deralbert/BA/InverseDynamicsWithForcePlate/catkin_ws/src/force_plate_data_transceiver/scripts/")
from geometry.one_euro_filter import OneEuroFilter

publisher = None
# https://gery.casiez.net/1euro/
# Next, the body part is moved quickly in different directions while beta is increased with a focus on minimizing lag.
# First find the right order of magnitude to tune beta, which depends on the kind of data you manipulate and their units: do not hesitate to start with values like 0.001 or 0.0001. You can first multiply and divide beta by factor 10 until you notice an effect on latency when moving quickly.
# Note that parameters mincutoff and beta have clear conceptual relationships: if high speed lag is a problem, increase beta; if slow speed jitter is a problem, decrease mincutoff.
config : "dict[str, float]" = {
        'freq': 1200.0,    # Data Update rate
        'mincutoff': 0.5,  # Minimum cutoff frequency. Increasing mincutoff increases jitter and decreases lag.
        'beta': 0.01,      # Cutoff slope
        'dcutoff': 0.5     # Cutoff frequency for derivate
        }
filter_fx : OneEuroFilter = OneEuroFilter(**config)
filter_fy : OneEuroFilter = OneEuroFilter(**config)
filter_fz : OneEuroFilter = OneEuroFilter(**config)
filter_mx : OneEuroFilter = OneEuroFilter(**config)
filter_my : OneEuroFilter = OneEuroFilter(**config)
filter_mz : OneEuroFilter = OneEuroFilter(**config)

#############################################

def callback(data: Force_plate_data):
    time : float = (10*data.frameNumber + data.subsampleNumber) / 1200.0
    
    msg: Force_plate_data = Force_plate_data()
    msg.frameNumber = data.frameNumber
    msg.fx_N = filter_fx(data.fx_N, time)
    msg.fy_N = filter_fy(data.fy_N, time)
    msg.fz_N = filter_fz(data.fz_N, time)
    msg.mx_Nm = filter_mx(data.mx_Nm, time)
    msg.my_Nm = filter_my(data.my_Nm, time)
    msg.mz_Nm = filter_mz(data.mz_Nm, time)
    
    publisher.publish(msg)


#############################################
def transceiver():
    name = 'Force_plate_data_1euro_filter'

    rospy.init_node(name, anonymous=True)

    global publisher
    publisher = rospy.Publisher(name, Force_plate_data, queue_size = 1000)
    
    rospy.Subscriber("Force_plate_data", Force_plate_data, callback)

    rospy.loginfo(f"{name} started.")

    rospy.spin()

#############################################
if __name__ == '__main__':
    transceiver()
