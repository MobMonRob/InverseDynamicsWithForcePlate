#!/usr/bin/env python

import rospy
from data_transformation.msg import Joints_spatial_force, Joints_spatial_force_normed
from pathlib import Path
from numpy.linalg import norm

publisher: rospy.Publisher = None

#############################################


def callback(jsf: Joints_spatial_force):
    global publisher

    joints_bottom_up_m = [None] * 6
    joints_bottom_up_f = [None] * 6
    joints_top_down_m = [None] * 6
    joints_top_down_f = [None] * 6
    for i in range(6):
        jbu_mf = jsf.joints_bottom_up[i].m_xyz__f_xyz
        joints_bottom_up_m[i] = norm(jbu_mf[0:3])
        joints_bottom_up_f[i] = norm(jbu_mf[3:6])

        jtd_mf = jsf.joints_top_down[i].m_xyz__f_xyz
        joints_top_down_m[i] = norm(jtd_mf[0:3])
        joints_top_down_f[i] = norm(jtd_mf[3:6])

    jsfn = Joints_spatial_force_normed(joints_bottom_up_m=joints_bottom_up_m, joints_bottom_up_f=joints_bottom_up_f, joints_top_down_m=joints_top_down_m, joints_top_down_f=joints_top_down_f)

    publisher.publish(jsfn)
    return


#############################################


def execute():
    name: str = f"{Path(__file__).stem}"

    rospy.init_node(name, anonymous=True)

    global publisher
    publisher = rospy.Publisher(name, Joints_spatial_force_normed, queue_size=1000)

    rospy.Subscriber("Inverse_dynamics", Joints_spatial_force, callback)

    rospy.loginfo(f"{name}: started.")

    rospy.spin()


#############################################
if __name__ == '__main__':
    execute()
