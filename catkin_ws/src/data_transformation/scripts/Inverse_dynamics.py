#!/usr/bin/env python

import rospy
from data_transformation.msg import Joint_torques
from vicon_data_publisher.msg import Force_plate_data
from ur_robot_data_acquisition.msg import Joint_parameters
from pathlib import Path
import statistics
from rospy import Time
from dataclasses import dataclass
from typing import TypeVar, Generic
from Common.Inverse_dynamics_bottom_up import Inverse_dynamics_force_plate_ur5e, SixTuple, ThreeTuple
from Common.Inverse_dynamics_top_down import Inverse_dynamics_top_down
from typing import Union

T = TypeVar('T')


@dataclass
class Timed_T(Generic[T]):
    time: Time
    value: T


Timed_q = Timed_T["list[float]"]

publisher = None
top_down: Inverse_dynamics_top_down = Inverse_dynamics_top_down()
bottom_up: Inverse_dynamics_force_plate_ur5e = Inverse_dynamics_force_plate_ur5e()
# publisher_calculated_qs = None

#############################################


# Higher frequency
def callback_force_plate_data(fpd: Force_plate_data):
    fpd_times.append(Time.now())
    fpds.append(fpd)
    return


#############################################


# Lower frequency
def callback_joint_parameters(jp: Joint_parameters):
    global publisher
    global top_down
    global bottom_up
    # global publisher_calculated_qs

    # Calculate q, q_dot, q_ddot
    qs = calculate_qs(time=Time.now(), jp=jp)
    if qs == None:
        return
    q, q_dot, q_ddot = qs

    # Calculate mean of Force_plate_data
    mean_fpd: Force_plate_data = calculate_mean_fpd()
    if mean_fpd == None:
        return
    f_force_plate: ThreeTuple = (mean_fpd.fx_N, mean_fpd.fy_N, mean_fpd.fz_N)
    m_force_plate: ThreeTuple = (mean_fpd.mx_Nm, mean_fpd.my_Nm, mean_fpd.mz_Nm)

    # Calculate torques
    # bottom_up_torques: SixTuple = bottom_up.calculate_torques(q=q.value, q_dot=q_dot.value, q_ddot=q_ddot.value, f_force_plate=f_force_plate, m_force_plate=m_force_plate)
    # top_down_torques: SixTuple = top_down.calculate_torques(q=q.value, q_dot=q_dot.value, q_ddot=q_ddot.value)

    # Compare forces
    index = 0
    bottom_up_torques: SixTuple = bottom_up.calculate_spatial_forces(q=q.value, q_dot=q_dot.value, q_ddot=q_ddot.value, f_force_plate=f_force_plate, m_force_plate=m_force_plate)[index]
    top_down_torques: SixTuple = top_down.calculate_spatial_forces(q=q.value, q_dot=q_dot.value, q_ddot=q_ddot.value)[index]

    # Validate bottom_up calculation part without force_plate.
    # Expect to see identical torques. Succeeded 11.08.2023.
    # top_down_torques: SixTuple = top_down.calculate_torques(q=q.value, q_dot=q_dot.value, q_ddot=q_ddot.value)
    # base_force: SixTuple = top_down.calculate_forces(q=q.value, q_dot=q_dot.value, q_ddot=q_ddot.value)[0]
    # bottom_up_torques: SixTuple = bottom_up.calculate_torques_from_base_force(q=q.value, q_dot=q_dot.value, q_ddot=q_ddot.value, base_force=base_force)

    # publisher_calculated_qs.publish(Joint_parameters(actual_joint_positions=q_dot.value, actual_joint_velocities=q_ddot.value))

    joint_torques: Joint_torques = Joint_torques(bottom_up=bottom_up_torques, top_down=top_down_torques)

    publisher.publish(joint_torques)
    fpd_times.clear()
    fpds.clear()
    return


#############################################

fpd_times: "list[Time]" = list()
fpds: "list[Force_plate_data]" = list()


def calculate_mean_fpd() -> "Union[Force_plate_data, None]":
    global fpd_times
    global fpds

    # Ohne Mitteln von Force_plate_data, mit (linearer) Interpolation von Joint_parameters, ginge es auch.
    if len(fpds) == 0:
        return None
    mean_fpd: Force_plate_data = Force_plate_data()
    for fieldName in Force_plate_data.__slots__:
        field_values_list: list[float] = [getattr(forcePlateData, fieldName) for forcePlateData in fpds]
        field_mean: float = statistics.fmean(field_values_list)
        setattr(mean_fpd, fieldName, field_mean)

    return mean_fpd


previous_q: Timed_q = None
previos_q_dot: Timed_q = None


def calculate_qs(time: Time, jp: Joint_parameters) -> "Union[tuple[Timed_q, Timed_q, Timed_q], None]":
    global previous_q
    global previos_q_dot

    q: Timed_q = Timed_q(time, jp.actual_joint_positions)

    if previous_q == None:
        previous_q = q
        return None

    # Jitter in actual_joint_positions will lead to wrong huge velocities.
    # Ideas to fix this: 1euroFilter on actual_joint_positions OR ignore huge velocities.
    # Current workaround: use actual_joint_velocities from the robot instead.
    # q_dot: Timed_q = backward_derivative_secs(q2=q, q1=previous_q)
    q_dot: Timed_q = Timed_q(time, jp.actual_joint_velocities)
    previous_q = q

    if previos_q_dot == None:
        previos_q_dot = q_dot
        return None

    q_ddot: Timed_q = backward_derivative_secs(q2=q_dot, q1=previos_q_dot)
    previos_q_dot = q_dot

    return (q, q_dot, q_ddot)


def backward_derivative_secs(q2: Timed_q, q1: Timed_q) -> Timed_q:
    dt_s: float = (q2.time - q1.time).to_sec()
    dq_rad: list[float] = [q2.value[i] - q1.value[i] for i in range(6)]
    q_dot: list[float] = [dq_rad[i] / dt_s for i in range(6)]
    return Timed_q(time=q2.time, value=q_dot)


#############################################
def execute():
    rospy.init_node(f"{Path(__file__).stem}", anonymous=True)

    global publisher
    publisher = rospy.Publisher(f"{Path(__file__).stem}", Joint_torques, queue_size=1000)

    # global publisher_calculated_qs
    # publisher_calculated_qs = rospy.Publisher(f"{Path(__file__).stem}_dbg", Joint_parameters, queue_size=1000)

    # sma reacts too slowly here.
    rospy.Subscriber("Force_plate_data_1euro_filter", Force_plate_data, callback_force_plate_data)
    rospy.Subscriber("Joint_parameters", Joint_parameters, callback_joint_parameters)

    rospy.loginfo(f"{Path(__file__).stem}: started.")

    rospy.spin()


#############################################
if __name__ == '__main__':
    execute()
