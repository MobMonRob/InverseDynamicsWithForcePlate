#!/usr/bin/env python

import rospy
from data_transformation.msg import Spatial_force, Joints_spatial_force
from vicon_data_publisher.msg import Force_plate_data
from ur_robot_data_acquisition.msg import Joint_parameters
import statistics
from rospy import Time
from dataclasses import dataclass
from typing import TypeVar, Generic
from Common.Inverse_dynamics_bottom_up import Inverse_dynamics_force_plate_ur5e, ThreeTuple, SixTupleTuple
from Common.Inverse_dynamics_top_down import Inverse_dynamics_top_down
from typing import Union

T = TypeVar('T')


@dataclass
class Timed_T(Generic[T]):
    time: Time
    value: T


Timed_q = Timed_T["list[float]"]


class Inverse_dynamics_node:
    def __init__(self):
        self.top_down: Inverse_dynamics_top_down = Inverse_dynamics_top_down()
        self.bottom_up: Inverse_dynamics_force_plate_ur5e = Inverse_dynamics_force_plate_ur5e()
        self.fpd_times: "list[Time]" = list()
        self.fpds: "list[Force_plate_data]" = list()
        self.previous_q: Timed_q = None
        self.previous_q_dot: Timed_q = None

    #############################################

    # Higher frequency

    def force_plate_data(self, fpd: Force_plate_data, time: rospy.Time):
        self.fpd_times.append(time)
        self.fpds.append(fpd)
        return

    #############################################

    # Lower frequency

    def joint_parameters(self, jp: Joint_parameters, time: rospy.Time) -> Union[Joints_spatial_force, None]:
        # Calculate q, q_dot, q_ddot
        qs = self.__calculate_qs(time=time, jp=jp)
        if qs == None:
            return None
        q, q_dot, q_ddot = qs

        # Calculate mean of Force_plate_data
        mean_fpd: Force_plate_data = self.__calculate_mean_fpd()
        if mean_fpd == None:
            return None
        f_force_plate: ThreeTuple = (mean_fpd.fx_N, mean_fpd.fy_N, mean_fpd.fz_N)
        m_force_plate: ThreeTuple = (mean_fpd.mx_Nm, mean_fpd.my_Nm, mean_fpd.mz_Nm)

        # Calculate torques
        # bottom_up_torques: SixTuple = bottom_up.calculate_torques(q=q.value, q_dot=q_dot.value, q_ddot=q_ddot.value, f_force_plate=f_force_plate, m_force_plate=m_force_plate)
        # top_down_torques: SixTuple = top_down.calculate_torques(q=q.value, q_dot=q_dot.value, q_ddot=q_ddot.value)

        # Validate bottom_up calculation part without force_plate.
        # Expect to see identical torques. Succeeded 11.08.2023.
        # top_down_torques: SixTuple = top_down.calculate_torques(q=q.value, q_dot=q_dot.value, q_ddot=q_ddot.value)
        # base_force: SixTuple = top_down.calculate_forces(q=q.value, q_dot=q_dot.value, q_ddot=q_ddot.value)[0]
        # bottom_up_torques: SixTuple = bottom_up.calculate_torques_from_base_force(q=q.value, q_dot=q_dot.value, q_ddot=q_ddot.value, base_force=base_force)

        # Calculate forces
        bottom_up_forces: SixTupleTuple = self.bottom_up.calculate_spatial_forces(q=q.value, q_dot=q_dot.value, q_ddot=q_ddot.value, f_force_plate=f_force_plate, m_force_plate=m_force_plate)
        top_down_forces: SixTupleTuple = self.top_down.calculate_spatial_forces(q=q.value, q_dot=q_dot.value, q_ddot=q_ddot.value)

        joints_bottom_up = [Spatial_force(m_xyz__f_xyz=force) for force in bottom_up_forces]
        joints_top_down = [Spatial_force(m_xyz__f_xyz=force) for force in top_down_forces]

        joints_spatial_force: Joints_spatial_force = Joints_spatial_force(joints_bottom_up=joints_bottom_up, joints_top_down=joints_top_down)

        self.fpd_times.clear()
        self.fpds.clear()
        return joints_spatial_force

    #############################################

    def __calculate_mean_fpd(self) -> "Union[Force_plate_data, None]":
        # Ohne Mitteln von Force_plate_data, mit (linearer) Interpolation von Joint_parameters, ginge es auch.
        if len(self.fpds) == 0:
            return None
        mean_fpd: Force_plate_data = Force_plate_data()
        for fieldName in Force_plate_data.__slots__:
            field_values_list: list[float] = [getattr(forcePlateData, fieldName) for forcePlateData in self.fpds]
            field_mean: float = statistics.fmean(field_values_list)
            setattr(mean_fpd, fieldName, field_mean)

        return mean_fpd

    def __calculate_qs(self, time: Time, jp: Joint_parameters) -> "Union[tuple[Timed_q, Timed_q, Timed_q], None]":
        q: Timed_q = Timed_q(time, jp.actual_joint_positions)

        if self.previous_q == None:
            self.previous_q = q
            return None

        # Jitter in actual_joint_positions will lead to wrong huge velocities.
        # Ideas to fix this: 1euroFilter on actual_joint_positions OR ignore huge velocities.
        # Current workaround: use actual_joint_velocities from the robot instead.
        # q_dot: Timed_q = backward_derivative_secs(q2=q, q1=previous_q)
        q_dot: Timed_q = Timed_q(time, jp.actual_joint_velocities)
        self.previous_q = q

        if self.previous_q_dot == None:
            self.previous_q_dot = q_dot
            return None

        q_ddot: Timed_q = Inverse_dynamics_node.__backward_derivative_secs(q2=q_dot, q1=self.previous_q_dot)
        self.previous_q_dot = q_dot

        return (q, q_dot, q_ddot)

    @staticmethod
    def __backward_derivative_secs(q2: Timed_q, q1: Timed_q) -> Timed_q:
        dt_s: float = (q2.time - q1.time).to_sec()
        dq_rad: list[float] = [q2.value[i] - q1.value[i] for i in range(6)]
        q_dot: list[float] = [dq_rad[i] / dt_s for i in range(6)]
        return Timed_q(time=q2.time, value=q_dot)
