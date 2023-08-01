import os
import urdf2casadi.urdfparser as u2c
import numpy as np
from typing import List
import math
import time


def forces_force_plate_to_forces_ur5e_base(forces: "list[float]") -> "list[float]":
    # R is calculated only once and it is
    R = np.array([[1,  0,  0],
                  [0, -1,  0],
                  [0,  0, -1]])

    forces_transformed = R.dot(forces)
    return forces_transformed


def moments_force_plate_to_moments_ur5e_base(forces_ur5e_base: "list[float]", moments: "list[float]") -> "list[float]":
    # R is calculated only once and it is
    R = np.array([[1,  0,  0],
                  [0, -1,  0],
                  [0,  0, -1]])

    # GC is position of (center of plate surface) in robot base frame
    GC = [-1.7915 / 1000, -5.1695 / 1000, -(22.58 + 1.4) / 1000]

    OC = [0, 0, -44.78 / 1000]

    GO = GC - R.dot(OC)

    # K is skew-symmetric, it is
    K = np.array([[0,     - GO[2],     GO[1]],
                  [GO[2],       0,   - GO[0]],
                  [-GO[1],   GO[0],         0]])

    moments_transformed = R.dot(moments) + K.dot(forces_ur5e_base)
    return moments_transformed


ur5 = u2c.URDFparser()
path_to_urdf = absPath = os.path.dirname(os.path.abspath(__file__)) + '/../urdf/ur5_mod.urdf'
ur5.from_file(path_to_urdf)

root = "base_link"
tip = "tool0"

joint_list, joint_names, q_max, q_min = ur5.get_joint_info(root, tip)
n_joints = ur5.get_n_joints(root, tip)
print("name of first joint:", joint_names[0], "\n")
print("joint information for first joint:\n", joint_list[0])
print("\n q max:", q_max)
print("\n q min:", q_min)

start_time = time.perf_counter()
tau_sym, forces_sym, force_base_sym, forces_debug_sym = ur5.get_inverse_dynamics_rnea(root, tip, gravity=[0, 0, -9.81])
end_time = time.perf_counter()
elapsed_time = end_time - start_time
print("Elapsed time 1: ", elapsed_time)
# tau_sym_bu, force_first_joint_sym_bu, force_base_sym_bu = ur5.get_inverse_dynamics_rnea_bottom_up(root, tip)

# Generate q, q_dot, q_ddot
q = [None]*n_joints
q_dot = [None]*n_joints
q_ddot = [None]*n_joints
for i in range(n_joints):
    # to make sure the inputs are within the robot's limits:
    q[i] = (q_max[i] - q_min[i])*np.random.rand()-(q_max[i] - q_min[i])/2
    q_dot[i] = (q_max[i] - q_min[i])*np.random.rand()-(q_max[i] - q_min[i])/2
    q_ddot[i] = (q_max[i] - q_min[i])*np.random.rand()-(q_max[i] - q_min[i])/2

# q = [0, 2, 3, 4, 5, 6]
# q_dot = [0.5, 1, 1.5, 2, 2.5, 3]
# q_ddot = [0.25, 0.5, 0.75, 1, 1.25, 1.5]

q = [math.pi, -2.3345737645286135E-6, -2.3345737645286135E-6, -math.pi / 2, 2.382993625360541E-5, math.pi]
q_dot = [0, 0, 0, 0, 0, 0]
q_ddot = [0, 0, 0, 0, 0, 0]

start_time = time.perf_counter()
tau_num_classic = tau_sym(q, q_dot, q_ddot)
end_time = time.perf_counter()
elapsed_time = end_time - start_time
print("Elapsed time 2: ", elapsed_time)
print("The output of the RNEA from urdf2casadi: \n", tau_num_classic)

forces = forces_sym(q, q_dot, q_ddot)
print("Spatial forces from this RNEA after update: \n", forces)

# force_before_update = forces_debug_sym(q, q_dot, q_ddot)
# print("Spatial forces from velocities and accelerations only: \n", force_before_update)

# each_force_transformed_sym = ur5.get_forces_bottom_up_from_forces(root, tip, forces)
# each_force_transformed = each_force_transformed_sym(q)
# print("Each_force_transformed: \n", each_force_transformed)

print("############################################################")

# TODO: Statt f_num die Kräfte und Momente [2.871, -1.971, 172.724, -17.089, -61.873, -0.089] nehmen (sie sind im lokalen Koordinatensystem der Kraftmessplatte), danach forces_force_plate_to_forces_ur5e_base, danach moments_force_plate_to_moments_ur5e_base, danach eine Spatial Kraft zusammenbauen, danach f_num diese Kraft zuweisen.
f_force_plate = [2.871, -1.971, 172.724]
f_ur5e_base = forces_force_plate_to_forces_ur5e_base(f_force_plate)

m_force_plate = [-17.089, -61.873, -0.089]
m_ur5e_base = moments_force_plate_to_moments_ur5e_base(f_ur5e_base, m_force_plate)

f_spacial_ur5e_base = np.concatenate([f_ur5e_base, m_ur5e_base])

tau_sym_bu = ur5.get_inverse_dynamics_rnea_bottom_up(root, tip)

f_sym = ur5.get_forces_bottom_up(root, tip, f_spacial_ur5e_base, gravity=[0, 0, -9.81])
f_num = f_sym(q, q_dot, q_ddot)
print("Bottom Up forces: \n", f_num)

tau_sym_bu_f = ur5.get_inverse_dynamics_rnea_bottom_up_f(root, tip, f_num)
tau_sym_bu_f_num = tau_sym_bu_f(q)
print("Bottom Up numerical inverse dynamics: \n", tau_sym_bu_f_num)

# i_X_p_sym = ur5.get_model_calculation(root, tip)
# i_X_p = i_X_p_sym(q)
# print("i_X_p: \n", i_X_p)
