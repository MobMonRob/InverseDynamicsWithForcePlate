import urdf2casadi.urdfparser as u2c
import numpy as np
from typing import List

ur5 = u2c.URDFparser()
import os
path_to_urdf = absPath = os.path.dirname(os.path.abspath(__file__)) + '/../urdf/ur5e.urdf' 
ur5.from_file(path_to_urdf)

root = "base_link"
tip = "tool0"

joint_list, joint_names, q_max, q_min = ur5.get_joint_info(root, tip)
n_joints = ur5.get_n_joints(root, tip)
print("name of first joint:", joint_names[0], "\n")
print("joint information for first joint:\n", joint_list[0])
print("\n q max:", q_max)
print("\n q min:", q_min)

tau_sym, forces_sym, force_base_sym, forces_debug_sym = ur5.get_inverse_dynamics_rnea(root, tip)
# tau_sym_bu, force_first_joint_sym_bu, force_base_sym_bu = ur5.get_inverse_dynamics_rnea_bottom_up(root, tip)

# Generate q, q_dot, q_ddot
q = [None]*n_joints
q_dot = [None]*n_joints
q_ddot = [None]*n_joints
for i in range(n_joints):
    #to make sure the inputs are within the robot's limits:
    q[i] = (q_max[i] - q_min[i])*np.random.rand()-(q_max[i] - q_min[i])/2
    q_dot[i] = (q_max[i] - q_min[i])*np.random.rand()-(q_max[i] - q_min[i])/2
    q_ddot[i] = (q_max[i] - q_min[i])*np.random.rand()-(q_max[i] - q_min[i])/2

# q = [1, 2, 3, 4, 5 ,6]
# q_dot = [0.5, 1, 1.5, 2, 2.5, 3]
# q_ddot = [0.25, 0.5, 0.75, 1, 1.25, 1.5]


tau_num_classic = tau_sym(q, q_dot, q_ddot)
print("The output of the RNEA from urdf2casadi: \n", tau_num_classic)

forces = forces_sym(q, q_dot, q_ddot)
print("Spatial forces from this RNEA after update: \n", forces)

# force_before_update = forces_debug_sym(q, q_dot, q_ddot)
# print("Spatial forces from velocities and accelerations only: \n", force_before_update)

# each_force_transformed_sym = ur5.get_forces_bottom_up_from_forces(root, tip, forces)
# each_force_transformed = each_force_transformed_sym(q)
# print("Each_force_transformed: \n", each_force_transformed)

print("############################################################")

tau_sym_bu = ur5.get_inverse_dynamics_rnea_bottom_up(root, tip)

f_sym = ur5.get_forces_bottom_up_ver1(root, tip, forces[0])
f_num = f_sym(q, q_dot, q_ddot)
print("Bottom Up forces: \n", f_num)

tau_sym_bu_f = ur5.get_inverse_dynamics_rnea_bottom_up_f(root, tip, f_num)
tau_sym_bu_f_num = tau_sym_bu_f(q)
print("Bottom Up numerical inverse dynamics: \n", tau_sym_bu_f_num)

# i_X_p_sym = ur5.get_model_calculation(root, tip)
# i_X_p = i_X_p_sym(q)
# print("i_X_p: \n", i_X_p)