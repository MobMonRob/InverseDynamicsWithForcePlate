import urdf2casadi.urdfparser as u2c
import numpy as np

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

tau_sym, force_first_joint_sym, force_base_sym = ur5.get_inverse_dynamics_rnea(root, tip)
# tau_sym_bu, force_first_joint_sym_bu, force_base_sym_bu = ur5.get_inverse_dynamics_rnea_bottom_up(root, tip)

q = [None]*n_joints
q_dot = [None]*n_joints
q_ddot = [None]*n_joints
for i in range(n_joints):
    #to make sure the inputs are within the robot's limits:
    q[i] = (q_max[i] - q_min[i])*np.random.rand()-(q_max[i] - q_min[i])/2
    q_dot[i] = (q_max[i] - q_min[i])*np.random.rand()-(q_max[i] - q_min[i])/2
    q_ddot[i] = (q_max[i] - q_min[i])*np.random.rand()-(q_max[i] - q_min[i])/2

tau_num_classic = tau_sym(q, q_dot, q_ddot)
print("RNEA numerical inverse dynamics: \n", tau_num_classic)

force_first_joint = force_first_joint_sym(q, q_dot, q_ddot)
print("Spatial force through the first joint: \n", force_first_joint)

force_base = force_base_sym(q, q_dot, q_ddot)
print("Spatial force of the base: \n", force_base)

print("############################################################")

tau_sym_bu = ur5.get_inverse_dynamics_rnea_bottom_up(root, tip)

# tau_num_bu = tau_sym_bu(q, q_dot, q_ddot, force_base)
# print("BU numerical inverse dynamics: \n", tau_num_bu)

f_sym = ur5.get_forces_bottom_up(root, tip, force_base)
f_num = f_sym(q)
print("BU forces: \n", f_num)

# numerical_values = [np.array(dm_val.toarray()) for dm_val in f_num]
# print("BU forces converted to numpy array: \n", numerical_values)

tau_sym_bu_f = ur5.get_inverse_dynamics_rnea_bottom_up_f(root, tip, f_num)
tau_sym_bu_f_num = tau_sym_bu_f(q)
print("BU numerical inverse dynamics: \n", tau_sym_bu_f_num)

# force_first_joint_bu = force_first_joint_sym_bu(q, q_dot, q_ddot)
# print("Spatial force through the first joint: \n", force_first_joint_bu)

# force_base_bu = force_base_sym_bu(q, q_dot, q_ddot)
# print("Spatial force of the base: \n", force_base_bu)

# iXp0_sym, iXp1_sym, iXp2_sym = ur5.get_model_calculation(root, tip)

# iXp0 = iXp0_sym(q)
# print("Model calculation: \n", iXp0)

# iXp1 = iXp1_sym(q)
# print("Model calculation: \n", iXp1)

#i_X_p, Si, Ic = ur5._model_calculation(root, tip, q)

#print("Transform matrices: \n", i_X_p)