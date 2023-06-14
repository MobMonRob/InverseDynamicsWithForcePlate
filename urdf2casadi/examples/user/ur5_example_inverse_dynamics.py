
# coding: utf-8

# In[1]:


import urdf2casadi.urdfparser as u2c
import numpy as np


# ## 1. Load robot from urdf 
# 
# 1. Create urdfparser-class instance. 
# 2. Load model to instance, either from file, string or ros parameter server. Examples uses from file. 

# In[2]:


ur5 = u2c.URDFparser()
import os
path_to_urdf = absPath = os.path.dirname(os.path.abspath(__file__)) + '/../urdf/ur5_mod.urdf' 
ur5.from_file(path_to_urdf)


# ## 2. Get joint information ? 
# Information about the joints of the robot model can be obtained using u2c's get_joint_info(). "root" and "tip" are the inputs, and represent the link names for the root and tip of the kinematic tree one wishes to evaluate. 

# In[3]:


root = "base_link"
tip = "tool0"

joint_list, joint_names, q_max, q_min = ur5.get_joint_info(root, tip)
n_joints = ur5.get_n_joints(root, tip)
print("name of first joint:", joint_names[0], "\n")
print("joint information for first joint:\n", joint_list[0])
print("\n q max:", q_max)
print("\n q min:", q_min)

# ## Inverse Dynamics 
# 
# Without accounting for gravitational forces:

# In[6]:


tau_sym = ur5.get_inverse_dynamics_rnea(root, tip)


# Accounting for gravitational forces:

# In[7]:


gravity = [0, 0, -9.81]
tau_g_sym = ur5.get_inverse_dynamics_rnea(root, tip, gravity = gravity)


# External forces can also be accounted for:

# In[8]:


q = [None]*n_joints
q_dot = [None]*n_joints
q_ddot = [None]*n_joints
for i in range(n_joints):
    #to make sure the inputs are within the robot's limits:
    q[i] = (q_max[i] - q_min[i])*np.random.rand()-(q_max[i] - q_min[i])/2
    q_dot[i] = (q_max[i] - q_min[i])*np.random.rand()-(q_max[i] - q_min[i])/2
    q_ddot[i] = (q_max[i] - q_min[i])*np.random.rand()-(q_max[i] - q_min[i])/2

tau_num = tau_sym(q, q_dot, q_ddot)
tau_g_num = tau_g_sym(q, q_dot, q_ddot)
#tau_fext_num = tau_fext_sym(q, q_dot, q_ddot)
print("Numerical inverse dynamics: \n", tau_num)
print("\nNumerical inverse dynamics w/ gravity: \n", tau_g_num)
#print("\nNumerical inverse dynamics w/ external forces: \n", G_num)
