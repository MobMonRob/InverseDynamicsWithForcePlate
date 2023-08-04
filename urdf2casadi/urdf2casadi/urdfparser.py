"""This module contains a class for turning a chain in a URDF to a
casadi function.
"""
import casadi as cs
import numpy as np
from platform import machine, system
from urdf_parser_py.urdf import URDF, Pose
import urdf2casadi.geometry.transformation_matrix as T
import urdf2casadi.geometry.plucker as plucker
import urdf2casadi.geometry.quaternion as quaternion
import urdf2casadi.geometry.dual_quaternion as dual_quaternion


class URDFparser(object):
    """Class that turns a chain from URDF to casadi functions."""
    actuated_types = ["prismatic", "revolute", "continuous"]
    func_opts = {}
    jit_func_opts = {"jit": True, "jit_options": {"flags": "-Ofast"}}
    # OS/CPU dependent specification of compiler
    if system().lower() == "darwin" or machine().lower() == "aarch64":
        jit_func_opts["compiler"] = "shell"

    def __init__(self, func_opts=None, use_jit=True):
        self.robot_desc = None
        if func_opts:
            self.func_opts = func_opts
        if use_jit:
            # NOTE: use_jit=True requires that CasADi is built with Clang
            for k, v in self.jit_func_opts.items():
                self.func_opts[k] = v

    def from_file(self, filename):
        """Uses an URDF file to get robot description."""
        self.robot_desc = URDF.from_xml_file(filename)

    def from_server(self, key="robot_description"):
        """Uses a parameter server to get robot description."""
        self.robot_desc = URDF.from_parameter_server(key=key)

    def from_string(self, urdfstring):
        """Uses a URDF string to get robot description."""
        self.robot_desc = URDF.from_xml_string(urdfstring)

    def get_joint_info(self, root, tip):
        """Using an URDF to extract joint information, i.e list of
        joints, actuated names and upper and lower limits."""
        chain = self.robot_desc.get_chain(root, tip)
        if self.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        joint_list = []
        upper = []
        lower = []
        actuated_names = []

        for item in chain:
            if item in self.robot_desc.joint_map:
                joint = self.robot_desc.joint_map[item]
                joint_list += [joint]
                if joint.type in self.actuated_types:
                    actuated_names += [joint.name]
                    if joint.type == "continuous":
                        upper += [cs.inf]
                        lower += [-cs.inf]
                    else:
                        upper += [joint.limit.upper]
                        lower += [joint.limit.lower]
                    if joint.axis is None:
                        joint.axis = [1., 0., 0.]
                    if joint.origin is None:
                        joint.origin = Pose(xyz=[0., 0., 0.],
                                            rpy=[0., 0., 0.])
                    elif joint.origin.xyz is None:
                        joint.origin.xyz = [0., 0., 0.]
                    elif joint.origin.rpy is None:
                        joint.origin.rpy = [0., 0., 0.]

        return joint_list, actuated_names, upper, lower

    def get_dynamics_limits(self, root, tip):
        """Using an URDF to extract joint max effort and velocity"""

        chain = self.robot_desc.get_chain(root, tip)
        if self.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        max_effort = []
        max_velocity = []

        for item in chain:
            if item in self.robot_desc.joint_map:
                joint = self.robot_desc.joint_map[item]
                if joint.type in self.actuated_types:
                    if joint.limit is None:
                        max_effort += [cs.inf]
                        max_velocity += [cs.inf]
                    else:
                        max_effort += [joint.limit.effort]
                        max_velocity += [joint.limit.velocity]
        max_effort = [cs.inf if x is None else x for x in max_effort]
        max_velocity = [cs.inf if x is None else x for x in max_velocity]

        return max_effort, max_velocity

    def get_friction_matrices(self, root, tip):
        """Using an URDF to extract joint frictions and dampings"""

        chain = self.robot_desc.get_chain(root, tip)
        if self.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        friction = []
        damping = []

        for item in chain:
            if item in self.robot_desc.joint_map:
                joint = self.robot_desc.joint_map[item]
                if joint.type in self.actuated_types:
                    if joint.dynamis is None:
                        friction += [0]
                        damping += [0]
                    else:
                        friction += [joint.dynamics.friction]
                        damping += [joint.dynamics.damping]
        friction = [0 if x is None else x for x in friction]
        damping = [0 if x is None else x for x in damping]
        Fv = np.diag(friction)
        Fd = np.diag(damping)
        return Fv, Fd

    def get_n_joints(self, root, tip):
        """Returns number of actuated joints."""

        chain = self.robot_desc.get_chain(root, tip)
        n_actuated = 0

        for item in chain:
            if item in self.robot_desc.joint_map:
                joint = self.robot_desc.joint_map[item]
                if joint.type in self.actuated_types:
                    n_actuated += 1

        return n_actuated

    def _model_calculation(self, root, tip, q):
        """Calculates and returns model information needed in the
        dynamics algorithms caluculations, i.e transforms, joint space
        and inertia."""
        if self.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        chain = self.robot_desc.get_chain(root, tip)
        spatial_inertias = []
        i_X_0 = []
        i_X_p = []
        Sis = []
        prev_joint = None
        n_actuated = 0
        i = 0

        for item in chain:
            if item in self.robot_desc.joint_map:
                joint = self.robot_desc.joint_map[item]

                if joint.type == "fixed":
                    if prev_joint == "fixed":
                        XT_prev = cs.mtimes(
                            plucker.XT(joint.origin.xyz, joint.origin.rpy),
                            XT_prev)
                    else:
                        XT_prev = plucker.XT(
                            joint.origin.xyz,
                            joint.origin.rpy)
                    inertia_transform = XT_prev
                    prev_inertia = spatial_inertia

                elif joint.type == "prismatic":
                    if n_actuated != 0:
                        spatial_inertias.append(spatial_inertia)
                    n_actuated += 1
                    XJT = plucker.XJT_prismatic(
                        joint.origin.xyz,
                        joint.origin.rpy,
                        joint.axis, q[i])
                    if prev_joint == "fixed":
                        XJT = cs.mtimes(XJT, XT_prev)
                    Si = cs.SX([0, 0, 0,
                                joint.axis[0],
                                joint.axis[1],
                                joint.axis[2]])
                    i_X_p.append(XJT)
                    Sis.append(Si)
                    i += 1

                elif joint.type in ["revolute", "continuous"]:
                    if n_actuated != 0:
                        spatial_inertias.append(spatial_inertia)
                    n_actuated += 1

                    XJT = plucker.XJT_revolute(
                        joint.origin.xyz,
                        joint.origin.rpy,
                        joint.axis,
                        q[i])
                    if prev_joint == "fixed":
                        XJT = cs.mtimes(XJT, XT_prev)
                    Si = cs.SX([
                        joint.axis[0],
                        joint.axis[1],
                        joint.axis[2],
                        0,
                        0,
                        0])
                    i_X_p.append(XJT)
                    Sis.append(Si)
                    i += 1

                prev_joint = joint.type

            if item in self.robot_desc.link_map:
                link = self.robot_desc.link_map[item]

                if link.inertial is None:
                    spatial_inertia = np.zeros((6, 6))
                else:
                    I = link.inertial.inertia
                    spatial_inertia = plucker.spatial_inertia_matrix_IO(
                        I.ixx,
                        I.ixy,
                        I.ixz,
                        I.iyy,
                        I.iyz,
                        I.izz,
                        link.inertial.mass,
                        link.inertial.origin.xyz)

                if prev_joint == "fixed":
                    spatial_inertia = prev_inertia + cs.mtimes(
                        inertia_transform.T,
                        cs.mtimes(spatial_inertia, inertia_transform))

                if link.name == tip:
                    spatial_inertias.append(spatial_inertia)

        return i_X_p, Sis, spatial_inertias

    def _apply_external_forces(self, external_f, f, i_X_p):
        """Internal function for applying external forces in dynamics
        algorithms calculations."""
        for i in range(0, len(f)):
            f[i] -= cs.mtimes(i_X_p[i].T, external_f[i])
        return f

    def _apply_external_forces_bottom_up(self, external_f, f, i_X_p):
        """Internal function for applying external forces in dynamics
        algorithms calculations."""
        for i in range(0, len(f)):
            f[i] -= cs.mtimes(i_X_p[i].T, external_f[i])
        return f

    def get_inverse_dynamics_rnea(self, root, tip,
                                  gravity=None, f_ext=None):
        """Returns the inverse dynamics as a casadi function."""
        if self.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        n_joints = self.get_n_joints(root, tip)
        q = cs.SX.sym("q", n_joints)
        q_dot = cs.SX.sym("q_dot", n_joints)
        q_ddot = cs.SX.sym("q_ddot", n_joints)
        i_X_p, Si, Ic = self._model_calculation(root, tip, q)

        v = []
        a = []
        f = []
        tau = cs.SX.zeros(n_joints)

        for i in range(0, n_joints):
            vJ = cs.mtimes(Si[i], q_dot[i])
            if i == 0:
                v.append(vJ)
                if gravity is not None:
                    ag = np.array([0.,
                                   0.,
                                   0.,
                                   gravity[0],
                                   gravity[1],
                                   gravity[2]])
                    a.append(
                        cs.mtimes(i_X_p[i], -ag) + cs.mtimes(Si[i], q_ddot[i]))
                else:
                    a.append(cs.mtimes(Si[i], q_ddot[i]))
            else:
                v.append(cs.mtimes(i_X_p[i], v[i-1]) + vJ)
                a.append(
                    cs.mtimes(i_X_p[i], a[i-1])
                    + cs.mtimes(Si[i], q_ddot[i])
                    + cs.mtimes(plucker.motion_cross_product(v[i]), vJ))

            f.append(
                cs.mtimes(Ic[i], a[i])
                + cs.mtimes(
                    plucker.force_cross_product(v[i]),
                    cs.mtimes(Ic[i], v[i])))

        if f_ext is not None:
            f = self._apply_external_forces(f_ext, f, i_X_p)

        fdebug = []
        fdebug.append(f[n_joints-1])

        for i in range(n_joints-1, -1, -1):
            tau[i] = cs.mtimes(Si[i].T, f[i])
            if i != 0:
                fdebug.append(f[i - 1])
                f[i-1] = f[i-1] + cs.mtimes(i_X_p[i].T, f[i])

        tau = cs.Function("C", [q, q_dot, q_ddot], [tau], self.func_opts)
        forces = cs.Function("f_f_j", [q, q_dot, q_ddot], [f[0], f[1], f[2], f[3], f[4], f[5]], self.func_opts)
        force_base = cs.Function("f_b", [q, q_dot, q_ddot], [cs.mtimes(i_X_p[0].T, f[0])], self.func_opts)
        forces_debug = cs.Function("f_dbg", [q, q_dot, q_ddot], [fdebug[0], fdebug[1], fdebug[2], fdebug[3], fdebug[4], fdebug[5]], self.func_opts)
        return tau, forces, force_base, forces_debug

    def get_gravity_rnea(self, root, tip, gravity):
        """Returns the gravitational term as a casadi function."""

        if self.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        n_joints = self.get_n_joints(root, tip)
        q = cs.SX.sym("q", n_joints)
        i_X_p, Si, Ic = self._model_calculation(root, tip, q)

        v = []
        a = []
        ag = cs.SX([0., 0., 0., gravity[0], gravity[1], gravity[2]])
        f = []
        tau = cs.SX.zeros(n_joints)

        for i in range(0, n_joints):
            if i == 0:
                a.append(cs.mtimes(i_X_p[i], -ag))
            else:
                a.append(cs.mtimes(i_X_p[i], a[i-1]))
            f.append(cs.mtimes(Ic[i], a[i]))

        for i in range(n_joints-1, -1, -1):
            tau[i] = cs.mtimes(Si[i].T, f[i])
            if i != 0:
                f[i-1] = f[i-1] + cs.mtimes(i_X_p[i].T, f[i])

        tau = cs.Function("C", [q], [tau],
                          self.func_opts)
        return tau

    def _get_M(self, Ic, i_X_p, Si, n_joints, q):
        """Internal function for calculating the inertia matrix."""
        M = cs.SX.zeros(n_joints, n_joints)
        Ic_composite = [None]*len(Ic)

        for i in range(0, n_joints):
            Ic_composite[i] = Ic[i]

        for i in range(n_joints-1, -1, -1):
            if i != 0:
                Ic_composite[i-1] = (Ic[i-1]
                                     + cs.mtimes(i_X_p[i].T,
                                                 cs.mtimes(Ic_composite[i], i_X_p[i])))

        for i in range(0, n_joints):
            fh = cs.mtimes(Ic_composite[i], Si[i])
            M[i, i] = cs.mtimes(Si[i].T, fh)
            j = i
            while j != 0:
                fh = cs.mtimes(i_X_p[j].T, fh)
                j -= 1
                M[i, j] = cs.mtimes(Si[j].T, fh)
                M[j, i] = M[i, j]

        return M

    def get_inertia_matrix_crba(self, root, tip):
        """Returns the inertia matrix as a casadi function."""
        if self.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        n_joints = self.get_n_joints(root, tip)
        q = cs.SX.sym("q", n_joints)
        i_X_p, Si, Ic = self._model_calculation(root, tip, q)
        M = cs.SX.zeros(n_joints, n_joints)
        Ic_composite = [None]*len(Ic)

        for i in range(0, n_joints):
            Ic_composite[i] = Ic[i]

        for i in range(n_joints-1, -1, -1):
            if i != 0:
                Ic_composite[i-1] = Ic[i-1] + cs.mtimes(i_X_p[i].T, cs.mtimes(Ic_composite[i], i_X_p[i]))

        for i in range(0, n_joints):
            fh = cs.mtimes(Ic_composite[i], Si[i])
            M[i, i] = cs.mtimes(Si[i].T, fh)
            j = i
            while j != 0:
                fh = cs.mtimes(i_X_p[j].T, fh)
                j -= 1
                M[i, j] = cs.mtimes(Si[j].T, fh)
                M[j, i] = M[i, j]

        M = cs.Function("M", [q], [M], self.func_opts)
        return M

    def _get_C(self, i_X_p, Si, Ic, q, q_dot, n_joints,
               gravity=None, f_ext=None):
        """Internal function for calculating the joint space bias matrix."""

        v = []
        a = []
        f = []
        C = cs.SX.zeros(n_joints)

        for i in range(0, n_joints):
            vJ = cs.mtimes(Si[i], q_dot[i])
            if i == 0:
                v.append(vJ)
                if gravity is not None:
                    ag = np.array([0., 0., 0., gravity[0], gravity[1], gravity[2]])
                    a.append(cs.mtimes(i_X_p[i], -ag))
                else:
                    a.append(cs.SX([0., 0., 0., 0., 0., 0.]))
            else:
                v.append(cs.mtimes(i_X_p[i], v[i-1]) + vJ)
                a.append(cs.mtimes(i_X_p[i], a[i-1]) + cs.mtimes(plucker.motion_cross_product(v[i]), vJ))

            f.append(cs.mtimes(Ic[i], a[i]) + cs.mtimes(plucker.force_cross_product(v[i]), cs.mtimes(Ic[i], v[i])))

        if f_ext is not None:
            f = self._apply_external_forces(f_ext, f, i_X_0)

        for i in range(n_joints-1, -1, -1):
            C[i] = cs.mtimes(Si[i].T, f[i])
            if i != 0:
                f[i-1] = f[i-1] + cs.mtimes(i_X_p[i].T, f[i])

        return C

    def get_coriolis_rnea(self, root, tip, f_ext=None):
        """Returns the Coriolis matrix as a casadi function."""

        if self.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        n_joints = self.get_n_joints(root, tip)
        q = cs.SX.sym("q", n_joints)
        q_dot = cs.SX.sym("q_dot", n_joints)
        i_X_p, Si, Ic = self._model_calculation(root, tip, q)

        v = []
        a = []
        f = []
        tau = cs.SX.zeros(n_joints)

        for i in range(0, n_joints):
            vJ = cs.mtimes(Si[i], q_dot[i])

            if i == 0:
                v.append(vJ)
                a.append(cs.SX([0., 0., 0., 0., 0., 0.]))
            else:
                v.append(cs.mtimes(i_X_p[i], v[i-1]) + vJ)
                a.append(cs.mtimes(i_X_p[i], a[i-1]) + cs.mtimes(plucker.motion_cross_product(v[i]), vJ))

            f.append(cs.mtimes(Ic[i], a[i]) + cs.mtimes(plucker.force_cross_product(v[i]), cs.mtimes(Ic[i], v[i])))

        if f_ext is not None:
            f = self._apply_external_forces(f_ext, f, i_X_0)

        for i in range(n_joints-1, -1, -1):
            tau[i] = cs.mtimes(Si[i].T, f[i])
            if i != 0:
                f[i-1] = f[i-1] + cs.mtimes(i_X_p[i].T, f[i])

        C = cs.Function("C", [q, q_dot], [tau], self.func_opts)
        return C

    def get_forward_dynamics_crba(self, root, tip, gravity=None, f_ext=None):
        """Returns the forward dynamics as a casadi function by
        solving the Lagrangian eq. of motion.  OBS! Not appropriate
        for robots with a high number of dof -> use
        get_forward_dynamics_aba().
        """
        if self.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')
        n_joints = self.get_n_joints(root, tip)
        q = cs.SX.sym("q", n_joints)
        q_dot = cs.SX.sym("q_dot", n_joints)
        tau = cs.SX.sym("tau", n_joints)
        q_ddot = cs.SX.zeros(n_joints)
        i_X_p, Si, Ic = self._model_calculation(root, tip, q)

        M = self._get_M(Ic, i_X_p, Si, n_joints, q)
        M_inv = cs.solve(M, cs.SX.eye(M.size1()))

        C = self._get_C(i_X_p, Si, Ic, q, q_dot, n_joints, gravity, f_ext)

        q_ddot = cs.mtimes(M_inv, (tau - C))
        q_ddot = cs.Function("q_ddot", [q, q_dot, tau],
                             [q_ddot], self.func_opts)

        return q_ddot

    def get_forward_dynamics_aba(self, root, tip, gravity=None, f_ext=None):
        """Returns the forward dynamics as a casadi function using the
        articulated body algorithm."""

        if self.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        n_joints = self.get_n_joints(root, tip)
        q = cs.SX.sym("q", n_joints)
        q_dot = cs.SX.sym("q_dot", n_joints)
        tau = cs.SX.sym("tau", n_joints)
        q_ddot = cs.SX.zeros(n_joints)
        i_X_p, Si, Ic = self._model_calculation(root, tip, q)

        v = []
        c = []
        pA = []
        IA = []

        u = [None]*n_joints
        U = [None]*n_joints
        d = [None]*n_joints

        for i in range(0, n_joints):
            vJ = cs.mtimes(Si[i], q_dot[i])
            if i == 0:
                v.append(vJ)
                c.append([0, 0, 0, 0, 0, 0])
            else:
                v.append(cs.mtimes(i_X_p[i], v[i-1]) + vJ)
                c.append(cs.mtimes(plucker.motion_cross_product(v[i]), vJ))
            IA.append(Ic[i])
            pA.append(cs.mtimes(plucker.force_cross_product(v[i]),
                                cs.mtimes(Ic[i], v[i])))

        if f_ext is not None:
            pA = self._apply_external_forces(f_ext, pA)

        for i in range(n_joints-1, -1, -1):
            U[i] = cs.mtimes(IA[i], Si[i])
            d[i] = cs.mtimes(Si[i].T, U[i])
            u[i] = tau[i] - cs.mtimes(Si[i].T, pA[i])
            if i != 0:
                Ia = IA[i] - ((cs.mtimes(U[i], U[i].T)/d[i]))
                pa = pA[i] + cs.mtimes(Ia, c[i]) + (cs.mtimes(U[i], u[i])/d[i])
                IA[i-1] += cs.mtimes(i_X_p[i].T, cs.mtimes(Ia, i_X_p[i]))
                pA[i-1] += cs.mtimes(i_X_p[i].T, pa)

        a = []
        for i in range(0, n_joints):
            if i == 0:
                if gravity is not None:
                    ag = np.array([0.,
                                   0.,
                                   0.,
                                   gravity[0],
                                   gravity[1],
                                   gravity[2]])
                    a_temp = (cs.mtimes(i_X_p[i], -ag) + c[i])
                else:
                    a_temp = c[i]
            else:
                a_temp = (cs.mtimes(i_X_p[i], a[i-1]) + c[i])
            q_ddot[i] = (u[i] - cs.mtimes(U[i].T, a_temp))/d[i]
            a.append(a_temp + cs.mtimes(Si[i], q_ddot[i]))

        q_ddot = cs.Function("q_ddot", [q, q_dot, tau],
                             [q_ddot], self.func_opts)
        return q_ddot

    def get_forward_kinematics(self, root, tip):
        """Returns the forward kinematics as a casadi function."""
        chain = self.robot_desc.get_chain(root, tip)
        if self.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')
        joint_list, actuated_names, upper, lower = self.get_joint_info(
            root,
            tip)
        nvar = len(actuated_names)
        T_fk = cs.SX.eye(4)
        q = cs.SX.sym("q", nvar)
        quaternion_fk = cs.SX.zeros(4)
        quaternion_fk[3] = 1.0
        dual_quaternion_fk = cs.SX.zeros(8)
        dual_quaternion_fk[3] = 1.0
        i = 0
        for joint in joint_list:
            if joint.type == "fixed":
                if joint.origin is not None:
                    xyz = joint.origin.xyz
                    rpy = joint.origin.rpy
                else:
                    # origin tag not given -> assume zero
                    xyz = [0.0]*3
                    rpy = [0.0]*3
                joint_frame = T.numpy_rpy(xyz, *rpy)
                joint_quaternion = quaternion.numpy_rpy(*rpy)
                joint_dual_quat = dual_quaternion.numpy_prismatic(
                    xyz,
                    rpy,
                    [1., 0., 0.],
                    0.)
                T_fk = cs.mtimes(T_fk, joint_frame)
                quaternion_fk = quaternion.product(
                    quaternion_fk,
                    joint_quaternion)
                dual_quaternion_fk = dual_quaternion.product(
                    dual_quaternion_fk,
                    joint_dual_quat)

            elif joint.type == "prismatic":
                if joint.axis is None:
                    axis = cs.np.array([1., 0., 0.])
                else:
                    axis = cs.np.array(joint.axis)
                # axis = (1./cs.np.linalg.norm(axis))*axis
                joint_frame = T.prismatic(joint.origin.xyz,
                                          joint.origin.rpy,
                                          joint.axis, q[i])
                joint_quaternion = quaternion.numpy_rpy(*joint.origin.rpy)
                joint_dual_quat = dual_quaternion.prismatic(
                    joint.origin.xyz,
                    joint.origin.rpy,
                    axis, q[i])
                T_fk = cs.mtimes(T_fk, joint_frame)
                quaternion_fk = quaternion.product(quaternion_fk,
                                                   joint_quaternion)
                dual_quaternion_fk = dual_quaternion.product(
                    dual_quaternion_fk,
                    joint_dual_quat)
                i += 1

            elif joint.type in ["revolute", "continuous"]:
                if joint.axis is None:
                    axis = cs.np.array([1., 0., 0.])
                else:
                    axis = cs.np.array(joint.axis)
                axis = (1./cs.np.linalg.norm(axis))*axis
                joint_frame = T.revolute(
                    joint.origin.xyz,
                    joint.origin.rpy,
                    joint.axis, q[i])
                joint_quaternion = quaternion.revolute(
                    joint.origin.xyz,
                    joint.origin.rpy,
                    axis, q[i])
                joint_dual_quat = dual_quaternion.revolute(
                    joint.origin.xyz,
                    joint.origin.rpy,
                    axis, q[i])
                T_fk = cs.mtimes(T_fk, joint_frame)
                quaternion_fk = quaternion.product(
                    quaternion_fk,
                    joint_quaternion)
                dual_quaternion_fk = dual_quaternion.product(
                    dual_quaternion_fk,
                    joint_dual_quat)
                i += 1
        T_fk = cs.Function("T_fk", [q], [T_fk], self.func_opts)
        quaternion_fk = cs.Function("quaternion_fk",
                                    [q], [quaternion_fk], self.func_opts)
        dual_quaternion_fk = cs.Function("dual_quaternion_fk",
                                         [q], [dual_quaternion_fk], self.func_opts)

        return {
            "joint_names": actuated_names,
            "upper": upper,
            "lower": lower,
            "joint_list": joint_list,
            "q": q,
            "quaternion_fk": quaternion_fk,
            "dual_quaternion_fk": dual_quaternion_fk,
            "T_fk": T_fk
        }

    def get_forces_bottom_up(self, root, tip, f_root, gravity=None):
        """
        Calculates the generalized body forces in bottom up manner.

        Parameters:
            root (string): Name of the base link.
            tip (string): Name of the endeffector.

        Returns:
            generalized_body_forces (set of doubles): A set of generalized body forces for each body in the kinematic chain as a CasADi function.
        """
        if self.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        # Get joint count for the robot.
        n_joints = self.get_n_joints(root, tip)

        # Declare symbolic verctor of angles q
        q = cs.SX.sym("q", n_joints)

        # Declare symbolic verctor of angular accelerations q_dot
        q_dot = cs.SX.sym("q_dot", n_joints)

        # Declare symbolic verctor of angular velocities q_ddot
        q_ddot = cs.SX.sym("q_ddot", n_joints)

        # Get Plücker transformation matrices i_X_p, joint motion subspaces Si, inertia matrices Ic
        i_X_p, Si, Ic = self._model_calculation(root, tip, q)

        velocities = []
        accelerations = []
        body_inertial_forces = []
        generalized_body_forces = []

        velocities.append(cs.mtimes(Si[0], q_dot[0]))

        if gravity is not None:
            ag = np.array([0., 0., 0., gravity[0], gravity[1], gravity[2]])
            accelerations.append(cs.mtimes(i_X_p[0], -ag) + cs.mtimes(Si[0], q_ddot[0]))
        else:
            accelerations.append(cs.mtimes(Si[0], q_ddot[0]))

        body_inertial_forces.append(
            cs.mtimes(Ic[0], accelerations[0])
            +
            cs.mtimes(plucker.force_cross_product(velocities[0]), cs.mtimes(Ic[0], velocities[0])))
        
        # * Hier ist keine Plücker-Transformation notwendig:
        # // generalized_body_forces.append(cs.mtimes(i_X_p[0], f_root))
        # * Begründung: Betrachte einen Roboter mit 3 Körperelementen, 3 Gelenken und einem Endeffektor.
        # * Annahme: Es gibt keine externen Kräfte.
        # * f_2 = f_2^B - (f_2^ext) + Sum[(f_j), j sind alle Nachfolger von 2] = f_2^B -----> tau_2 = S_2^T * f_2
        # * f_1 = f_1^B + 1^X_2 * f_2 -----> tau_1 = S_1^T * f_1
        # * f_0 = f_0^B + 0^X_1 * f_1 -----> tau_0 = S_0^T * f_0
        # * Die Berechnung geht Zig-Zagweise von oben nach unten. Drehe die Richtung um.
        # * Du siehst jetzt, dass du als erstes tau_0 = S_0^T * f_0 berechnest.
        # * Danach: f_0 = f_0^B + 0^X_1 * f_1 => f_1 = inv(0^X_1)*(f_0 - f_0^B) und so weiter.
        generalized_body_forces.append(f_root)

        for i in range(1, n_joints):
            vJ = cs.mtimes(Si[i], q_dot[i])
            velocities.append(cs.mtimes(i_X_p[i], velocities[i - 1]) + vJ)
            accelerations.append(
                cs.mtimes(i_X_p[i], accelerations[i-1])
                + cs.mtimes(Si[i], q_ddot[i])
                + cs.mtimes(plucker.motion_cross_product(velocities[i]), vJ))

            body_inertial_forces.append(
                cs.mtimes(Ic[i], accelerations[i])
                +
                cs.mtimes(plucker.force_cross_product(velocities[i]), cs.mtimes(Ic[i], velocities[i])))
            
            generalized_body_forces.append(
                cs.mtimes(
                    cs.inv_minor(i_X_p[i].T),
                    generalized_body_forces[i - 1] - body_inertial_forces[i - 1]))

        # Declare the symbolic function with input [q, q_dot, q_ddot] and the output generalized_body_forces.
        generalized_body_forces = cs.Function("forces_bottom_up", [q, q_dot, q_ddot], generalized_body_forces, self.func_opts)
        body_inertial_forces = cs.Function("body_intertials", [q, q_dot, q_ddot], body_inertial_forces, self.func_opts)

        return generalized_body_forces, body_inertial_forces

    def get_inverse_dynamics_rnea_bottom_up(self, root, tip):
        """Returns the inverse dynamics as a casadi function."""
        if self.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        n_joints = self.get_n_joints(root, tip)
        q = cs.SX.sym("q", n_joints)
        q_dot = cs.SX.sym("q_dot", n_joints)
        q_ddot = cs.SX.sym("q_ddot", n_joints)
        i_X_p, Si, _ = self._model_calculation(root, tip, q)
        # print("\n" + i_X_p[0] + "\n")

        f_root = cs.SX.sym("f_root", 6)
        f = []
        tau_bu = cs.SX.zeros(n_joints)

        f.append(f_root)

        # if f_ext is not None:
        # f[0] = self._apply_external_forces_bottom_up(f[0], f, i_X_p)

        for i in range(0, n_joints - 1):
            tau_bu[i] = cs.mtimes(Si[i].T, f[i])
            f.append(cs.mtimes(i_X_p[i], f[i]))

        tau_bu = cs.Function("C_bu", [q, q_dot, q_ddot, f_root], [tau_bu], self.func_opts)
        # force_first_joint = cs.Function("f_f_j", [q, q_dot, q_ddot], [f[0]], self.func_opts)
        # force_base = cs.Function("f_b", [q, q_dot, q_ddot], [cs.mtimes(i_X_p[0].T, f[0])], self.func_opts)
        return tau_bu  # , force_first_joint, force_base

    def get_inverse_dynamics_rnea_bottom_up_f(self, root, tip, forces):
        """Returns the inverse dynamics as a casadi function."""
        if self.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        n_joints = self.get_n_joints(root, tip)
        q = cs.SX.sym("q", n_joints)

        _, Si, _ = self._model_calculation(root, tip, q)

        tau_bu = cs.SX.zeros(n_joints)

        for i in range(0, n_joints):
            print("Si[{}]: {}".format(i, cs.DM(Si[i])))
            print("The used force is: ", forces[i])
            tau_bu[i] = cs.mtimes(Si[i].T, forces[i])
            print("Intermediate result for tau_bu[{}]: {}\n".format(i, tau_bu[i]))

        tau_bu = cs.Function("C_bu", [q], [tau_bu], self.func_opts)
        return tau_bu

    def get_model_calculation(self, root, tip):
        n_joints = self.get_n_joints(root, tip)
        q = cs.SX.sym("q", n_joints)
        i_X_p, Si, Ic = self._model_calculation(root, tip, q)

        i_X_p = cs.Function("i_X_p", [q], [i_X_p[0], i_X_p[1], i_X_p[2], i_X_p[3], i_X_p[4], i_X_p[5]], self.func_opts)
        return i_X_p

    def _model_calculation_bottom_up(self, root, tip, q):
        """Calculates and returns model information needed in the
        dynamics algorithms caluculations, i.e transforms, joint space
        and inertia."""
        if self.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        chain = self.robot_desc.get_chain(root, tip)
        spatial_inertias = []
        _0_X_i = []
        p_X_i = []
        Sis = []
        prev_joint = None
        n_actuated = 0
        i = 0

        for item in chain:
            if item in self.robot_desc.joint_map:
                joint = self.robot_desc.joint_map[item]

                if joint.type == "fixed":
                    if prev_joint == "fixed":
                        XT_prev = cs.mtimes(
                            plucker.XT(joint.origin.xyz, joint.origin.rpy),
                            XT_prev)
                    else:
                        XT_prev = plucker.XT(
                            joint.origin.xyz,
                            joint.origin.rpy)
                    inertia_transform = XT_prev
                    prev_inertia = spatial_inertia

                elif joint.type in ["revolute", "continuous"]:
                    if n_actuated != 0:
                        spatial_inertias.append(spatial_inertia)
                    n_actuated += 1

                    XJT = plucker.XJT_revolute(
                        joint.origin.xyz,
                        joint.origin.rpy,
                        joint.axis,
                        q[i])
                    if prev_joint == "fixed":
                        XJT = cs.mtimes(XJT, XT_prev)
                    Si = cs.SX([
                        joint.axis[0],
                        joint.axis[1],
                        joint.axis[2],
                        0,
                        0,
                        0])
                    p_X_i.append(XJT)
                    Sis.append(Si)
                    i += 1

                prev_joint = joint.type

            if item in self.robot_desc.link_map:
                link = self.robot_desc.link_map[item]

                if link.inertial is None:
                    spatial_inertia = np.zeros((6, 6))
                else:
                    I = link.inertial.inertia
                    spatial_inertia = plucker.spatial_inertia_matrix_IO(
                        I.ixx,
                        I.ixy,
                        I.ixz,
                        I.iyy,
                        I.iyz,
                        I.izz,
                        link.inertial.mass,
                        link.inertial.origin.xyz)

                if prev_joint == "fixed":
                    spatial_inertia = prev_inertia + cs.mtimes(
                        inertia_transform.T,
                        cs.mtimes(spatial_inertia, inertia_transform))

                if link.name == tip:
                    spatial_inertias.append(spatial_inertia)

        return p_X_i, Sis, spatial_inertias
