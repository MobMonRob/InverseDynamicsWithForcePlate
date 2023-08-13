import casadi as cs
import numpy as np
import urdf2casadi.geometry.plucker as plucker
from urdf2casadi.urdfparser import URDFparser


class Inverse_dynamics_rnea(object):
    def __init__(self, urdfparser: URDFparser):
        self.urdfparser: URDFparser = urdfparser

    # Von der Dame, von mir modifiziert
    def get_inverse_dynamics_rnea(self, root, tip,
                                  gravity=None, f_ext=None):
        """Returns the inverse dynamics as a casadi function."""
        if self.urdfparser.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        n_joints = self.urdfparser.get_n_joints(root, tip)
        q = cs.SX.sym("q", n_joints)
        q_dot = cs.SX.sym("q_dot", n_joints)
        q_ddot = cs.SX.sym("q_ddot", n_joints)
        i_X_p, Si, Ic = self.urdfparser._model_calculation(root, tip, q)

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
            f = self.urdfparser._apply_external_forces(f_ext, f, i_X_p)

        for i in range(n_joints-1, -1, -1):
            tau[i] = cs.mtimes(Si[i].T, f[i])
            if i != 0:
                f[i-1] = f[i-1] + cs.mtimes(i_X_p[i].T, f[i])

        tau = cs.Function("C", [q, q_dot, q_ddot], [tau], self.urdfparser.func_opts)
        forces = cs.Function("forces", [q, q_dot, q_ddot], [f[0], f[1], f[2], f[3], f[4], f[5]], self.urdfparser.func_opts)

        return tau, forces

    # RNEA von mir
    def get_inverse_dynamics_rnea_bottom_up_f(self, root, tip):
        """Returns the inverse dynamics as a casadi function."""
        if self.urdfparser.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        n_joints = self.urdfparser.get_n_joints(root, tip)
        q = cs.SX.sym("q", n_joints)
        spatial_forces = [cs.SX.sym(f"spatial_force{i}", n_joints) for i in range(n_joints)]

        _, Si, _ = self.urdfparser._model_calculation(root, tip, q)

        tau_bu = cs.SX.zeros(n_joints)

        for i in range(0, n_joints):
            tau_bu[i] = cs.mtimes(Si[i].T, spatial_forces[i])

        tau_bu = cs.Function("C_bu", [q] + spatial_forces, [tau_bu], self.urdfparser.func_opts)
        return tau_bu

    # Von mir, berechnet Kräfte bottom-up, wenn die unterste spatial Kraft gegeben ist
    def get_forces_bottom_up(self, root, tip, gravity=None):
        """
        Calculates the generalized body forces in bottom up manner.

        Parameters:
            root (string): Name of the base link.
            tip (string): Name of the endeffector.

        Returns:
            generalized_body_forces (set of doubles): A set of generalized body forces for each body in the kinematic chain as a CasADi function.
        """
        if self.urdfparser.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        # Get joint count for the robot.
        n_joints = self.urdfparser.get_n_joints(root, tip)

        # Declare symbolic verctor of angles q
        q = cs.SX.sym("q", n_joints)

        # Declare symbolic verctor of angular accelerations q_dot
        q_dot = cs.SX.sym("q_dot", n_joints)

        # Declare symbolic verctor of angular velocities q_ddot
        q_ddot = cs.SX.sym("q_ddot", n_joints)

        f_spatial_joint_0 = cs.SX.sym("f_root", n_joints)

        # Get Plücker transformation matrices i_X_p, joint motion subspaces Si, inertia matrices Ic
        i_X_p, Si, Ic = self.urdfparser._model_calculation(root, tip, q)

        velocities = []
        accelerations = []
        body_inertial_forces = []
        joint_spatial_forces = []

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
        joint_spatial_forces.append(f_spatial_joint_0)

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

            joint_spatial_forces.append(
                cs.mtimes(
                    cs.inv_minor(i_X_p[i].T),
                    joint_spatial_forces[i - 1] - body_inertial_forces[i - 1]))

        # Declare the symbolic function with input [q, q_dot, q_ddot] and the output joint_spatial_forces_func.
        joint_spatial_forces_func = cs.Function("forces_bottom_up", [q, q_dot, q_ddot, f_spatial_joint_0], joint_spatial_forces, self.urdfparser.func_opts)
        body_inertial_forces_func = cs.Function("body_intertial_forces", [q, q_dot, q_ddot], body_inertial_forces, self.urdfparser.func_opts)

        return joint_spatial_forces_func, body_inertial_forces_func

    # Zeigt die Pluecker-Matrizen (Transformationen) an. Ich denke, die Methode brauche ich nicht.
    def get_model_calculation(self, root, tip):
        n_joints = self.urdfparser.get_n_joints(root, tip)
        q = cs.SX.sym("q", n_joints)
        i_X_p, Si, Ic = self.urdfparser._model_calculation(root, tip, q)

        i_X_p = cs.Function("i_X_p", [q], [i_X_p[0], i_X_p[1], i_X_p[2], i_X_p[3], i_X_p[4], i_X_p[5]], self.urdfparser.func_opts)
        return i_X_p

    # Berechnet Hilfsvariablen für RNEA. Ich denke, die Methode bracuhe ich nicht, da ich für meinen RNEA die Standardvariante _model_calculation von der Dame benutzen kann und bis jetzt auch benutzt habe.
    def _model_calculation_bottom_up(self, root, tip, q):
        """Calculates and returns model information needed in the
        dynamics algorithms caluculations, i.e transforms, joint space
        and inertia."""
        if self.urdfparser.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        chain = self.urdfparser.robot_desc.get_chain(root, tip)
        spatial_inertias = []
        _0_X_i = []
        p_X_i = []
        Sis = []
        prev_joint = None
        n_actuated = 0
        i = 0

        for item in chain:
            if item in self.urdfparser.robot_desc.joint_map:
                joint = self.urdfparser.robot_desc.joint_map[item]

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

            if item in self.urdfparser.robot_desc.link_map:
                link = self.urdfparser.robot_desc.link_map[item]

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
