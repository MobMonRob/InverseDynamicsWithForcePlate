import casadi as cs
import numpy as np
import urdf2casadi.geometry.plucker as plucker
from urdf2casadi.urdfparser import URDFparser


class Inverse_dynamics_rnea(object):
    def __init__(self, urdfparser: URDFparser):
        self.urdfparser: URDFparser = urdfparser

    #! TODO: root, tip übergeben in Konstruktor, n_joints als public variable.

    def get_n_joints(self, root: str, tip: str) -> int:
        return self.urdfparser.get_n_joints(root, tip)

    # RNEA: Von der Dame, von mir modifiziert
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
        forces = cs.Function("forces", [q, q_dot, q_ddot], [f[i] for i in range(n_joints)], self.urdfparser.func_opts)

        return tau, forces

    # BURNEA: joint spatial forces -> taus (mz)
    def get_inverse_dynamics_rnea_bottom_up_f(self, root, tip):
        """Returns the calculation of the tau values from the spatial forces as a casadi function."""
        if self.urdfparser.robot_desc is None:
            raise ValueError('Robot description not loaded from urdf')

        n_joints = self.urdfparser.get_n_joints(root, tip)
        q = cs.SX.sym("q", n_joints)

        # 6 is the dimension of a spatial force vector.
        spatial_forces = [cs.SX.sym(f"spatial_force{i}", 6) for i in range(n_joints)]

        _, Si, _ = self.urdfparser._model_calculation(root, tip, q)

        tau_bu = cs.SX.zeros(n_joints)

        for i in range(0, n_joints):
            tau_bu[i] = cs.mtimes(Si[i].T, spatial_forces[i])

        tau_bu = cs.Function("C_bu", [q] + spatial_forces, [tau_bu], self.urdfparser.func_opts)
        return tau_bu

    # BURNEA: joint spatial forces
    def get_forces_bottom_up(self, root, tip, gravity=None):
        """
        Calculates the joint spatial forces in a bottom up manner.
        External forces are not considered.
        Spatial forces are represented as such a tuple: (Mx, My, Mz, Fx, Fy, Fz)

        Parameters:
            - root (string): Name of the base link.
            - tip (string): Name of the endeffector.
            - gravity (None or [0, 0, -9.81]): Gravity constant.

        Returns:
            joint_spatial_forces_func: A CasADi function which calculates the spatial forces of each joint at a point in time.
            The parameters for joint_spatial_forces_func are:
            - q (6-tuple of float): Position of each joint.
            - q_dot (6-tuple of float): Velocity of each joint.
            - q_ddot (6-tuple of float): Acceleration of each joint.
            - f_spatial_body_0 (6-tuple of float): Spatial force within the lowest body.
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

        # 6 is the dimension of a spatial force vector.
        f_spatial_body_0 = cs.SX.sym("f_root", 6)

        # Get Plücker transformation matrices i_X_p, joint motion subspaces Si, inertia matrices Ic
        i_X_p, Si, Ic = self.urdfparser._model_calculation(root, tip, q)

        body_velocities = [None] * n_joints
        body_accelerations = [None] * n_joints
        body_inertial_forces = [None] * n_joints
        joint_spatial_forces = [None] * n_joints

        body_velocities[0] = cs.mtimes(Si[0], q_dot[0])

        if gravity is not None:
            ag = np.array([0., 0., 0., gravity[0], gravity[1], gravity[2]])
            body_accelerations[0] = cs.mtimes(i_X_p[0], -ag) + cs.mtimes(Si[0], q_ddot[0])
        else:
            body_accelerations[0] = cs.mtimes(Si[0], q_ddot[0])

        body_inertial_forces[0] = (
            cs.mtimes(Ic[0], body_accelerations[0])
            +
            cs.mtimes(plucker.force_cross_product(body_velocities[0]), cs.mtimes(Ic[0], body_velocities[0])))

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

        joint_spatial_forces[0] = f_spatial_body_0

        # In Theorie fangen Bodies bei 0 und Gelenke bei 1 an.
        # In Implementierung fangen Bodies und Gelenke bei 0 an.
        # i_X_p[i] transformiert die Geschwindigkeiten und Beschleunigungen von Body i-1 auf Body i in Implementierung und Theorie. Dabei muss immer die Kraft von dem unteren Body genommen werden und die Inertia von dem unteren Body abgezogen.
        # i_X_p[i].T transformiert die Kraft von Body i auf Body i-1.
        # cs.inv_minor(i_X_p[i].T) transformiert die Kraft von von Body i-1 auf Body 1.
        # In Theorie und Implementierung ist der Joint i der vor dem Body i.
        # Body_i ist der Body nach Joint_i. Bodies vor Joint_0 werden ignoriert durch urdf2casadi. Daher zur Not unten einen Dummy Joint setzen. Weil man aber als root einen Body angeben muss, braucht man dann davor auch noch einen Dummy Body.

        for i in range(1, n_joints):
            joint_velocity_i = cs.mtimes(Si[i], q_dot[i])

            body_velocities[i] = (cs.mtimes(i_X_p[i], body_velocities[i - 1]) + joint_velocity_i)

            body_accelerations[i] = (
                cs.mtimes(i_X_p[i], body_accelerations[i-1])
                + cs.mtimes(Si[i], q_ddot[i])
                + cs.mtimes(plucker.motion_cross_product(body_velocities[i]), joint_velocity_i))

            body_inertial_forces[i] = (
                cs.mtimes(Ic[i], body_accelerations[i])
                +
                cs.mtimes(plucker.force_cross_product(body_velocities[i]), cs.mtimes(Ic[i], body_velocities[i])))

            joint_spatial_forces[i] = (
                cs.mtimes(
                    cs.inv_minor(i_X_p[i].T),
                    joint_spatial_forces[i - 1] - body_inertial_forces[i - 1]))

        # Declare the symbolic function with input [q, q_dot, q_ddot] and the output joint_spatial_forces_func.
        joint_spatial_forces_func = cs.Function("forces_bottom_up", [q, q_dot, q_ddot, f_spatial_body_0], joint_spatial_forces, self.urdfparser.func_opts)
        body_inertial_forces_func = cs.Function("body_intertial_forces", [q, q_dot, q_ddot], body_inertial_forces, self.urdfparser.func_opts)

        #! Evtl. keine Casadi Matrizen aus dieser Funktion zurück geben lassen, sondern direkt umwandeln.

        return joint_spatial_forces_func, body_inertial_forces_func

    # Zeigt die Pluecker-Matrizen (Transformationen) an. Ich denke, die Methode brauche ich nicht.
    def get_model_calculation(self, root, tip):
        n_joints = self.urdfparser.get_n_joints(root, tip)

        print(f"root: {root}")

        q = cs.SX.sym("q", n_joints)
        i_X_p, Si, Ic = self.urdfparser._model_calculation(root, tip, q)

        i_X_p = cs.Function("i_X_p", [q], [i_X_p[i] for i in range(n_joints)], self.urdfparser.func_opts)
        Ic = cs.Function("Ic", [q], [Ic[i] for i in range(n_joints)], self.urdfparser.func_opts)
        return i_X_p, Ic
