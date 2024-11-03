import numpy as np
from scipy.spatial.transform import Rotation


class SE3Control(object):
    """

    """

    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass = quad_params['mass']  # kg
        self.Ixx = quad_params['Ixx']  # kg*m^2
        self.Iyy = quad_params['Iyy']  # kg*m^2
        self.Izz = quad_params['Izz']  # kg*m^2
        self.arm_length = quad_params['arm_length']  # meters
        self.rotor_speed_min = quad_params['rotor_speed_min']  # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max']  # rad/s
        self.k_thrust = quad_params['k_thrust']  # N/(rad/s)**2
        self.k_drag = quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(
            np.array([self.Ixx, self.Iyy, self.Izz]))  # kg*m^2
        self.g = 9.81  # m/s^2

        # STUDENT CODE HERE

    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # STUDENT CODE HERE
        # geometric controller
        # compute F_des
        # Kd = np.diag([8, 8, 5])  # TODO
        # Kp = np.diag([7, 7, 5])  # TODO
        # Kd = np.diag([4.5, 4.5, 4])  # TODO
        # Kp = np.diag([6.8, 6.8, 5])  # TODO
        Kd = np.diag([4.5, 4.5, 4])  # TODO
        Kp = np.diag([6.8, 6.8, 5])  # TODO
        rddt = flat_output["x_ddot"] - Kd @ (state["v"] - flat_output["x_dot"]) - Kp @ (state["x"] - flat_output["x"])
        # rddt = np.clip(flat_output["x_ddot"] - Kd @ (state["v"]-flat_output["x_dot"]) - Kp @ (state["x"] - flat_output["x"])
        #               , np.array([-13.5,-7,-100]), np.array([7,7,100]))
        F_des = self.mass * rddt + np.array([0, 0, self.mass * self.g])

        # calculate rotation matrix
        i, j, k, w = state["q"]
        R = np.array([
            [1 - 2 * j ** 2 - 2 * k ** 2, 2 * i *
                j - 2 * k * w, 2 * i * k + 2 * j * w],
            [2 * i * j + 2 * k * w, 1 - 2 * i ** 2 -
                2 * k ** 2, 2 * j * k - 2 * i * w],
            [2 * i * k - 2 * j * w, 2 * j * k + 2 *
                i * w, 1 - 2 * i ** 2 - 2 * j ** 2]
        ])

        # compute u1
        u1 = (R @ np.array([0, 0, 1])).T @ F_des

        # compute R_des
        b3_des = F_des / np.linalg.norm(F_des)
        a_psi = np.array([np.cos(flat_output["yaw"]),
                         np.sin(flat_output["yaw"]), 0])
        b2_des = np.cross(b3_des, a_psi) / np.linalg.norm(np.cross(b3_des, a_psi))
        R_des = np.column_stack([np.cross(b2_des, b3_des), b2_des, b3_des])

        # compute e_R, e_omega
        e_omega = state["w"]
        mat1 = (R_des.T @ R - R.T @ R_des)/2
        e_R = np.array([mat1[2, 1], -mat1[2, 0], mat1[1, 0]])

        # compute u2
        # K_R = np.diag([800, 800, 800])  # TODO
        # K_omega = np.diag([80, 80, 80])  # TODO
        # K_R = np.diag([250, 250, 60])  # TODO
        # K_omega = np.diag([20, 20, 30])  # TODO
        # K_R = np.diag([280, 280, 90])  # TODO
        # K_omega = np.diag([20, 20, 15])  # TODO
        K_R = np.diag([250, 250, 60])  # TODO
        K_omega = np.diag([20, 20, 30])  # TODO
        u2 = self.inertia @ (-K_R @ e_R - K_omega @ e_omega)

        # compute output numbers
        cmd_thrust = u1
        cmd_moment = u2
        l = self.arm_length
        kf = self.k_thrust
        km = self.k_drag
        gamma = km / kf
        mat2 = np.array([
            [1, 1, 1, 1],
            [0, l, 0, -l],
            [-l, 0, l, 0],
            [gamma, -gamma, gamma, -gamma]
        ])
        u = np.array([u1, u2[0], u2[1], u2[2]])
        F1, F2, F3, F4 = np.linalg.solve(mat2, u)
        omega1 = np.sign(F1) * np.sqrt(np.clip(F1, 0, 1e6)/kf)
        omega2 = np.sign(F2) * np.sqrt(np.clip(F2, 0, 1e6)/kf)
        omega3 = np.sign(F3) * np.sqrt(np.clip(F3, 0, 1e6)/kf)
        omega4 = np.sign(F4) * np.sqrt(np.clip(F4, 0, 1e6)/kf)
        cmd_motor_speeds = np.array([omega1, omega2, omega3, omega4])
        tr = np.trace(R_des)
        S = np.sqrt(tr + 1.0) * 2
        cmd_q[0] = (R_des[2, 1] - R_des[1, 2]) / S
        cmd_q[1] = (R_des[0, 2] - R_des[2, 0]) / S
        cmd_q[2] = (R_des[1, 0] - R_des[0, 1]) / S
        cmd_q[3] = 0.25 * S

        control_input = {'cmd_motor_speeds': cmd_motor_speeds,
                         'cmd_thrust': cmd_thrust,
                         'cmd_moment': cmd_moment,
                         'cmd_q': cmd_q}
        return control_input
