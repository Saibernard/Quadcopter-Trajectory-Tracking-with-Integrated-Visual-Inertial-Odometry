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
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2

        # STUDENT CODE HERE
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz]))  # kg*m^2
        self.g = 9.81  # m/s^2
        self.kp_pos = np.diag(np.array([9, 9, 11]))  # 9,9,11
        self.kd_pos = np.diag(np.array([5, 5, 8]))  # 5,5,8
        self.gamma = self.k_drag / self.k_thrust
        self.kp_att = np.diag(np.array([1500, 1500, 180]))
        self.kd_att = np.diag(np.array([120, 120, 80]))

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
        # Extracting variables from input dictionaries for clarity
        x_ddot = flat_output["x_ddot"].reshape(3, 1)
        v_state = state["v"].reshape(3, 1)
        x_dot = flat_output["x_dot"].reshape(3, 1)
        x_state = state["x"].reshape(3, 1)
        x = flat_output["x"].reshape(3, 1)

        # Calculating the differences between state and expected state
        v_diff = v_state - x_dot
        x_diff = x_state - x

        # Applying the control gains
        damping_term = self.kd_pos @ v_diff
        proportional_term = self.kp_pos @ x_diff

        # Final command acceleration calculation
        cmd_acc = x_ddot - damping_term - proportional_term

        # Define gravitational force array
        gravitational_force = np.array([[0], [0], [self.mass * self.g]])

        # Compute desired force
        F_des = self.mass * cmd_acc + gravitational_force

        # Obtain the rotation matrix from the quaternion state
        R = Rotation.from_quat(state["q"]).as_matrix()

        # Compute the third body-fixed axis
        b_3 = R @ np.array([[0], [0], [1]])

        # Compute the thrust scalar
        u1 = b_3.T[0] @ F_des

        # Compute the desired third body-fixed axis
        b3_des = F_des / np.linalg.norm(F_des)

        # Extract yaw and compute its direction vector
        yaw = flat_output["yaw"]
        yaw_vector = np.array([
            [np.cos(yaw)],
            [np.sin(yaw)],
            [0]
        ])

        # Compute cross product between the desired third body-fixed axis and yaw direction vector
        cross_product = np.cross(b3_des.reshape(1, 3), yaw_vector.reshape(1, 3)).squeeze()

        # Normalize the cross product to get the desired second body-fixed axis
        b2_des = cross_product / np.linalg.norm(cross_product)

        # Compute a new cross product between b2_des and b3_des
        new_cross_product = np.cross(b2_des, b3_des.reshape(1, 3))

        # Form the desired rotation matrix
        R_des = np.column_stack((new_cross_product.squeeze(), b2_des, b3_des.squeeze()))

        hat = np.dot(R_des.T, R) - np.dot(R.T, R_des)
        v_op = hat[[2, 0, 1], [1, 2, 0]].reshape(-1, 1)

        e_R = (1/2) * v_op
        e_omega = state["w"].reshape(-1, 1)

        # Calculate the proportional control contribution
        proportional_control = self.kp_att @ e_R

        # Calculate the derivative control contribution
        derivative_control = self.kd_att @ e_omega

        # Calculate the total control effort
        control_effort = proportional_control + derivative_control

        # Negate the control effort
        negative_control_effort = -control_effort

        # Calculate the desired moment
        u2 = self.inertia @ negative_control_effort

        u = np.concatenate((np.array([u1]), u2), axis=0)

        arm_mat = np.array([[1, 1, 1, 1],
                            [0, self.arm_length, 0, -self.arm_length],
                            [-self.arm_length, 0, self.arm_length, 0],
                            [self.gamma, -self.gamma, self.gamma, -self.gamma]])

        motor_sq = np.linalg.inv(arm_mat) @ u

        # Compute the absolute value of the motor square
        absolute_motor_sq = np.abs(motor_sq)

        # Divide the absolute value of the motor square by the thrust coefficient
        motor_speed_ratio = absolute_motor_sq / self.k_thrust

        # Compute the square root of the motor speed ratio
        sqrt_motor_speed_ratio = np.sqrt(motor_speed_ratio)

        # Determine the sign of the motor square
        sign_motor_sq = np.sign(motor_sq)

        # Multiply the square root of the motor speed ratio by the sign of the motor square
        # This gives the commanded motor speeds
        cmd_motor_speeds = sqrt_motor_speed_ratio * sign_motor_sq

        cmd_motor_speeds = cmd_motor_speeds.reshape(4, )
        cmd_thrust = u1
        cmd_moment = u2
        cmd_q = Rotation.from_matrix(R_des).as_quat()

        control_input = {'cmd_motor_speeds': cmd_motor_speeds,
                         'cmd_thrust': cmd_thrust,
                         'cmd_moment': cmd_moment,
                         'cmd_q': cmd_q}
        return control_input

