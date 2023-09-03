# %% Imports

import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from scipy.spatial.transform import Rotation


# %% Functions

def nominal_state_update(nominal_state, w_m, a_m, dt):
    """
    function to perform the nominal state update

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                    all elements are 3x1 vectors except for q which is a Rotation object
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :return: new tuple containing the updated state
    """
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    # YOUR CODE HERE
    new_p = np.zeros((3, 1))
    new_v = np.zeros((3, 1))
    new_q = Rotation.identity()

    # Convert quaternion to rotation matrix
    R = Rotation.as_matrix(q)

    # Calculate change in position and velocity due to acceleration
    acceleration_effect = R @ (a_m - a_b) + g
    position_change = v * dt + 0.5 * acceleration_effect * dt ** 2
    velocity_change = acceleration_effect * dt

    # Update position and velocity
    new_p = p + position_change
    new_v = v + velocity_change

    # Calculate new rotation due to angular velocity
    angular_velocity_effect = (w_m - w_b) * dt
    q_new = Rotation.from_rotvec(angular_velocity_effect.flatten())

    # Update rotation
    new_q = q * q_new

    return new_p, new_v, new_q, a_b, w_b, g


def error_covariance_update(nominal_state, error_state_covariance, w_m, a_m, dt,
                            accelerometer_noise_density, gyroscope_noise_density,
                            accelerometer_random_walk, gyroscope_random_walk):
    """
    Function to update the error state covariance matrix

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :param accelerometer_noise_density: standard deviation of accelerometer noise
    :param gyroscope_noise_density: standard deviation of gyro noise
    :param accelerometer_random_walk: accelerometer random walk rate
    :param gyroscope_random_walk: gyro random walk rate
    :return:
    """

    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    # YOUR CODE HERE
    Fx = np.zeros((18, 18))
    Qi = np.zeros((12, 12))
    Fi = np.zeros((18, 12))
    R = Rotation.as_matrix(q)
    
    I = np.identity(3)

    Fx[0:3, 0:3] = I
    Fx[0:3, 3:6] = I * dt
    Fx[3:6, 3:6] = I
    a_diff_vec = a_m - a_b
    a_skew = np.array([[0, -a_diff_vec[2], a_diff_vec[1]],
                       [a_diff_vec[2], 0, -a_diff_vec[0]],
                       [-a_diff_vec[1], a_diff_vec[0], 0]], dtype=object)

    # I = np.array([[1,0,0], [0,1,0], [0,0,1]])

    # Updating Fx matrix
    Fx[3:6, 6:9] = np.negative(np.dot(R, a_skew) * dt)
    Fx[3:6, 9:12] = np.negative(R) * dt
    Fx[3:6, 15:] = I * dt
    rot_vec = np.multiply(w_m - w_b, dt)
    rot_f_vec = Rotation.from_rotvec(np.ravel(rot_vec))
    Fx[6:9, 6:9] = rot_f_vec.as_matrix().transpose()
    Fx[6:9, 12:15] = np.negative(I) * dt
    Fx[9:12, 9:12] = I
    Fx[12:15, 12:15] = I
    Fx[15:, 15:] = I

    # Updating Fi matrix
    Fi[3:6, 0:3] = I
    Fi[6:9, 3:6] = I
    Fi[9:12, 6:9] = I
    Fi[12:15, 9:] = I

    velocity_i = pow(accelerometer_noise_density,2) * pow(dt,2) * I
    theta_i = pow(gyroscope_noise_density, 2) * pow(dt, 2) * I
    Acc_i = pow(accelerometer_random_walk, 2) * dt * I
    omega_i = pow(gyroscope_random_walk, 2) * dt * I
    Qi[0:3, 0:3] = velocity_i
    Qi[3:6, 3:6] = theta_i
    Qi[6:9, 6:9] = Acc_i
    Qi[9:, 9:] = omega_i

    P = (Fx @ error_state_covariance @ Fx.T) + (Fi @ Qi @ Fi.T)
    return P

    # return an 18x18 covariance matrix
    # return np.identity(18)


def measurement_update_step(nominal_state, error_state_covariance, uv, Pw, error_threshold, Q):
    """
    Function to update the nominal state and the error state covariance matrix based on a single
    observed image measurement uv, which is a projection of Pw.

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param uv: 2x1 vector of image measurements
    :param Pw: 3x1 vector world coordinate
    :param error_threshold: inlier threshold
    :param Q: 2x2 image covariance matrix
    :return: new_state_tuple, new error state covariance matrix
    """

    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    # YOUR CODE HERE - compute the innovation next state, next error_state covariance
    innovation = np.zeros((2, 1))
    ni = 0
    R = Rotation.as_matrix(q)
    # Calculate Pc
    Pc = np.dot(R.T, (Pw - p))

    # Compute innovation
    innovation = np.subtract(uv, (Pc[0:2] / Pc[2]).reshape(-1, 1))

    if norm(innovation) < error_threshold:
        # Compute z_t
        z_t = np.divide(Pc[0:2], Pc[2]) + ni

        # Compute d_z_c
        d_z_c = np.multiply(1 / Pc[2], np.array([[1, 0, -float(z_t[0])],
                                                 [0, 1, -float(z_t[1])]], dtype=float))

        # Compute P_c_0
        P_c_0 = np.dot(R.T, (Pw - p))

        # Compute d_P_theta
        d_P_theta = np.array([[0, -float(P_c_0[2]), float(P_c_0[1])],
                              [float(P_c_0[2]), 0, -float(P_c_0[0])],
                              [-float(P_c_0[1]), float(P_c_0[0]), 0]], dtype=float)

        # Compute d_P_p
        d_P_p = - R.T

        # Compute d_z_theta and d_z_p
        d_z_theta = np.dot(d_z_c, d_P_theta)
        d_z_p = np.dot(d_z_c, d_P_p)

        # Initialize H matrix
        H = np.zeros((2, 18))

        # Assign appropriate sub-matrices
        H[0:2, 0:3] = d_z_p
        H[0:2, 6:9] = d_z_theta

        # Compute Kalman gain
        K = np.dot(np.dot(error_state_covariance, H.T),
                   np.linalg.inv(np.dot(np.dot(H, error_state_covariance), H.T) + Q))

        # Compute new error state covariance
        I = np.identity(18)
        error_state_covariance = np.dot(np.dot((I - np.dot(K, H)), error_state_covariance), (I - np.dot(K, H)).T) \
                                 + np.dot(np.dot(K, Q), K.T)

        # Extract sub-vectors from the error state
        delta_x = np.dot(K, innovation)
        delta_substates = np.split(delta_x, [3, 6, 9, 12, 15, 18])

        delta_p = delta_substates[0]
        delta_v = delta_substates[1]
        delta_theta = delta_substates[2]
        delta_a_b = delta_substates[3]
        delta_w_b = delta_substates[4]
        delta_g = delta_substates[5]

        # Update the state variables with their respective error state values
        p = np.add(p, delta_p)
        v = np.add(v, delta_v)
        q = Rotation.from_matrix(np.dot(R, Rotation.from_rotvec(delta_theta.flatten()).as_matrix()))
        a_b = np.add(a_b, delta_a_b)
        w_b = np.add(w_b, delta_w_b)
        g = np.add(g, delta_g)

    return (p, v, q, a_b, w_b, g), error_state_covariance, innovation
