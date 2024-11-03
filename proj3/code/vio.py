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
    R = q.as_matrix()
    new_p = p + v * dt + 1 / 2 * (R @ (a_m - a_b) + g) * dt ** 2
    new_v = v + (R @ (a_m - a_b) + g) * dt
    q_diff = q * Rotation.from_rotvec(((w_m - w_b) * dt).reshape(3, ))
    # new_q = Rotation.from_rotvec(np.cross(q.as_rotvec(), q_diff.as_rotvec()).reshape(3,))
    new_q = q_diff
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
    R = q.as_matrix()
    I = np.eye(3)
    F230 = (R @ [a_m - a_b]).reshape(3, )
    F231 = np.array([[0, -F230[2], F230[1]],
                     [F230[2], 0, -F230[0]],
                     [-F230[1], F230[0], 0]])
    Fx1 = np.hstack((I, I * dt, np.zeros([3, 12])))
    Fx2 = np.hstack((np.zeros([3, 3]), I, - F231 * dt, - R * dt, np.zeros([3, 3]), I * dt))
    Fx3 = np.hstack((np.zeros((3, 6)), Rotation.from_rotvec(((w_m - w_b) * dt).reshape(3, )).as_matrix().T,
                     np.zeros((3, 3)), -I * dt, np.zeros((3, 3))))
    Fx4 = np.hstack((np.zeros((9, 9)), np.eye(9)))
    Fx = np.vstack((Fx1, Fx2, Fx3, Fx4))
    P1 = Fx @ error_state_covariance @ Fx.T
    # P2
    Qi = np.diag([accelerometer_noise_density ** 2 * dt ** 2, accelerometer_noise_density ** 2 * dt ** 2,
                  accelerometer_noise_density ** 2 * dt ** 2,
                  gyroscope_noise_density ** 2 * dt ** 2, gyroscope_noise_density ** 2 * dt ** 2,
                  gyroscope_noise_density ** 2 * dt ** 2,
                  accelerometer_random_walk ** 2 * dt, accelerometer_random_walk ** 2 * dt,
                  accelerometer_random_walk ** 2 * dt,
                  gyroscope_random_walk ** 2 * dt, gyroscope_random_walk ** 2 * dt, gyroscope_random_walk ** 2 * dt])
    Fi = np.vstack((np.zeros([3, 12]), np.eye(12), np.zeros([3, 12])))
    P2 = Fi @ Qi @ Fi.T
    # return an 18x18 covariance matrix
    return P1 + P2


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
    Q = Q / 4
    # YOUR CODE HERE - compute the innovation next state, next error_state covariance
    R = q.as_matrix()
    Pc = R.T @ (Pw - p)
    x = Pc[0, 0]
    y = Pc[1, 0]
    z = Pc[2, 0]
    print("xyz", x, y, z)
    Zt = np.array([[x / z], [y / z]])
    innovation = uv - Zt
    if norm(innovation) > error_threshold:
        return (p, v, q, a_b, w_b, g), error_state_covariance, innovation

    dPc_dthe = np.array([[0, -Pc[2, 0], Pc[1, 0]],
                         [Pc[2, 0], 0, -Pc[0, 0]],
                         [-Pc[1, 0], Pc[0, 0], 0]])
    dPc_dp = -R.T
    dzt_dPc = np.array([[1, 0, -x / z],
                        [0, 1, -y / z]])
    Ht = np.hstack((dzt_dPc @ dPc_dp * 1 / z, np.zeros([2, 3]), dzt_dPc @ dPc_dthe * 1 / z, np.zeros((2, 9))))
    Kt = error_state_covariance @ Ht.T @ inv(Ht @ error_state_covariance @ Ht.T + Q)
    error_state_covariance2 = (np.eye(18) - Kt @ Ht) @ error_state_covariance @ (np.eye(18) - Kt @ Ht).T + Kt @ Q @ Kt.T
    dx = Kt @ innovation
    dp = dx[0:3, :].reshape(3, 1)
    dv = dx[3:6, :].reshape(3, 1)
    dq = Rotation.from_rotvec(dx[6:9, :].reshape(3, ))
    dab = dx[9:12, :].reshape(3, 1)
    dwb = dx[12:15, :].reshape(3, 1)
    dg = dx[15:18, :].reshape(3, 1)
    return (p + dp, v + dv, q * dq, a_b + dab, w_b + dwb, g + dg), error_state_covariance2, innovation
