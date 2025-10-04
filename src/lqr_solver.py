#!/usr/bin/env python3

from threading import Lock
import scipy.linalg
import numpy as np
import math
from ABG_matrix import get_AG_matrices, Bm

PI_TIMES_2 = 2 * math.pi  # Preprocessed for performance concern


class LqrSolver:
    """
    A solver for Riccati's equation in the Linear Quadratic Regulator problem. It implements methods to compute A
    matrix, K gain, thread safe state/command error (called LQR error) and finally control vectors. It also allows to
    change Q and R parameter matrix in a thread safe manner.
    """

    def __init__(self, q_matrix, r_matrix):
        """
        :param q_matrix: float list representing the state cost matrix
        :param r_matrix: float list representing the motor cost matrix
        """
        # Physical model matrix
        self._b_matrix = Bm

        # State Cost matrix
        self._q_matrix = np.diag(np.array(q_matrix, dtype=np.float32))

        # Motors Cost matrix
        self._r_matrix = np.diag(np.array(r_matrix, dtype=np.float32))

        self._inv_r_matrix = np.linalg.inv(self._r_matrix)

        # Quaternions received by a sensor
        self._quaternions = np.zeros(4, dtype=np.float32)

        # Actual state read from hardware sensors
        self._state = np.zeros(12, dtype=np.float32)

        # Desired state asked by operator
        self._target_state = np.zeros(12, dtype=np.float32)

        # Error between _state and _target_state
        self._lqr_error = np.zeros(12, dtype=np.float32)

        # State lock to protect data on write/read
        self._state_lock = Lock()

        # Target state lock to protect data on write/read
        self._target_lock = Lock()

        # Q and R matrix lock to handle tuning requests concurrently
        self._matrix_lock = Lock()

    @staticmethod
    def _compute_a_matrix(state):
        # TODO Create clean class to get A matrix
        return get_AG_matrices(state)

    @staticmethod
    def _angle_transfer(angle):
        """
        Makes sure the angle is between -pi and pi. Check if angle is greater than 2 x PI to avoid useless
        loop modulo operations. This is for performance concerns since this function will be part of a control loop
        :param angle: float angle in rad
        :return: float Angle in rad between -pi and pi
        """
        if angle >= PI_TIMES_2 or angle <= -PI_TIMES_2:
            angle %= PI_TIMES_2
        if angle > math.pi:
            angle -= PI_TIMES_2
        elif angle < -math.pi:
            angle += PI_TIMES_2
        return angle

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """
        For details on the transfer logic, see:
        https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return roll, pitch, yaw

    @staticmethod
    def compute_ned_pos_quats(body_positions: np.array, quaternions: np.array) -> np.array:
        """
        Computes the North East Down velocity vector from quaternions. Equations 2.59, 2.60 and 2.61 in
        "Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen, Edition I, page 29" (see doc folder)
        :param body_positions: vector of 3 floats representing positions in the body-fixed reference frame (x,y,z)
        :param quaternions: vector of 4 quaternions x,y,z,z (float) in the body-fixed reference frame
        """
        # Extract velocities and quaternions with variable name from manual reference
        x, y, z = body_positions
        e1, e2, e3, n = quaternions
        # Apply transformation equations
        x_north = x * (1 - 2 * (e2 ** 2) - 2 * (e3 ** 2)) + 2 * y * ((e1 * e2) - e3 * n) + 2 * z * (e1 * e3 + e2 * n)
        y_east = 2 * x * (e1 * e2 + e3 * n) + y * (1 - 2 * (e1 ** 2) - 2 * (e3 ** 2)) + 2 * z * (e2 * e3 - e1 * n)
        z_down = 2 * x * (e1 * e3 - e2 * n) + 2 * y * (e2 * e3 + e1 * n) + z * (1 - 2 * (e1 ** 2) - 2 * (e2 ** 2))
        return np.array([x_north, y_east, z_down])

    @property
    def r_matrix(self):
        with self._matrix_lock:
            return self._r_matrix

    @r_matrix.setter
    def r_matrix(self, value):
        with self._matrix_lock:
            self._r_matrix = value
            # Pre process of inverse R matrix for performance concerns
            self._inv_r_matrix = np.linalg.inv(value)

    @property
    def q_matrix(self):
        with self._matrix_lock:
            return self._q_matrix

    @q_matrix.setter
    def q_matrix(self, value):
        with self._matrix_lock:
            self._q_matrix = np.diag(np.array(value, dtype=np.float32))

    @property
    def quaternions(self):
        return self._quaternions

    @quaternions.setter
    def quaternions(self, value):
        self._quaternions = value

    @property
    def state(self):
        with self._state_lock:
            return self._state

    @state.setter
    def state(self, value):
        with self._state_lock:
            self._state = value

    @property
    def target_state(self):
        with self._target_lock:
            return self._target_state

    @target_state.setter
    def target_state(self, value):
        with self._target_lock:
            self._target_state = value

    @property
    def lqr_error(self):
        lqr_error = self.state - self.target_state
        """Reverse logic for Z axis from North East Down referential because ASUQTR sensor uses altitude
        instead of depth"""
        lqr_error[2] = self.state[2] - self.target_state[2]
        # The following operation makes sure the error is between -pi and pi.
        lqr_error[3:6] = [self._angle_transfer(angle) for angle in lqr_error[3:6]]
        return lqr_error

    @lqr_error.setter
    def lqr_error(self, value):
        raise AttributeError("LQR error attribute should be READ ONLY")

    def cancel_current_target(self):
        self.target_state = self.state.copy()

    def update_q_matrix(self, q_matrix):
        if isinstance(q_matrix, list):
            if len(q_matrix) == len(self.q_matrix):
                self.q_matrix = q_matrix
            else:
                raise IndexError("New Q matrix value is not of same length as the current one")
        else:
            raise TypeError("q_matrix parameter should be a list")

    def update_r_matrix(self, r_matrix):
        if isinstance(r_matrix, list):
            if len(r_matrix) == len(self.r_matrix):
                self.r_matrix = r_matrix
            else:
                raise IndexError("New R matrix value is not of same length as the current one")
        else:
            raise TypeError("r_matrix parameter should be a list")

    def _compute_k_gain(self):
        # Get A matrix
        state = self.state
        a_matrix = self._compute_a_matrix(state)
        # Try to solve the riccati equation of matrices
        x_matrix = scipy.linalg.solve_continuous_are(a_matrix, self._b_matrix, self.q_matrix, self.r_matrix)
        # Compute the LQR optimal K gain
        k_gain = np.dot(self._inv_r_matrix, np.dot(self._b_matrix.T, x_matrix))
        return k_gain

    def compute_control_vectors(self):
        return -np.dot(self._compute_k_gain(), self.lqr_error)
