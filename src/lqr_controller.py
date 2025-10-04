#!/usr/bin/env python3
from ABG_matrix import THRUST_ALLOC_MAT
from lqr_solver import LqrSolver
import numpy as np
from collections import namedtuple
from threading import Lock
import math

LQR_THROTTLE_LIMIT = 0.2
RAW_THROTTLE_LIMIT = 0.8

ControllerParameters = namedtuple('ControllerParameters', ['q_matrix', 'r_matrix',
                                                           'max_lqr_throttle',
                                                           'max_raw_throttle',
                                                           'throttle_attenuation_factor',
                                                           'lqr_enabled_on_start'])


class LqrController(LqrSolver):
    """
    A throttle generator for ASUQTR AUV's 8 motors. Takes a parameters class as argument
    (limits, attenuation factors, etc.) and applies them to control vectors generated
    by inherited method. Also allows to generate uncontrolled motor throttles, called
    'raw throttles'
    """

    def __init__(self, controller_parameters):
        super().__init__(controller_parameters.q_matrix, controller_parameters.r_matrix)

        # Check correctness and limit ROS params
        self._sanity_check_controller_parameters(controller_parameters)

        # Load ROS Parameters
        self.throttle_attenuation_factor = controller_parameters.throttle_attenuation_factor
        self.max_lqr_throttle = controller_parameters.max_lqr_throttle
        self.max_raw_throttle = controller_parameters.max_raw_throttle

        # Command vector for raw uncontrolled thrusts. The vector represents 3d for velocities and angular velocities
        self.raw_cmd_vector = np.zeros(6, dtype=np.float32)

        # Lock for concurrent access to 'lqr enables' switch
        self._lqr_enabled_lock = Lock()
        self._lqr_enabled = controller_parameters.lqr_enabled_on_start

    @staticmethod
    def _sanity_check_controller_parameters(controller_parameters):
        """
        Checks if given parameters are within possible operation limits.
        Else, assign default limit value.
        :param kwargs: dict of ROS parameters and publishers
        """
        if controller_parameters.max_lqr_throttle > 1:
            controller_parameters.max_lqr_throttle = LQR_THROTTLE_LIMIT
        if controller_parameters.max_raw_throttle > 1:
            controller_parameters.max_raw_throttle = RAW_THROTTLE_LIMIT
        if controller_parameters.throttle_attenuation_factor <= 0 or controller_parameters.throttle_attenuation_factor > 1:
            controller_parameters.throttle_attenuation_factor = 1

    @property
    def lqr_enabled(self):
        with self._lqr_enabled_lock:
            return self._lqr_enabled

    @lqr_enabled.setter
    def lqr_enabled(self, value):
        with self._lqr_enabled_lock:
            self._lqr_enabled = value

    def change_throttle_attenuation_factor(self, new_value):
        if not isinstance(new_value, (float, int)):
            raise TypeError
        if new_value <= 0 or new_value > 1:
            raise ValueError
        self.throttle_attenuation_factor = new_value

    def compute_throttles(self) -> np.ndarray:
        if self.lqr_enabled:
            return self.compute_controlled_throttles()
        else:
            return self.compute_raw_throttles()

    def compute_raw_throttles(self) -> np.ndarray:
        """ Bypass LQR calculation and get motor thrusts based only on the AUV's motor allocation matrix. Output vector
        is velocities in [x, y, z, roll, pitch, yaw]. This feature is useful for pool testing, for example when tuning
        the lqr control algorithm. """
        raw_throttles = np.clip(THRUST_ALLOC_MAT.dot(self.raw_cmd_vector) * self.max_raw_throttle,
                                -self.max_raw_throttle,
                                self.max_raw_throttle)
        return raw_throttles

    def compute_controlled_throttles(self) -> np.ndarray:
        """
        Computes motor thrusts by applying specific attenuation factor and limits to lqr control vectors.
        :return: numpy array size 8 of floats between -1 and 1
        """
        lqr_thrusts = self.compute_control_vectors() * self.throttle_attenuation_factor
        lqr_thrusts = np.clip(lqr_thrusts, -self.max_lqr_throttle, self.max_lqr_throttle)
        return lqr_thrusts

    def set_rel_target_angles(self, roll, pitch, yaw):
        """
        Increment the current angle target with the received target
        :param roll: float angle around x axis in rad
        :param pitch: float angle around y axis in rad
        :param yaw: float angle around z axis in rad
        """
        current_state = self.target_state.copy()
        roll += current_state[3]
        pitch += current_state[4]
        yaw += current_state[5]

        """ angle_transfer is called again here even if its called in the internal methods for lqr_error calculations.
        This is because we want to send the correct target_angle value to any caller class that would want to monitor
        this value. Also, this duplicate use should not negatively impact performance unless commands are sent at a
        high rate, in which case the impact won't be breaking bad. """
        target_angles = [self._angle_transfer(angle) for angle in (roll, pitch, yaw)]

        # TODO: Handle case for -90 < pitch < 90
        self.target_state[3:6] = target_angles

    def set_abs_target_angles(self, roll, pitch, yaw):
        """
        Overwrite the current target angles to an absolute angular position. If the commanded x, y and/or z angle
        value is the float representation of NaN, then the target angle is not updated
        :param roll: float angle around x axis in rad
        :param pitch: float angle around y axis in rad
        :param yaw: float angle around z axis in rad
        """
        # check if any angle command is set to NaN. This would be the case if you only wanted to force an absolute
        # angle on one of the 3 axes
        current_state = self.target_state.copy()
        roll = current_state[3] if math.isnan(roll) else roll
        pitch = current_state[4] if math.isnan(pitch) else pitch
        yaw = current_state[5] if math.isnan(yaw) else yaw

        target_angles = [self._angle_transfer(angle) for angle in (roll, pitch, yaw)]
        self.target_state[3:6] = target_angles

    def set_body_target_pos(self, x, y, z):
        """ Transfer body referential position command to NED referential relative position command """
        target_pos_increment = self.compute_ned_pos_quats(np.array([x, y, z]), self.quaternions)
        self.target_state[0:3] += target_pos_increment
        #TODO test this

    def set_abs_ned_target_pos(self, x, y, z):
        """ Overwrites target position relative to North East Down referential """
        new_target_state = self.target_state.copy()
        x = new_target_state[0] if math.isnan(x) else x
        y = new_target_state[1] if math.isnan(y) else y
        z = new_target_state[2] if math.isnan(z) else z
        self.target_state[0:3] = np.array([x, y, z])

    def set_rel_ned_target_pos(self, x, y, z):
        """ Increments target position relative to North East Down referential """
        current_state = self.target_state[0:3].copy()
        x += current_state[0]
        y += current_state[1]
        z += current_state[2]
        self.target_state[0:3] = np.array([x, y, z])
