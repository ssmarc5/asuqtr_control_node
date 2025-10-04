#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty, Bool
from lqr_controller import LqrController
from asuqtr_control_node.srv import TuneMatrixResponse, ToggleLqrResponse, \
    UpdateAttenuationResponse
import numpy as np
from collections import namedtuple  # TODO install and use DataClass

# These values were found experimentally with /flexbe/status topic
BEHAVIOR_RUNNING_CODE = 0
BEHAVIOR_FINISHED_CODE = 1

GamePadParameters = namedtuple('GamePadParameters', ['joystick_dead_zone', 'angular_speed', 'position_speed'])


class AsuqtrRosController(LqrController):
    """
    Implements ASUQTR's specific service functions for the ASUQTR Dashboard. This includes functionalities like
    game pad control of the AUV, control system tuning and control system enable/disable
    """
    MAX_GAMEPAD_DISTANCE = 0.5
    MAX_GAMEPAD_ANGLE = 20 * np.pi / 180

    def __init__(self, game_pad_parameters, controller_parameters):
        super().__init__(controller_parameters)
        self.behavior_is_active = False
        self.start_button_last_state = False
        self.behavior_interrupt = lambda: rospy.Publisher('flexbe/command/preempt', Empty,
                                                          queue_size=1).publish(Empty())
        self.lqr_enabled_topic = rospy.Publisher('control/lqr_enabled_feedback', Bool, latch=True, queue_size=1)
        self.joystick_dead_zone = game_pad_parameters.joystick_dead_zone
        self.game_pad_angular_speed = game_pad_parameters.angular_speed
        self.game_pad_position_speed = game_pad_parameters.position_speed

    def handle_matrix_tuning_request(self, req):
        try:
            self.update_q_matrix(req.Q)
            rospy.set_param("state_cost_matrix", req.Q)
            self.update_r_matrix(req.R)
            rospy.set_param("motor_cost_matrix", req.R)
            return TuneMatrixResponse(201)
        except Exception as parameter_error:
            rospy.logwarn(f'Q or R matrix requested values were invalid:\n{str(parameter_error)}')
            return TuneMatrixResponse(400)

    def handle_lqr_enabled_update_request(self, req):
        try:
            self.update_lqr_enabled_status(req.lqr_enabled)
            if self.lqr_enabled:
                self.cancel_current_target()
            return ToggleLqrResponse(201)
        except TypeError:
            return ToggleLqrResponse(500)

    def update_lqr_enabled_status(self, requested_status):
        self.lqr_enabled = requested_status
        # Give feedback of LQR state on ros network
        self.lqr_enabled_topic.publish(Bool(self.lqr_enabled))

    def handle_lqr_attenuation_factor_change_request(self, req):
        try:
            self.change_throttle_attenuation_factor(req.factor)
            rospy.set_param('attenuation_factor', req.factor)
            return UpdateAttenuationResponse(201)
        except (TypeError, ValueError):
            UpdateAttenuationResponse(400)

    def _avoid_joystick_dead_zone(self, joystick):
        return 0 if (-self.joystick_dead_zone <= joystick <= self.joystick_dead_zone) else joystick

    def game_pad_cb(self, data):
        """ Callback listening to the ASUQTR Dashboard for game pad commands. Allows to control the AUV's motors,
        enable/disable LQR control, cancel a running behavior and various useful functions. Game pad commands are
        disabled while autonomous behavior is active. """
        # Extracting axis, button and data
        button_a = data.buttons[0]  # roll
        button_b = data.buttons[1]  # roll
        button_x = data.buttons[2]  # X
        # button_Y = data.buttons[3]  # Y
        # button_LB = data.buttons[4]  # LB
        # button_RB = data.buttons[5]  # RB
        button_back = data.buttons[6]  # Back
        button_start = data.buttons[7]  # Start
        # button_xbox = data.buttons[8]  # Xbox middle button
        # button_LS_click = data.buttons[9]  # Left stick click
        # button_RS_click = data.buttons[10]  # Right stick click
        left_stick_x = data.axes[0]  # linear X
        left_stick_y = data.axes[1]  # linear Y
        right_stick_x = data.axes[3]  # yaw
        right_stick_y = -data.axes[4]  # inversed pitch
        triggers_axis = (data.axes[2] - data.axes[5]) / 2  # linear Z
        # dpad_x = data.axes[6]  # Dpad left right
        # dpad_y = data.axes[7]  # Dpad up down

        # Interrupt currently running behavior if game pad operator wants to
        if button_back:
            self.behavior_interrupt()

        # Stop processing further commands if autonomous behavior is running
        if self.behavior_is_active:
            return

        # Enable/Disable LQR control if game pad operator wants to
        if button_start and (self.start_button_last_state != button_start):
            self.update_lqr_enabled_status(not self.lqr_enabled)
        self.start_button_last_state = button_start

        # Cancel joysticks dead zones
        right_stick_y = self._avoid_joystick_dead_zone(right_stick_y)
        right_stick_x = self._avoid_joystick_dead_zone(right_stick_x)
        left_stick_x = self._avoid_joystick_dead_zone(left_stick_x)
        left_stick_y = self._avoid_joystick_dead_zone(left_stick_y)

        if self.lqr_enabled:
            rospy.loginfo_throttle(15, 'LQR control Active! Press START on your game pad to disable')
            #  Reset roll and pitch to 0 if this button is pressed
            if button_x:
                self.set_abs_target_angles(0, 0, float("nan"))
                return

            # Update target position of control system. Limit target position updates to configured max
            x = left_stick_y * self.game_pad_position_speed
            y = left_stick_x * self.game_pad_position_speed
            z = triggers_axis * self.game_pad_position_speed
            self.set_body_target_pos(x, y, z)

            # Update target angles of control system. Limit target angles updates to configured max
            roll = (button_a - button_b) * self.game_pad_angular_speed
            pitch = right_stick_y * self.game_pad_angular_speed
            yaw = right_stick_x * self.game_pad_angular_speed
            self.set_rel_target_angles(roll, pitch, yaw)

        else:
            rospy.logwarn_throttle(15, 'Press START on your game pad to start LQR control')
            # Don't use LQR control and compute raw throttles
            self.raw_cmd_vector = [left_stick_y, left_stick_x,
                                   triggers_axis, (button_a - button_b),
                                   -right_stick_y, right_stick_x]

    def behavior_status_cb(self, status):
        """ When a behavior starts, activate control system and disable game pad commands. """
        if status.code == BEHAVIOR_RUNNING_CODE:
            self.lqr_enabled = True
            self.behavior_is_active = True
        else:
            self.behavior_is_active = False

    def imu_angles_cb(self, imu_msg):
        """ Callback to handle ROS IMU msg and update the controller's angles ang angle velocities """
        quaternions = imu_msg.orientation
        ang_vel = imu_msg.angular_velocity
        self.quaternions = np.array([quaternions.x, quaternions.y, quaternions.z, quaternions.w])
        self.state[3:6] = self.quaternion_to_euler(quaternions.x, quaternions.y, quaternions.z, quaternions.w)
        self.state[9:12] = np.array([ang_vel.x, ang_vel.y, ang_vel.z])

    def position_cb(self, pos_msg ):
        """ Callback to handle ROS Point msg and update the controller's position """
        self.state[0:3] = np.array([pos_msg.point.x, pos_msg.point.y, pos_msg.point.z])

    def velocity_cb(self, vel_msg):
        """ Callback to handle ROS Point msg and update the controller's velocities """
        self.state[6:9] = np.array([vel_msg.point.x, vel_msg.point.y, vel_msg.point.z])

    def update_body_target_cb(self, target_msg):
        self.set_body_target_pos(target_msg.x, target_msg.y, target_msg.z)

    def update_rel_ned_target_cb(self, target_msg):
        self.set_rel_ned_target_pos(target_msg.x, target_msg.y, target_msg.z)

    def update_abs_ned_target_cb(self, target_msg):
        self.set_abs_ned_target_pos(target_msg.x, target_msg.y, target_msg.z)

    def update_abs_angle_target_cb(self, target_msg):
        self.set_abs_target_angles(target_msg.x, target_msg.y, target_msg.z)

    def update_rel_angle_target_cb(self, target_msg):
        self.set_rel_target_angles(target_msg.x, target_msg.y, target_msg.z)
