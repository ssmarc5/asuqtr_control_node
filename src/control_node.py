#!/usr/bin/env python3
import rospy
import numpy as np
from asuqtr_actuator_node.msg import ActuatorThrottle
from asuqtr_control_node.srv import TuneMatrix, ToggleLqr, UpdateAttenuation, UpdateMaxLqrRate, \
    UpdateMaxLqrRateResponse, UpdatePosThreshold, UpdateAngleThreshold, UpdateAsRate
from asuqtr_ros_controller import AsuqtrRosController, GamePadParameters
from asuqtr_control_action_server import AsuqtrControlActionServer, ActionServerParameters
from lqr_controller import ControllerParameters
from sensor_msgs.msg import Imu, Joy
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Float32MultiArray, Header
from flexbe_msgs.msg import BEStatus
from rospy.numpy_msg import numpy_msg

DEFAULT_MAX_LQR_THROTTLE = 0.2
DEFAULT_MAX_RAW_THROTTLE = 0.8
DEFAULT_ATTENUATION_FACTOR = 0.2
DEFAULT_MAX_RATE = 30
DEFAULT_ANGLE_THRESHOLD = 5
DEFAULT_POSITION_THRESHOLD = 0.05
DEFAULT_AS_LOOP_RATE = 10
DEFAULT_ANGULAR_SPEED_FACTOR = 0.1
DEFAULT_JOY_DEAD_ZONE = 0.1
DEFAULT_POS_SPEED_FACTOR = 0.1
DEF_Q = [4, 4, 4, 0.4, 0.4, 0.4, 0.0000000001, 0.0000000001, 0.0000000001, 0.0000000001, 0.0000000001, 0.0000000001]
DEF_R = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]

def load_controller_ros_params():
    q_matrix = rospy.get_param('~state_cost_matrix', DEF_Q)
    r_matrix = rospy.get_param('~motor_cost_matrix', DEF_R)
    max_lqr_throttle = rospy.get_param('~max_lqr_throttle', DEFAULT_MAX_LQR_THROTTLE)
    max_raw_throttle = rospy.get_param('~max_raw_throttle', DEFAULT_MAX_RAW_THROTTLE)
    attenuation_factor = rospy.get_param('~attenuation_factor', DEFAULT_ATTENUATION_FACTOR)
    lqr_enabled_on_start = rospy.get_param('~lqr_enabled_on_start', False)
    return ControllerParameters(q_matrix, r_matrix, max_lqr_throttle, max_raw_throttle, attenuation_factor,
                                lqr_enabled_on_start)


def load_action_server_ros_params():
    angle_threshold = rospy.get_param('~target_angle_threshold', DEFAULT_ANGLE_THRESHOLD) * np.pi / 180
    position_threshold = rospy.get_param('~target_position_threshold', DEFAULT_POSITION_THRESHOLD)
    max_loop_rate = rospy.get_param('~max_action_server_loop_rate', DEFAULT_AS_LOOP_RATE)
    return ActionServerParameters(angle_threshold, position_threshold, max_loop_rate)


def load_game_pad_ros_params():
    ang_speed = rospy.get_param('~angular_speed_factor', DEFAULT_ANGULAR_SPEED_FACTOR)
    pos_speed = rospy.get_param('~position_speed_factor', DEFAULT_POS_SPEED_FACTOR)
    joy_dead_zone = rospy.get_param('~joy_dead_zone', DEFAULT_JOY_DEAD_ZONE)
    return GamePadParameters(joy_dead_zone, ang_speed, pos_speed)


def control_node():
    rospy.init_node('control_node')
    # Load Control System ROS parameters and declare useful application parameters
    max_control_loop_rate = rospy.Rate(rospy.get_param('~max_lqr_loop_rate', DEFAULT_MAX_RATE))
    controller_parameters = load_controller_ros_params()
    as_parameters = load_action_server_ros_params()
    game_pad_parameters = load_game_pad_ros_params()
    ids = np.array(range(8), dtype=np.uint8)

    motors_topic = rospy.Publisher('actuator/motors', numpy_msg(ActuatorThrottle), queue_size=1)
    state_topic = rospy.Publisher('control/state', Float32MultiArray, queue_size=1)
    target_topic = rospy.Publisher('control/target_state', Float32MultiArray, queue_size=1)
    lqr_error_topic = rospy.Publisher('control/lqr_error', Float32MultiArray, queue_size=1)

    # Create controller
    controller = AsuqtrRosController(game_pad_parameters, controller_parameters)
    # Create action server with controller and parameters
    action_server = AsuqtrControlActionServer(controller, as_parameters)

    def update_max_lqr_rate(request):
        nonlocal max_control_loop_rate
        new_rate = request.rate
        try:
            if new_rate > 0:
                max_control_loop_rate = rospy.Rate(new_rate)
                """Set the new loop rate on the parameter server in case a ros node 
                needs to know (ASUQTR dashboard for example) """
                rospy.set_param('~max_lqr_loop_rate', request.rate)
                return UpdateMaxLqrRateResponse(201)
            else:
                raise ValueError
        except (ValueError, Exception) as error:
            UpdateMaxLqrRateResponse(400)
            rospy.logwarn(f"Service 'Update Max Lqr Rate' failed with:\n{str(error)}")

    # Start ROS subscribers with the correct control objects callback
    rospy.Subscriber('vectornav/IMU', numpy_msg(Imu), controller.imu_angles_cb)
    rospy.Subscriber('nav_node/position', numpy_msg(PointStamped), controller.position_cb)
    rospy.Subscriber('nav_node/velocity', numpy_msg(PointStamped), controller.velocity_cb)
    rospy.Subscriber('flexbe/status', BEStatus, controller.behavior_status_cb)
    rospy.Subscriber('dashboard/gamepad', Joy, controller.game_pad_cb)
    rospy.Subscriber('control/body_pos_target', numpy_msg(Point), controller.update_body_target_cb)
    rospy.Subscriber('control/rel_ned_pos_target', numpy_msg(Point), controller.update_rel_ned_target_cb)
    rospy.Subscriber('control/abs_ned_pos_target', numpy_msg(Point), controller.update_abs_ned_target_cb)
    rospy.Subscriber('control/rel_angle_target', numpy_msg(Point), controller.update_rel_angle_target_cb)
    rospy.Subscriber('control/abs_angle_target', numpy_msg(Point), controller.update_abs_angle_target_cb)

    # Start ROS services to change control parameters on runtime
    rospy.Service('control/tune_lqr_matrix', TuneMatrix, controller.handle_matrix_tuning_request)
    rospy.Service('control/toggle_lqr_control', ToggleLqr, controller.handle_lqr_enabled_update_request)
    rospy.Service('control/update_throttle_attenuation', UpdateAttenuation,
                  controller.handle_lqr_attenuation_factor_change_request)
    rospy.Service('control/update_position_threshold', UpdatePosThreshold, action_server.update_position_threshold)
    rospy.Service('control/update_angle_threshold', UpdateAngleThreshold, action_server.update_angle_threshold)
    rospy.Service('control/update_action_server_rate', UpdateAsRate, action_server.update_max_loop_rate)
    rospy.Service('control/update_max_lqr_rate', UpdateMaxLqrRate, update_max_lqr_rate)

    while not rospy.is_shutdown():
        # Compute motor throttles publish to motors topic
        throttles = controller.compute_throttles()
        motors_topic.publish(ActuatorThrottle(ids=ids, throttles=throttles,
                                              header=Header(stamp=rospy.get_rostime())))
        target_topic.publish(Float32MultiArray(data=controller.target_state))
        state_topic.publish(Float32MultiArray(data=controller.state))
        lqr_error_topic.publish(Float32MultiArray(data=controller.lqr_error))
        max_control_loop_rate.sleep()


if __name__ == '__main__':
    control_node()
