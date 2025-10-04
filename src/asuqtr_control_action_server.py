#!/usr/bin/env python3
from actionlib import SimpleActionServer
from asuqtr_control_node.msg import ControlAction, ControlFeedback, ControlResult
from asuqtr_control_node.srv import UpdateAngleThresholdResponse, UpdatePosThresholdResponse, UpdateAsRateResponse
import rospy
from collections import namedtuple

ActionServerParameters = namedtuple('ActionServerParameters', ['position_threshold', 'angle_threshold', 'max_loop_rate'])


class AsuqtrControlActionServer(SimpleActionServer):
    """
    Action server for ASUQTR which listens for Control Goal requests. It will send angle/position commands to the
    controller and continuously (max_loop_rate) check if the current state is within given thresholds for the last
    command. If so, the goal is completed and the server waits for another request.
    """
    def __init__(self, controller, action_server_parameters):
        self.controller = controller
        self.position_threshold = action_server_parameters.position_threshold
        self.angle_threshold = action_server_parameters.angle_threshold
        self.rate = rospy.Rate(action_server_parameters.max_loop_rate)
        super().__init__('control_action_server', ControlAction, execute_cb=self.action_server_cb, auto_start=False)
        self.start()
        rospy.loginfo('Control Action Server initialized!')

    def action_server_cb(self, goal):
        is_absolute = goal.absolute
        if is_absolute:
            self.controller.set_abs_target_angles(goal.roll, goal.pitch, goal.yaw)
            self.controller.set_rel_ned_target_pos(goal.x, goal.y, goal.z)
        else:
            self.controller.set_rel_target_angles(goal.roll, goal.pitch, goal.yaw)
            self.controller.set_body_target_pos(goal.x, goal.y, goal.z)

        while not rospy.is_shutdown():
            # If preempt requested, set target state as current state to neutralise control actions
            if self.is_preempt_requested():
                self.set_preempted()
                self.controller.cancel_current_target()
                break

            # Check if current pose is within accepted threshold and send feedback to client
            lqr_error = self.controller.lqr_error.copy()
            angle_errors = lqr_error[3:6]
            position_errors = lqr_error[0:3]
            if any(abs(angle) > self.angle_threshold for angle in angle_errors) or \
                    any(abs(position) > self.position_threshold for position in position_errors):
                # TODO think about a nice feedback info
                #    self._LQR_action_server.publish_feedback(feedback(percentage_done=x))
                rospy.loginfo_throttle(2, 'Control goal still not acheived...')
                pass
            else:
                self.set_succeeded(ControlResult())
                rospy.loginfo('Control goal reached')
                break
            self.rate.sleep()
        self.set_aborted(ControlResult())

    def update_angle_threshold(self, request):
        if not isinstance(request.threshold, (int, float)) or 0 < request.threshold:
            return UpdateAngleThresholdResponse(400)
        self.angle_threshold = request.threshold
        rospy.set_param("target_angle_threshold", request.threshold)
        return UpdateAngleThresholdResponse(201)

    def update_position_threshold(self, request):
        if not isinstance(request.threshold, (int, float)) or 0 < request.threshold:
            return UpdatePosThresholdResponse(400)
        self.position_threshold = request.threshold
        rospy.set_param("target_position_threshold", request.threshold)
        return UpdatePosThresholdResponse(201)

    def update_max_loop_rate(self, request):
        if not isinstance(request.rate, int) or 0 < request.rate:
            return UpdateAsRateResponse(400)
        self.rate = rospy.Rate(request.rate)
        rospy.set_param("max_loop_rate", request.rate)
        return UpdateAsRateResponse(201)
