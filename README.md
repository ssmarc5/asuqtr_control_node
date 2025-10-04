ASUQTR Control Node
====================

A ROS node for ASUQTR motor's **LQR** control system.

This package serves the purpose of controlling the AUV's movements in space. With an LQR algorithm at its core, it 
manages commands and directs the motors behavior to fit the requested positions.

QuickStart Guide
----------------


####Requirements:
1. A Nvidia Jetson device OR a PC with ROS for ASUQTR installed:
[How to Install ROS for ASUQTR](https://confluence.asuqtr.com/display/SUBUQTR/How+to+install+ROS+for+ASUQTR)
2. (_Optional_) A Vectornav VN-100 IMU connected via USB to your machine. _LQR tests are suboptimal without this sensor_.
3. (_Optional_) A Bar30 pressure sensor connected via I2C on your Jetson device. _LQR tests are suboptimal without this sensor_.

#### Run for LQR tunnings (without autonomous behavior engine):

* <pre><code>roslaunch asuqtr_control_node tunning_lqr.launch</code></pre>
* **Ctrl+C** to quit

#### Run with the whole ASUQTR system:
_On a Jetson device_
* <pre><code>roslaunch asuqtr_mission_management jetson.launch</code></pre>
_On a standard PC_
* <pre><code>roslaunch asuqtr_mission_management pc.launch</code></pre>
* **Ctrl+C** to quit


Overview 
--------

#### Control node

This node acts as an middle-man between the motors' PWM hardware (actuator node) and the commands sent by a 
user(manual node) or the autonomous control mode(flexBE nodes).

If the manual node sends control commands (With an Xbox/Logitech controller, video games style), the control node
receives those commands directly through a topic subscription and update the target state. The the "manual control
mode" works this way because **the operator does not need feedback**.

If an autonomous behavior (a FlexBE state) wants to reach a certain position (or angle position), **this state needs
feedback of the requested goal**. Hence, the need to send a goal to the _control action server_ if we want to use
the control system from an autonomous setup.


#### tunning_lqr.launch

This launch file is used mainly for testing and calibrating the LQR control matrices and parameters.

It starts the the control node with the other necessary nodes for testing:
 * Actuator node (for motors)
 * Imu node (for position)
 * Sensor node (for depth)
 * Manual node (for sending target commands)
 * Rosbridge node (For allowing web page communication with Xbox/Logitech controller)

#### lqr_conf.yaml
This is a parameter file for initial state of the lqr control and various other useful configurations. Descriptions of parameters are included in the file.

Some parameters  contained in this file are :

* max_lqr_loop_rate (Hz)

* max_lqr_throttle (%) for hardware pwm
* target_angle_treshold (degrees)
* state_cost_matrix (coefficients)

and more ...# asuqtr_control_node
