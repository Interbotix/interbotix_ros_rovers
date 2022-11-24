# interbotix_xslocobot_joy

[![View Documentation](https://trossenrobotics.com/docs/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros1_packages/joystick_control.html)

## Overview

This package can be used to control the movements of any rover in the Interbotix X-Series LoCoBot Family using a SONY PS3 or PS4 controller via Bluetooth. In this demo, the 'arm' (if equipped) and 'pan/tilt' servos work in 'position' control mode, the gripper operates in 'PWM' mode, and the base operates in 'velocity' control mode. Refer to the joystick button map below to see how to operate the robot. Specifically, some of the joystick controls manipulate individual joints while others are used to perform 'inverse kinematics' on all the joints to get the end-effector of the robot (defined at 'ee_gripper_link') to move as if it's in Cartesian space. This is done using the [modern_robotics](https://github.com/NxRLab/ModernRobotics/tree/master/packages/Python) code library offered by Northwestern University.
