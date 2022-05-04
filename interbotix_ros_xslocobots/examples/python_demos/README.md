# Interbotix X-Series Locobot Python API Demos

[![View Documentation](https://trossenrobotics.com/docs/docs_button.svg)](https://www.trossenrobotics.com/docs/interbotix_xslocobots/ros_packages/python_demos.html)

## Overview

This directory showcases various ways of using the [Interbotix Python Locobot Module](https://github.com/Interbotix/interbotix_ros_toolboxes/blob/main/interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/locobot.py) (click the link to see the fully documented code). Simply put, this API was created so that users with little to no ROS experience would still have the ability to control any Interbotix Locobot supported by the *interbotix_xs_sdk*. Specifically, the API also allows a user to make an arm go to desired end-effector poses or follow Cartesian trajectories. This last feature was made possible by the [Modern Robotics: Mechanics, Planning, and Control Code Library](https://github.com/NxRLab/ModernRobotics) created at Northwestern University. It also allows the ability to move the camera pan/tilt servos and send velocity commands to the base.

For the API to work, the arm and pan/tilt joints must be set to 'position' control and the gripper set to 'PWM' control (conveniently, these are the default configs in the *interbotix_xslocobot_control* package). Furthermore, the API assumes that all the arm-joint motors' [Drive Mode](http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#drive-mode) registers are set to [Time-Based-Profile](http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#profile-velocity112) (this is also the default configuration). In a nutshell, this setting makes it very easy for you as the user to customize the duration and smoothness of an arm's motion from one pose to the next.
