# interbotix_xslocobot_perception

[![View Documentation](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros1_packages/perception_pipeline_configuration.html)

## Overview

This package contains the necessary config and launch files to get any of the Interbotix X-Series LoCoBot arms working with the [perception pipeline](https://industrial-training-master.readthedocs.io/en/melodic/_source/session5/Building-a-Perception-Pipeline.html). The end result allows for an arm to pick up any small, non-reflective object from a tabletop-type surface that is within a RealSense color/depth camera's field of view. While any Intel RealSense color/depth camera can be used, this package was mainly tested with the [D435](https://www.intelrealsense.com/depth-camera-d435/) camera. See more details on how the pipeline works in the [interbotix_perception_modules](https://github.com/Interbotix/interbotix_ros_toolboxes/tree/main/interbotix_perception_toolbox/interbotix_perception_modules) ROS package.
