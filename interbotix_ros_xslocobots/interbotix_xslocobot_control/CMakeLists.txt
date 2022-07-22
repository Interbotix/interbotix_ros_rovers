cmake_minimum_required(VERSION 2.8.3)
project(interbotix_xslocobot_control)

set(ROS_DEPS
  interbotix_xs_sdk
  interbotix_xs_modules
  interbotix_xslocobot_descriptions
  realsense2_camera
  rgbd_launch
  rplidar_ros
)

# Add kobuki_node to dependencies if base_type is kobuki
if($ENV{INTERBOTIX_XSLOCOBOT_BASE_TYPE} STREQUAL "kobuki")
  list(APPEND ROS_DEPS "kobuki_node")
endif()

find_package(catkin REQUIRED COMPONENTS
  ${ROS_DEPS}
)

catkin_package(CATKIN_DEPENDS ${ROS_DEPS})
