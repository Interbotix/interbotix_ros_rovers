# Copyright 2022 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from interbotix_xs_modules.xs_common import (
    get_interbotix_xslocobot_models,
)
from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xslocobot_robot_description_launch_arguments,
    determine_use_sim_time_param,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    arm_model_launch_arg = LaunchConfiguration('arm_model')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    slam_toolbox_params_file_launch_arg = LaunchConfiguration('slam_toolbox_params_file')
    slam_mode_launch_arg = LaunchConfiguration('slam_mode')
    camera_tilt_angle_launch_arg = LaunchConfiguration('camera_tilt_angle')
    launch_driver_launch_arg = LaunchConfiguration('launch_driver')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    use_base_odom_tf_launch_arg = LaunchConfiguration('use_base_odom_tf')
    launch_nav2_launch_arg = LaunchConfiguration('launch_nav2')
    map_yaml_file_launch_arg = LaunchConfiguration('map')

    # sets use_sim_time parameter to 'true' if using gazebo hardware
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=hardware_type_launch_arg
    )

    xslocobot_control_launch_include = IncludeLaunchDescription(
        condition=IfCondition(launch_driver_launch_arg),
        launch_description_source=PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xslocobot_control'),
                'launch',
                'xslocobot_control.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'arm_model': arm_model_launch_arg,
            'use_lidar': 'true',
            'use_rviz': use_rviz_launch_arg,
            'use_base_odom_tf': use_base_odom_tf_launch_arg,
            'rviz_frame': 'map',
            'use_camera': 'true',
            'rs_camera_align_depth': 'true',
            'use_base': 'true',
            # 'use_dock': 'true',
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
            'use_sim_time': use_sim_time_param,
        }.items(),
    )

    camera_tilt_angle_cmd = (
        f"ros2 topic pub --once {robot_name_launch_arg.perform(context)}/commands/joint_group "
        "interbotix_xs_msgs/msg/JointGroupCommand "
        f"'{{name: 'camera', cmd: [0, {camera_tilt_angle_launch_arg.perform(context)}]}}'"
    )

    camera_tilt_angle_executable = ExecuteProcess(
        name='camera_tilt',
        cmd=[camera_tilt_angle_cmd],
        shell=True,
    )
    slam_toolbox_online_sync_slam_node = Node(
        condition=LaunchConfigurationEquals('slam_mode', 'online_sync'),
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_params_file_launch_arg,
            {
                'use_sim_time': use_sim_time_param,
            }
        ],
        output='screen'
    )

    slam_toolbox_online_async_slam_node = Node(
        condition=LaunchConfigurationEquals('slam_mode', 'online_async'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_params_file_launch_arg,
            {
                'use_sim_time': use_sim_time_param,
            }
        ],
        output='screen'
    )

    slam_toolbox_localization_slam_node = Node(
        condition=LaunchConfigurationEquals('slam_mode', 'localization'),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_params_file_launch_arg,
            {
                'use_sim_time': use_sim_time_param,
            }
        ],
        output='screen'
    )

    nav2_bringup_launch_include = IncludeLaunchDescription(
        condition=IfCondition(launch_nav2_launch_arg),
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('interbotix_xslocobot_nav'),
                'launch',
                'xslocobot_nav2_bringup.launch.py'
            ])
        ),
        launch_arguments={
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'use_sim_time': use_sim_time_param,
            'autostart': 'true',
            'params_file': LaunchConfiguration('nav2_params_file'),
            'use_slam_toolbox': 'true',
            'slam_toolbox_mode': slam_mode_launch_arg,
            'map': map_yaml_file_launch_arg,
        }.items(),
    )

    return [
        xslocobot_control_launch_include,
        slam_toolbox_online_sync_slam_node,
        slam_toolbox_online_async_slam_node,
        slam_toolbox_localization_slam_node,
        camera_tilt_angle_executable,
        nav2_bringup_launch_include,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            default_value=EnvironmentVariable('INTERBOTIX_XSLOCOBOT_ROBOT_MODEL'),
            choices=get_interbotix_xslocobot_models(),
            description=(
              'model type of the Interbotix Locobot such as `locobot_base` or `locobot_wx250s`.'
            )
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='locobot',
            description='name of the robot (could be anything but defaults to `locobot`).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_model',
            default_value=PythonExpression([
                '"mobile_" + "', LaunchConfiguration('robot_model'), '".split("_")[1]'
            ]),
            description=(
                'the Interbotix Arm model on the locobot; this should never be set manually but '
                'rather left to its default value.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'xs_driver_logging_level',
            default_value='INFO',
            choices=('DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'),
            description='set the logging level of the X-Series Driver.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_tilt_angle',
            default_value='0.2618',
            description=(
                'desired angle [rad] that the camera should be tilted when doing mapping or '
                'localization.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_base_odom_tf',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'if `true`, the odom TF from the base will be published. This only works on the '
                'Create 3 base.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_driver',
            default_value='true',
            choices=('true', 'false'),
            description=(
                '`true` if the xslocobot_control.launch.py file should be launched; set to `false`'
                ' if you would like to run your own version of this file separately.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            choices=('true', 'false'),
            description='if `true`, the RPLidar node is launched.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'slam_mode',
            default_value='online_async',
            choices=(
                # 'lifelong',
                'localization',
                # 'offline',
                'online_async',
                'online_sync'
            ),
            description=(
                "the mode to launch the SLAM in using the slam_toolbox. Currently only "
                "'localization', 'online_sync', and 'online_async' modes are supported."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'slam_toolbox_params_filename',
            default_value=('slam_toolbox_', LaunchConfiguration('slam_mode'), '.yaml')
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'slam_toolbox_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare("interbotix_xslocobot_nav"),
                'config',
                LaunchConfiguration('slam_toolbox_params_filename'),
            ]),
            description='full path to the ROS 2 parameters file to use for the slam_toolbox node',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_nav2',
            default_value='true',
            choices=('true', 'false'),
            description=(
                '`true` if the Navigation2 stack should be launched; set to `false` if launching '
                'the Nav2 stack using a different method.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xslocobot_nav'),
                'config',
                'nav2_params.yaml'
            ]),
            description=(
                'full path to the ROS 2 parameters file to use when configuring the Nav2 stack.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value=(LaunchConfiguration('robot_name'), '/mobile_base/cmd_vel'),
            description="topic to remap /cmd_vel to."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock; this value is automatically set to `true` if'
                ' using Gazebo hardware.'
            )
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file to load if using localization mode'
        )
    )
    declared_arguments.extend(
        declare_interbotix_xslocobot_robot_description_launch_arguments(
            show_gripper_bar='true',
            show_gripper_fingers='true',
            hardware_type='actual',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
