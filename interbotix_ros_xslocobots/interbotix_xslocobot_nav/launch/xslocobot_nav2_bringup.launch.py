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

"""
This launch script borrows heavily from the original Nav2 bringup launch file:
    https://github.com/ros-planning/navigation2/blob/2de3f92c0f476de4bda21d1fc5268657b499b258/nav2_bringup/bringup/launch/bringup_launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    GroupAction
)
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import (
    Node,
    SetParameter
)

from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    namespace_launch_arg = LaunchConfiguration('namespace')
    use_sim_time_launch_arg = LaunchConfiguration('use_sim_time')
    log_level_launch_arg = LaunchConfiguration('log_level')
    autostart_launch_arg = LaunchConfiguration('autostart')
    use_composition_launch_arg = LaunchConfiguration('use_composition')
    use_respawn_launch_arg = LaunchConfiguration('use_respawn')
    nav2_params_file_launch_arg = LaunchConfiguration('nav2_params_file')
    cmd_vel_topic_launch_arg = LaunchConfiguration('cmd_vel_topic')
    map_yaml_file_launch_arg = LaunchConfiguration('map')
    # Set env var to print messages to stdout immediately
    set_logging_env_var = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    lifecycle_nodes_navigation = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    lifecycle_nodes_slam = [
        'map_saver'
    ]

    lifecycle_nodes_localization = [
        'map_server',
        'amcl'
    ]

    remappings = [
        ('/cmd_vel', cmd_vel_topic_launch_arg),
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    tf_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time_launch_arg,
        'autostart': autostart_launch_arg,
        'yaml_filename': map_yaml_file_launch_arg,
    }

    configured_params = RewrittenYaml(
        source_file=nav2_params_file_launch_arg,
        root_key=namespace_launch_arg,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[
            configured_params
        ],
        remappings=tf_remappings + remappings,
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            configured_params
        ],
        remappings=tf_remappings,
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            configured_params
        ],
        remappings=tf_remappings,
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            configured_params
        ],
        remappings=tf_remappings,
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            configured_params
        ],
        remappings=tf_remappings,
    )

    lifecycle_manager_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time_launch_arg},
            {'autostart': autostart_launch_arg},
            {'node_names': lifecycle_nodes_navigation},
        ]
    )

    slam_toolbox_nav2_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time_launch_arg),
            Node(
                condition=IfCondition(use_composition_launch_arg),
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[configured_params, {'autostart': autostart_launch_arg}],
                arguments=['--ros-args', '--log-level', log_level_launch_arg],
                remappings=tf_remappings,
                output='screen'
            ),
            Node(
                condition=LaunchConfigurationEquals('slam_mode', 'mapping'),
                package='nav2_map_server',
                executable='map_saver_server',
                output='screen',
                respawn=use_respawn_launch_arg,
                respawn_delay=2.0,
                arguments=[
                    '--ros-args', '--log-level', log_level_launch_arg
                ],
                parameters=[configured_params],
                remappings=tf_remappings
            ),
            Node(
                condition=LaunchConfigurationEquals('slam_mode', 'localization'),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn_launch_arg,
                respawn_delay=2.0,
                arguments=[
                    '--ros-args', '--log-level', log_level_launch_arg
                ],
                parameters=[configured_params],
                remappings=tf_remappings
            ),
            Node(
                condition=LaunchConfigurationEquals('slam_mode', 'localization'),
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn_launch_arg,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level_launch_arg],
                remappings=tf_remappings
            ),
            Node(
                condition=LaunchConfigurationEquals('slam_mode', 'mapping'),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                arguments=[
                    '--ros-args', '--log-level', log_level_launch_arg
                ],
                parameters=[
                    {'use_sim_time': use_sim_time_launch_arg},
                    {'autostart': autostart_launch_arg},
                    {'node_names': lifecycle_nodes_slam},
                ]
            ),
            Node(
                condition=LaunchConfigurationEquals('slam_mode', 'localization'),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=[
                    '--ros-args', '--log-level', log_level_launch_arg
                ],
                parameters=[
                    {'use_sim_time': use_sim_time_launch_arg},
                    {'autostart': autostart_launch_arg},
                    {'node_names': lifecycle_nodes_localization},
                ]
            ),
        ]
    )

    return [
        set_logging_env_var,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        lifecycle_manager_navigation_node,
        slam_toolbox_nav2_nodes,
    ]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='top-level namespace',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            choices=('true', 'false'),
            description='automatically startup the Nav2 stack.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_composition',
            default_value='true',
            choices=('true', 'false'),
            description='Whether to use composed bringup',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value=(LaunchConfiguration('robot_name'), '/diffdrive_controller/cmd_vel_unstamped'),
            description="topic to remap /cmd_vel to."
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
            'slam_mode',
            default_value='mapping',
            choices=('mapping', 'localization'),
            description='the mode to launch the SLAM in using RTAB-MAP or SLAM Toolbox.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file to load if using localization mode'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            choices=('debug', 'info', 'warn', 'error', 'fatal'),
            description='set the logging level of the Nav2 nodes.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_respawn', default_value='false',
            choices=('true', 'false'),
            description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])