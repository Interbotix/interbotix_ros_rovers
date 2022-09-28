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

from pathlib import Path

from interbotix_xs_modules.xs_common import (
    get_interbotix_xslocobot_models,
)
from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xslocobot_robot_description_launch_arguments,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.event_handlers import OnProcessExit
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
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    rviz_config_launch_arg = LaunchConfiguration('rvizconfig')
    world_filepath_launch_arg = LaunchConfiguration('world_filepath')
    use_gazebo_gui_launch_arg = LaunchConfiguration('use_gazebo_gui')
    use_gazebo_verbose_launch_arg = LaunchConfiguration('use_gazebo_verbose')
    use_gazebo_debug_launch_arg = LaunchConfiguration('use_gazebo_debug')
    start_gazebo_paused_launch_arg = LaunchConfiguration('start_gazebo_paused')
    enable_gazebo_recording_launch_arg = LaunchConfiguration('enable_gazebo_recording')
    robot_description_launch_arg = LaunchConfiguration('robot_description')

    # Set ignition resource paths
    gz_resource_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value=''),
            ':',
            str(Path(
                FindPackageShare('interbotix_common_sim').perform(context)
            ).parent.resolve()),
            ':',
            str(Path(
                FindPackageShare('interbotix_xslocobot_descriptions').perform(context)
            ).parent.resolve()),
        ]
    )

    gz_model_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            '/usr/share/gazebo-11/models/',
            ':',
            str(Path(
                FindPackageShare('interbotix_common_sim').perform(context)
            ).parent.resolve()),
            ':',
            str(Path(
                FindPackageShare('interbotix_xslocobot_descriptions').perform(context)
            ).parent.resolve()),
        ]
    )

    gz_media_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_MEDIA_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MEDIA_PATH', default_value=''),
            ':',
            str(Path(
                FindPackageShare('interbotix_common_sim').perform(context)
            ).parent.resolve()),
            ':',
            str(Path(
                FindPackageShare('interbotix_xslocobot_descriptions').perform(context)
            ).parent.resolve()),
        ]
    )

    # Set GAZEBO_MODEL_URI to empty string to prevent Gazebo from downloading models
    gz_model_uri_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_URI',
        value=['']
    )

    # Set GAZEBO_MODEL_DATABASE_URI to empty string to prevent Gazebo from downloading models
    gz_model_uri_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=['']
    )

    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ]),
        ]),
        launch_arguments={
            'verbose': use_gazebo_verbose_launch_arg,
            'world': world_filepath_launch_arg,
            'pause': start_gazebo_paused_launch_arg,
            'record': enable_gazebo_recording_launch_arg,
            'gdb': use_gazebo_debug_launch_arg,
            'valgrind': use_gazebo_debug_launch_arg,
            'gui': use_gazebo_gui_launch_arg,
        }.items(),
    )

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-entity', 'robot_description',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
        output={'both': 'log'},
    )

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            'controller_manager',
            'joint_state_broadcaster',
        ],
    )

    spawn_arm_controller_node = Node(
        condition=LaunchConfigurationNotEquals(
            launch_configuration_name='robot_model',
            expected_value='locobot_base'
        ),
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            'controller_manager',
            'arm_controller',
        ],
    )

    spawn_gripper_controller_node = Node(
        condition=IfCondition(LaunchConfiguration('use_gripper')) and
        LaunchConfigurationNotEquals(
            launch_configuration_name='robot_model',
            expected_value='locobot_base'
        ),
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            'controller_manager',
            'gripper_controller',
        ],
    )

    spawn_camera_controller_node = Node(
        name='camera_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            'controller_manager',
            'camera_controller',
        ],
    )

    spawn_diffdrive_controller_node = Node(
        name='diffdrive_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            'controller_manager',
            'diffdrive_controller',
        ],
    )

    xslocobot_description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xslocobot_descriptions'),
                'launch',
                'xslocobot_description.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'use_rviz': use_rviz_launch_arg,
            'rvizconfig': rviz_config_launch_arg,
            'use_sim_time': 'true',
            'robot_description': robot_description_launch_arg,
        }.items(),
    )

    # spawn joint_state_broadcaster after robot is spawned
    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[spawn_joint_state_broadcaster_node]
        )
    )

    # spawn diffdrive_controller after joint_state_broadcaster is spawned
    load_diffdrive_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_diffdrive_controller_node]
        )
    )

    # spawn camera_controller after joint_state_broadcaster is spawned
    load_camera_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_camera_controller_node]
        )
    )

    # spawn arm_controller controller after joint_state_broadcaster is spawned
    load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_arm_controller_node]
        )
    )

    # spawn gripper_controller controller after joint_state_broadcaster is spawned
    load_gripper_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_gripper_controller_node]
        )
    )

    return [
        gz_resource_path_env_var,
        gz_model_path_env_var,
        gz_media_path_env_var,
        gz_model_uri_env_var,
        gazebo_launch_include,
        spawn_robot_node,
        load_diffdrive_controller_event,
        load_camera_controller_event,
        load_joint_state_broadcaster_event,
        load_arm_controller_event,
        load_gripper_controller_event,
        xslocobot_description_launch_include,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=get_interbotix_xslocobot_models(),
            description=(
              'model type of the Interbotix LoCoBot such as `locobot_base` or `locobot_wx250s`.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_model',
            default_value=PythonExpression([
                '"mobile_" + "', LaunchConfiguration('robot_model'), '".split("_")[1]'
            ]),
            description=(
                'the Interbotix Arm model on the LoCoBot; this should never be set manually but '
                'rather left to its default value.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description=(
                "launches RViz if set to `true`; set to `false` if SSH'd into the physical robot."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xslocobot_sim'),
                'rviz',
                'xslocobot_gz_classic.rviz',
            ]),
            description='file path to the config file RViz should load.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'world_filepath',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_common_sim'),
                'worlds',
                'interbotix.world',
            ]),
            description="the file path to the Gazebo 'world' file to load.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_gazebo_gui',
            default_value='true',
            choices=('true', 'false'),
            description='launches the Gazebo GUI if `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_gazebo_verbose',
            default_value='false',
            choices=('true', 'false'),
            description='launches Gazebo with verbose console logging if `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_gazebo_debug',
            default_value='false',
            choices=('true', 'false'),
            description='start gzserver in debug mode using gdb.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_gazebo_paused',
            default_value='false',
            choices=('true', 'false'),
            description='start Gazebo in a paused state.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_gazebo_recording',
            default_value='false',
            choices=('true', 'false'),
            description='enable Gazebo state log recording.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock.'
            )
        )
    )
    declared_arguments.extend(
        declare_interbotix_xslocobot_robot_description_launch_arguments(
            hardware_type='gz_classic',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
