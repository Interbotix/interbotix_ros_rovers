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
#    * Neither the name of the the copyright holder nor the names of its
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

from typing import List

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
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    arm_model_launch_arg = LaunchConfiguration('arm_model')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    rviz_frame_launch_arg = LaunchConfiguration('rviz_frame')
    use_base_launch_arg = LaunchConfiguration('use_base')
    use_lidar_launch_arg = LaunchConfiguration('use_lidar')
    use_camera_launch_arg = LaunchConfiguration('use_camera')
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    threshold_launch_arg = LaunchConfiguration('threshold')
    controller_launch_arg = LaunchConfiguration('controller')
    launch_driver_launch_arg = LaunchConfiguration('launch_driver')
    use_sim_launch_arg = LaunchConfiguration('use_sim')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')

    joy_node = Node(
        name='joy_node',
        package='joy',
        executable='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
        }],
        remappings=[
            ('joy', 'commands/joy_raw'),
        ]
    )

    xslocobot_joy_node = Node(
        name='xslocobot_joy',
        package='interbotix_xslocobot_joy',
        executable='xslocobot_joy',
        parameters=[{
            'threshold': threshold_launch_arg,
            'controller': controller_launch_arg
        }],
    )

    xslocobot_robot_node = Node(
        package='interbotix_xslocobot_joy',
        executable='xslocobot_robot.py',
        parameters=[{
            'robot_model': robot_model_launch_arg,
        }],
        arguments=[
            '--robot_model', robot_model_launch_arg.perform(context),
            '--robot_name', '',
            '--use_base', use_base_launch_arg.perform(context),
        ],
    )

    xslocobot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xslocobot_control'),
                'launch',
                'xslocobot_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'arm_model': arm_model_launch_arg,
            'use_rviz': use_rviz_launch_arg,
            'rviz_frame': rviz_frame_launch_arg,
            'use_base': use_base_launch_arg,
            'use_lidar': use_lidar_launch_arg,
            'use_camera': use_camera_launch_arg,
            'mode_configs': mode_configs_launch_arg,
            'use_sim': use_sim_launch_arg,
            'robot_description': robot_description_launch_arg,
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items(),
        condition=IfCondition(launch_driver_launch_arg)
    )

    return [
        xslocobot_control_launch,
        joy_node,
        xslocobot_joy_node,
        xslocobot_robot_node,
    ]


def generate_launch_description():
    declared_arguments: List[DeclareLaunchArgument] = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=get_interbotix_xslocobot_models(),
            description=(
              'model type of the Interbotix LoCoBot such as `locobot_base` or `locobot_wx250s`.'
            )
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
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_frame',
            default_value='base_footprint',
            description=(
                'fixed frame in RViz; this should be changed to `map` or `odom` if '
                'mapping or using local odometry respectively; `base_footprint` otherwise.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_base',
            default_value='true',
            choices=('true', 'false'),
            description='if `true`, the base ROS nodes are launched.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_lidar',
            default_value='false',
            choices=('true', 'false'),
            description='if `true`, the RPLidar node is launched.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_camera',
            default_value='false',
            choices=('true', 'false'),
            description='if `true`, the RealSense camera nodes are launched.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PythonExpression([
                '"',
                PathJoinSubstitution([
                    FindPackageShare('interbotix_xslocobot_joy'),
                    'config',
                    'modes_base.yaml',
                ]),
                '" if "',
                LaunchConfiguration('robot_model'),
                '" == "locobot_base" else "',
                PathJoinSubstitution([
                    FindPackageShare('interbotix_xslocobot_joy'),
                    'config',
                    'modes_all.yaml',
                ]),
                '"',
            ]),
            description="the file path to the 'mode config' YAML file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'threshold',
            default_value='0.75',
            description=(
                'value from 0 to 1 defining joystick sensitivity; a larger number means the '
                'joystick should be less sensitive.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controller',
            default_value='ps4',
            choices=('ps4', 'ps3', 'xbox360'),
            description='type of controller.',

        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_driver',
            default_value='true',
            choices=('true', 'false'),
            description=(
                '`true` if xslocobot_control should be launched - set to `false` if you would like'
                ' to run your own version of this file separately.'
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
            'use_sim',
            default_value='false',
            choices=('true', 'false'),
            description=(
                "if `true`, the DYNAMIXEL simulator node is run; use RViz to visualize the robot's"
                ' motion; if `false`, the real DYNAMIXEL driver node is run.'
            ),
        )
    )
    declared_arguments.extend(
        declare_interbotix_xslocobot_robot_description_launch_arguments()
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
