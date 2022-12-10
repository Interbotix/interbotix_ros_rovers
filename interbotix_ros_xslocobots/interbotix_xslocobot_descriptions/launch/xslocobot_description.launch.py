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
)
from interbotix_xs_modules.xs_launch.xs_launch import determine_use_sim_time_param
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    rviz_frame_launch_arg = LaunchConfiguration('rviz_frame')
    rvizconfig_launch_arg = LaunchConfiguration('rvizconfig')
    use_joint_pub_launch_arg = LaunchConfiguration('use_joint_pub')
    use_joint_pub_gui_launch_arg = LaunchConfiguration('use_joint_pub_gui')
    rate_launch_arg = LaunchConfiguration('rate')
    source_list_launch_arg = LaunchConfiguration('source_list')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')

    # sets use_sim_time parameter to 'true' if using gazebo hardware
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=hardware_type_launch_arg
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name_launch_arg,
        parameters=[{
            'robot_description': ParameterValue(robot_description_launch_arg, value_type=str),
            'use_sim_time': use_sim_time_param,
        }],
        output={'both': 'log'},
    )

    joint_state_publisher_node = Node(
        condition=IfCondition(use_joint_pub_launch_arg),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=robot_name_launch_arg,
        parameters=[{
            'rate': rate_launch_arg,
            'source_list': source_list_launch_arg,
        }],
        output={'both': 'log'},
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(use_joint_pub_gui_launch_arg),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=robot_name_launch_arg,
        output={'both': 'log'},
    )

    rviz2_node = Node(
        condition=IfCondition(use_rviz_launch_arg),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_name_launch_arg,
        arguments=[
            '-d', rvizconfig_launch_arg,
            '-f', rviz_frame_launch_arg,
        ],
        parameters=[{
            'use_sim_time': use_sim_time_param,
        }],
        output={'both': 'log'},
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            default_value=EnvironmentVariable('INTERBOTIX_XSLOCOBOT_ROBOT_MODEL'),
            choices=get_interbotix_xslocobot_models(),
            description=(
                'model type of the Interbotix LoCoBot such as `locobot_base` or `locobot_wx250s`.'
            ),
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
                'the Interbotix Arm model on the LoCoBot; this should never be set manually but '
                'rather left to its default value.'
            ),
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
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_frame',
            default_value=(LaunchConfiguration('robot_name'), '/base_link'),
            description=(
                'fixed frame in RViz; this should be changed to `map` or `odom` if '
                'mapping or using local odometry respectively.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xslocobot_descriptions'),
                'rviz',
                'xslocobot_description.rviz',
            ]),
            description='file path to the config file RViz should load.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_joint_pub',
            default_value='false',
            choices=('true', 'false'),
            description='if `true`, launches the joint_state_publisher node.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_joint_pub_gui',
            default_value='false',
            choices=('true', 'false'),
            description='if `true`, launches the joint_state_publisher GUI.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rate',
            default_value='10',
            description='JointState topic publish rate in Hz.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'source_list',
            default_value=TextSubstitution(text='[]'),
            description='list of joint state topics that should be merged together.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock.'
            )
        )
    )
    declared_arguments.extend(
        declare_interbotix_xslocobot_robot_description_launch_arguments(),
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
