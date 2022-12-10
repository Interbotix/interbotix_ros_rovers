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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    rviz_frame_launch_arg = LaunchConfiguration('rviz_frame')
    rvizconfig_launch_arg = LaunchConfiguration('rvizconfig')

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_name_launch_arg,
        arguments=[
            '-d', rvizconfig_launch_arg,
            '-f', rviz_frame_launch_arg,
        ],
        output={'both': 'log'},
    )

    return [
        rviz2_node,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='locobot',
            description='name of the robot (could be anything but defaults to `locobot`).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_frame',
            default_value=(LaunchConfiguration('robot_name'), '/base_footprint'),
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
