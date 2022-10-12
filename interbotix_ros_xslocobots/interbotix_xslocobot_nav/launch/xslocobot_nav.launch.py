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
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    arm_model_launch_arg = LaunchConfiguration('arm_model')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')
    use_lidar_launch_arg = LaunchConfiguration('use_lidar')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    nav_mode_launch_arg = LaunchConfiguration('nav_mode')
    rtabmap_args_launch_arg = LaunchConfiguration('rtabmap_args')
    use_rtabmapviz_launch_arg = LaunchConfiguration('use_rtabmapviz')
    rtabmapviz_args_launch_arg = LaunchConfiguration('rtabmapviz_args')
    database_path_launch_arg = LaunchConfiguration('database_path')
    camera_tilt_angle_launch_arg = LaunchConfiguration('camera_tilt_angle')
    launch_driver_launch_arg = LaunchConfiguration('launch_driver')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')

    # sets use_sim_time parameter to 'true' if using gazebo hardware
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=hardware_type_launch_arg
    )

    if use_lidar_launch_arg.perform(context):
        rtabmap_default_args = [{
            '--RGBD/NeighborLinkRefining': 'true',
            '--RGBD/ProximityBySpace': 'true',
            '--RGBD/ProximityPathMaxNeighbors': '10',
            '--RGBD/AngularUpdate': '0.01',
            '--RGBD/LinearUpdate': '0.01',
            '--RGBD/LocalRadius': '5',
            '--RGBD/OptimizeFromGraphEnd': 'false',
            '--Grid/FromDepth': 'false',
            '--Grid/MaxObstacleHeight': '0.7',
            '--Grid/RayTracing': 'true',
            '--Grid/RangeMax': '0',
            '--Reg/Force3DoF': 'true',
            '--Reg/Strategy': '1',
            '--Mem/STMSize': '30',
            '--Icp/VoxelSize': '0.05',
            '--Icp/CorrespondenceRatio': '0.4',
            '--Icp/MaxCorrespondenceDistance': '0.1'
        }].append(rtabmap_args_launch_arg.perform(context))
    else:
        rtabmap_default_args = [{
            '--RGBD/NeighborLinkRefining': 'true',
            '--RGBD/AngularUpdate': '0.01',
            '--RGBD/LinearUpdate': '0.01',
            '--RGBD/LocalRadius': '5',
            '--RGBD/OptimizeFromGraphEnd': 'false',
            '--Grid/FromDepth': 'true',
            '--Grid/MaxObstacleHeight': '0.7',
            '--Grid/RayTracing': 'true',
            '--Reg/Force3DoF': 'true',
            '--Reg/Strategy': '0',
            '--Mem/STMSize': '30'
        }].append(rtabmap_args_launch_arg.perform(context))

    is_mapping = 'false' if nav_mode_launch_arg.perform(context) == 'mapping' else 'true'
    not_is_mapping = 'true' if nav_mode_launch_arg.perform(context) == 'mapping' else 'false'

    xslocobot_control_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xslocobot_control'),
                'launch',
                'xslocobot_control.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'arm_model': arm_model_launch_arg,
            'use_lidar': use_lidar_launch_arg,
            'use_rviz': use_rviz_launch_arg,
            'rviz_frame': 'map',
            'use_camera': 'true',
            'rs_camera_align_depth': 'true',
            'use_base': 'true',
            # 'use_dock': 'true',
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
            'use_sim_time': use_sim_time_param,
        }.items(),
        condition=IfCondition(launch_driver_launch_arg),
    )

    rtabmap_container = ComposableNodeContainer(
        name='rtabmap_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='rtabmap_ros',
                plugin='rtabmap_ros::RGBDSync',
                name='rgbd_sync',
                parameters=[{
                    'approx_sync': False,
                    'use_sim_time': use_sim_time_param,
                }],
                remappings=[
                    ('rbg/image', '/camera/color/image_raw'),
                    ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                    ('rgb/camera_info', '/camera/color/camera_info'),
                ],
            ),
            ComposableNode(
                package='rtabmap_ros',
                plugin='rtabmap_ros::PointCloudXYZRGB',
                name='point_cloud_xyzrgb',
                parameters=[{
                    'decimation': 4,
                    'voxel_size': 0.05,
                    'approx_sync': False,
                    'use_sim_time': use_sim_time_param,
                }],
                remappings=[
                    ('cloud', 'depth/color/voxels')
                ],
            ),
            ComposableNode(
                package='rtabmap_ros',
                plugin='rtabmap_ros::ObstaclesDetection',
                name='obstacles_detection',
                parameters=[{
                    'wait_for_transform': 0.2,
                    'frame_id': 'base_footprint',
                    'use_sim_time': use_sim_time_param,
                }],
                remappings=[
                    ('cloud', 'depth/color/voxels'),
                    ('ground', 'depth/color/ground'),
                    ('obstacles', 'depth/color/obstacles'),
                ],
            ),
            ComposableNode(
                package='rtabmap_ros',
                plugin='rtabmap_ros::CoreWrapper',
                name='rtabmap',
                parameters=[{
                    'subscribe_depth': False,
                    'subscribe_rgb': False,
                    'subscribe_rgbd': True,
                    'subscribe_stereo': False,
                    'subscribe_scan': use_lidar_launch_arg,
                    'subscribe_scan_cloud': False,
                    'subscribe_scan_descriptor': False,
                    'subscribe_user_data': False,
                    'subscribe_odom_info': False,
                    'frame_id': 'base_footprint',
                    'map_frame_id': 'map',
                    'odom_frame_id': 'odom',
                    'publish_tf': True,
                    'odom_tf_angular_variance': 0.05,
                    'odom_tf_linear_variance': 0.1,
                    'odom_sensor_sync': False,
                    'wait_for_transform_duration': 0.2,
                    'database_path': database_path_launch_arg,
                    'approx_sync': True,
                    'queue_size': 10,
                    'tag_linear_variance': 0.0001,
                    'tag_angular_variance': 9999,
                    'Mem/InitWMWithAllNodes': not_is_mapping,
                    'Mem/IncrementalMemory': is_mapping,
                    'use_sim_time': use_sim_time_param,
                }],
                remappings=[
                    ('scan', '/scan'),
                    ('initialpose', '/initialpose'),
                    ('goal_out', '/move_base_simple/goal'),
                ],
                extra_arguments=rtabmap_default_args,
                # extra_arguments=rtabmap_args,
            ),
        ],
        output={'both': 'screen'},
    )

    rtabmapviz_node = Node(
        package='rtabmap_ros',
        executable='rtabmapviz',
        parameters=[{
            'subscribe_rgbd': True,
            'subscribe_scan': use_lidar_launch_arg,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'wait_for_transform': True,
            'use_sim_time': use_sim_time_param,
        }],
        arguments=rtabmapviz_args_launch_arg.perform(context),
        condition=IfCondition(use_rtabmapviz_launch_arg.perform(context)),
        output={'both': 'screen'},
    )

    camera_tilt_angle_executable = ExecuteProcess(
        cmd=[
            'ros2 topic pub --once commands/joint_group interbotix_xs_msgs/msg/JointGroupCommand ',
            "--latch '{name: 'camera', cmd: [0, ", camera_tilt_angle_launch_arg, ']}'
        ]
    )

    nav2_bringup_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        )
    )

    return [
        # xslocobot_control_launch_include,
        rtabmap_container,
        # rtabmapviz_node,
        # camera_tilt_angle_executable,
        # nav2_bringup_launch_include,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=get_interbotix_xslocobot_models(),
            description=(
              'model type of the Interbotix Locobot such as `locobot_base` or `locobot_wx250s`.'
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
            'use_lidar',
            default_value='false',
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
            'nav_mode',
            default_value='mapping',
            choices=('mapping', 'localization'),
            description=(
                'set to `mapping` if mapping a new environment; set to `localization` if '
                'navigating through a pre-explored environment.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rtabmap_args',
            default_value='',
            description=(
                'arguments that should be passed to the rtabmap node; note that these arguments '
                'are in addition to the arguments already specified in the `rtabmap_default_args` '
                'variable in the xslocobot_nav.launch.py file.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rtabmapviz',
            default_value='false',
            choices=('true', 'false'),
            description='whether or not to use the RTAB-Map Visualization tool.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rtabmapviz_args',
            default_value='',
            description='arguments to pass to the RTAB-Map Visualization tool.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'database_path',
            default_value=PathJoinSubstitution(['~', '.ros', 'rtabmap.db']),
            description='location where all the mapping data RTAB-Map collects should be stored.',
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
    declared_arguments.extend(
        declare_interbotix_xslocobot_robot_description_launch_arguments(
            show_gripper_bar='true',
            show_gripper_fingers='true',
            hardware_type='actual',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
