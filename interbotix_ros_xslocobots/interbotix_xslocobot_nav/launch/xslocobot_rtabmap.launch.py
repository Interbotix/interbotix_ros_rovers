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
    TimerAction,
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
    use_lidar_launch_arg = LaunchConfiguration('use_lidar')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    slam_mode_launch_arg = LaunchConfiguration('slam_mode')
    rtabmap_args_launch_arg = LaunchConfiguration('rtabmap_args')
    use_rtabmapviz_launch_arg = LaunchConfiguration('use_rtabmapviz')
    rtabmap_output_location_launch_arg = LaunchConfiguration('rtabmap_output_location')
    rtabmapviz_args_launch_arg = LaunchConfiguration('rtabmapviz_args')
    database_path_launch_arg = LaunchConfiguration('database_path')
    camera_tilt_angle_launch_arg = LaunchConfiguration('camera_tilt_angle')
    launch_driver_launch_arg = LaunchConfiguration('launch_driver')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    use_base_odom_tf_launch_arg = LaunchConfiguration('use_base_odom_tf')
    launch_nav2_launch_arg = LaunchConfiguration('launch_nav2')

    # sets use_sim_time parameter to 'true' if using gazebo hardware
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=hardware_type_launch_arg
    )

    rtabmap_parameters = {
        'visual_odometry': False,  # TODO
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_rgbd': True,
        'subscribe_stereo': False,
        'subscribe_scan': use_lidar_launch_arg,
        'subscribe_scan_cloud': False,
        'subscribe_scan_descriptor': False,
        'subscribe_user_data': False,
        'subscribe_odom_info': False,
        'qos_scan': 1,
        'frame_id': (robot_name_launch_arg, '/base_footprint'),
        'map_frame_id': 'map',
        'odom_frame_id': (robot_name_launch_arg, '/odom'),
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
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/AngularUpdate': '0.01',
        'RGBD/LinearUpdate': '0.01',
        'RGBD/LocalRadius': '5',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'Grid/MaxObstacleHeight': '0.7',
        'Grid/RayTracing': 'true',
        'Reg/Force3DoF': 'true',
        'Mem/STMSize': '30',
        'use_sim_time': use_sim_time_param,
    }

    remappings_rtabmap=[
        ('scan', ('/', robot_name_launch_arg, '/scan')),
        ('initialpose', '/initialpose'),
        ('map', '/map'),
    ]

    if IfCondition(use_lidar_launch_arg.perform(context)).evaluate(context):
        rtabmap_default_args = [
            '--RGBD/ProximityBySpace true',
            '--RGBD/ProximityPathMaxNeighbors 10',
            '--Grid/FromDepth false',
            '--Grid/RangeMax 0',
            '--Reg/Strategy 1',
            '--Icp/VoxelSize 0.05',
            '--Icp/CorrespondenceRatio 0.4',
            '--Icp/MaxCorrespondenceDistance 0.1'
        ]
    else:
        rtabmap_default_args = [
            '--Grid/FromDepth true',
            '--Grid/MaxObstacleHeight 0.7',
            '--Reg/Strategy 0',
        ]

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
            'use_lidar': use_lidar_launch_arg,
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

    rtabmap_rgbd_sync_node = Node(
        package='rtabmap_ros',
        executable='rgbd_sync',
        name='rgbd_sync',
        namespace=(robot_name_launch_arg, '/rtabmap'),
        parameters=[{
            'approx_sync': False,
            'use_sim_time': use_sim_time_param,
        }],
        remappings=[
            ('rgb/image', ('/', robot_name_launch_arg,'/camera/color/image_raw')),
            ('depth/image', ('/', robot_name_launch_arg,'/camera/aligned_depth_to_color/image_raw')),
            ('rgb/camera_info', ('/', robot_name_launch_arg,'/camera/color/camera_info')),
        ],
        output={'both': rtabmap_output_location_launch_arg.perform(context)},
    )

    rtabmap_point_cloud_xyzrgb_node = Node(
        package='rtabmap_ros',
        executable='point_cloud_xyzrgb',
        name='point_cloud_xyzrgb',
        namespace=(robot_name_launch_arg, '/rtabmap'),
        parameters=[{
            'decimation': 4,
            'voxel_size': 0.05,
            'approx_sync': False,
            'use_sim_time': use_sim_time_param,
        }],
        remappings=[
            ('cloud', 'depth/color/voxels')
        ],
        output={'both': rtabmap_output_location_launch_arg.perform(context)},
    )

    rtabmap_obstacles_detection_node = Node(
        package='rtabmap_ros',
        executable='obstacles_detection',
        namespace=(robot_name_launch_arg, '/rtabmap'),
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
        output={'both': rtabmap_output_location_launch_arg.perform(context)},
    )

    rtabmap_mapping_node = Node(
        condition=LaunchConfigurationEquals('slam_mode', 'mapping'),
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        namespace=(robot_name_launch_arg, '/rtabmap'),
        parameters=[
            rtabmap_parameters,
            {
                'Mem/IncrementalMemory': 'True',
                'Mem/InitWMWithAllNodes': 'False',
            },
        ],
        remappings=remappings_rtabmap,
        arguments=[rtabmap_default_args, rtabmap_args_launch_arg],
        output={'both': rtabmap_output_location_launch_arg.perform(context)},
    )

    rtabmap_localization_node = Node(
        condition=LaunchConfigurationEquals('slam_mode', 'localization'),
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        namespace=(robot_name_launch_arg, '/rtabmap'),
        parameters=[
            rtabmap_parameters,
            {
                'Mem/IncrementalMemory':'False',
                'Mem/InitWMWithAllNodes':'True'
            },
        ],
        remappings=remappings_rtabmap,
        arguments=[rtabmap_default_args, rtabmap_args_launch_arg],
        output={'both': rtabmap_output_location_launch_arg.perform(context)},
    )

    rtabmapviz_node = Node(
        package='rtabmap_ros',
        executable='rtabmapviz',
        namespace=robot_name_launch_arg,
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
        output={'both': rtabmap_output_location_launch_arg.perform(context)},
    )

    rtabmap_bringup_delay_action = TimerAction(
        period=LaunchConfiguration('rtabmap_bringup_delay'),
        actions=[
            rtabmap_rgbd_sync_node,
            rtabmap_obstacles_detection_node,
            rtabmap_point_cloud_xyzrgb_node,
            rtabmap_mapping_node,
            rtabmap_localization_node,
            rtabmapviz_node,
        ]
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
        }.items(),
    )

    nav2_bringup_delay_action = TimerAction(
        period=LaunchConfiguration('nav2_bringup_delay'),
        actions=[
            nav2_bringup_launch_include,
        ]
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

    return [
        xslocobot_control_launch_include,
        camera_tilt_angle_executable,
        rtabmap_bringup_delay_action,
        nav2_bringup_delay_action,
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
            'use_lidar',
            default_value='false',
            choices=('true', 'false'),
            description='if `true`, the RPLidar node is launched.',
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
            'use_rviz',
            default_value='false',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'slam_mode',
            default_value='mapping',
            choices=('mapping', 'localization'),
            description='the mode to launch the SLAM in using RTAB-MAP.',
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
            'rtabmap_output_location',
            default_value='screen',
            choices=('screen', 'log'),
            description='set the logging location for the rtabmap nodes.',
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
            'rtabmap_bringup_delay',
            default_value='15',
            description='time in seconds to delay the bringup of the RTAB-Map nodes.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'nav2_bringup_delay',
            default_value='20',
            description='time in seconds to delay the bringup of Nav2.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_nav2',
            default_value='true',
            choices=('true', 'false'),
            description=(
                '`true` if the Nav2 stack should be launched; set to `false` if launching '
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
    declared_arguments.extend(
        declare_interbotix_xslocobot_robot_description_launch_arguments(
            show_gripper_bar='true',
            show_gripper_fingers='true',
            hardware_type='actual',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
