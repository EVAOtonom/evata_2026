from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():

    pkg_share = get_package_share_directory('reel_nav')

    home_dir = os.path.expanduser('~')
    src_dir = os.path.join(home_dir, 'real_ws', 'src', 'reel_nav')

    sdf_file = os.path.join(
        src_dir,
        'final_deneme.sdf'
    )

    rviz_config = os.path.join(
        src_dir,
        'config',
        'nav2_evata_view.rviz'
    )

    map_file = os.path.join(
        src_dir,
        'map',
        'harita.yaml'
    )

    nav2_params = os.path.join(
        src_dir,
        'config',
        'test.yaml'
    )

    ekf_params = os.path.join(
        src_dir,
        'config',
        'ekf.yaml'
    )

    doc = xacro.parse(open(sdf_file))
    xacro.process_doc(doc)

    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': doc.toxml(),
                'use_sim_time': False
            }]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),

	Node(
	    package='reel_evata',
	    executable='OdometerListener',
	    name='initial_pose_setter',
	    output='screen',
	    parameters=[{
		'initialpose_topic': '/initialpose',
		'ekf_set_pose_service': '/set_pose',
		'force_frame_id': 'map',
		'use_current_time': True
	    }]
	),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[ekf_params],
            remappings=[
                ('imu/data', '/imu/data'),
                ('gps/fix', '/gnss_1/llh_position'),
                ('odometry/filtered', '/odom'),
                ('odometry/gps', '/odometry/gps'),
                ('gps/filtered', '/gps/filtered')
            ]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization_node',
            output='screen',
            parameters=[ekf_params],
            remappings=[
                ('odometry/filtered', '/odom')
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'map': map_file,
                'params_file': nav2_params,
                'use_sim_time': 'false'
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                rviz_config
            ],
            output='screen'
        )
    ])
