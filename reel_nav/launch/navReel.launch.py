from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory,
)

import os
import xacro


def generate_launch_description():

    # ============================================================
    # PATHS
    # ============================================================

    home_dir = os.path.expanduser('~')

    src_dir = os.path.join(
        home_dir,
        'real_ws',
        'src',
        'reel_nav'
    )

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

    lidar_localization_params = os.path.join(
        src_dir,
        'config',
        'lidar_localization.yaml'
    )

    # ============================================================
    # LAUNCH CONFIGURATIONS
    # ============================================================

    use_sim_time = LaunchConfiguration(
        'use_sim_time'
    )

    # ============================================================
    # ROBOT DESCRIPTION
    # ============================================================

    with open(sdf_file, 'r') as sdf_handle:
        robot_description_document = xacro.parse(
            sdf_handle
        )

    xacro.process_doc(
        robot_description_document
    )

    robot_description = (
        robot_description_document.toxml()
    )

    # ============================================================
    # NAV2 BRINGUP
    # ============================================================

    nav2_launch = os.path.join(
        get_package_share_directory(
            'nav2_bringup'
        ),
        'launch',
        'bringup_launch.py'
    )

    # ============================================================
    # LAUNCH DESCRIPTION
    # ============================================================

    return LaunchDescription([

        # --------------------------------------------------------
        # ARGUMENTS
        # --------------------------------------------------------

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        # --------------------------------------------------------
        # ROBOT STATE PUBLISHER
        # --------------------------------------------------------

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',

            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        # --------------------------------------------------------
        # JOINT STATE PUBLISHER
        # --------------------------------------------------------

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',

            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        # --------------------------------------------------------
        # WHEEL ENCODER ODOMETRY
        # --------------------------------------------------------

        Node(
            package='reel_evata',
            executable='OdometerListener',
            name='encoder_odom_publisher',
            output='screen',

            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),

        # --------------------------------------------------------
        # LOCAL EKF
        #
        # Inputs:
        #   /lio_sam/mapping/odometry
        #   /teker
        #   /imu/data
        #
        # Output:
        #   /odometry/filtered/local
        #   odom -> base_footprint
        # --------------------------------------------------------

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',

            parameters=[
                ekf_params
            ],

            remappings=[
                (
                    'odometry/filtered',
                    '/odometry/filtered/local'
                ),
            ],
        ),

        # --------------------------------------------------------
        # NAVSAT TRANSFORM
        #
        # Inputs:
        #   /imu/data
        #   /gnss_1/llh_position
        #   /odometry/filtered/local
        #
        # Outputs:
        #   /odometry/gps
        #   /gps/filtered
        # --------------------------------------------------------

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',

            parameters=[
                ekf_params
            ],

            remappings=[
                (
                    'imu',
                    '/imu/data'
                ),
                (
                    'gps/fix',
                    '/gnss_1/llh_position'
                ),
                (
                    'odometry/filtered',
                    '/odometry/filtered/local'
                ),
                (
                    'odometry/gps',
                    '/odometry/gps'
                ),
                (
                    'gps/filtered',
                    '/gps/filtered'
                ),
            ],
        ),

        # --------------------------------------------------------
        # GLOBAL EKF
        #
        # Inputs:
        #   /odometry/filtered/local
        #   /odometry/gps
        #   /pcl_pose
        #
        # Outputs:
        #   /odometry/filtered/global
        #   map -> odom
        # --------------------------------------------------------

  
        # --------------------------------------------------------
        # NAV2 BRINGUP
        #
        # İlk kullandığın bringup_launch.py korunmuştur.
        # --------------------------------------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                nav2_launch
            ),

            launch_arguments={
                'map': map_file,
                'params_file': nav2_params,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
            }.items(),
        ),

        # --------------------------------------------------------
        # RVIZ
        # --------------------------------------------------------

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',

            arguments=[
                '-d',
                rviz_config,
            ],

            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),
    ])

