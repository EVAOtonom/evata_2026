import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    dir_path = os.path.dirname(os.path.realpath(__file__))
    src_dir = dir_path.split('/install')[0]

    sdf_path = os.path.join(
        src_dir,
        "src",
        "reel_nav",
        "final_deneme.sdf"
    )

    workspace_setup = os.path.join(
        src_dir,
        "install",
        "setup.bash"
    )

    use_sim_time = LaunchConfiguration(
        'use_sim_time',
        default='false'
    )

    rviz_config_dir = os.path.join(
        src_dir,
        "src",
        "reel_nav",
        "config",
        "nav2_evata_view.rviz"
    )

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    ekf_params_file = os.path.join(
        src_dir,
        "src",
        "reel_nav",
        "config",
        "ekf.yaml"
    )

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            src_dir,
            "src",
            "reel_nav",
            "map",
            "harita.yaml"
        )
    )

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            src_dir,
            "src",
            "reel_nav",
            "config",
            "test.yaml"
        )
    )

    # Xacro dosyasını oku ve işle
    doc = xacro.parse(open(sdf_path))
    xacro.process_doc(doc)

    # ============================================================
    # GNOME TERMINAL YENİ SEKME FONKSİYONU
    # ============================================================

    def gnome_tab(title, command):

        full_command = (
            'source /opt/ros/humble/setup.bash && '
            f'source "{workspace_setup}" && '
            f'{command}'
        )

        return ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--tab',
                '-t',
                title,
                '--',
                'bash',
                '-c',
                f'{full_command}; exec bash'
            ],
            output='screen'
        )

    # ============================================================
    # TERMİNAL SEKMELERİNDE ÇALIŞACAK KOMUTLAR
    # ============================================================

    # 1. AKS
    aks = gnome_tab(
        'AKS',
        'ros2 run reel_evata Aks'
    )

    # 2. MICROSTRAIN IMU
    microstrain = gnome_tab(
        'MICROSTRAIN IMU',
        'ros2 launch microstrain_inertial_driver microstrain_launch.py'
    )

    # 3. RSLIDAR
    rslidar = gnome_tab(
        'RSLIDAR',
        'ros2 launch rslidar_sdk start.py'
    )

    # 4. LIO-SAM
    lio_sam = gnome_tab(
        'LIO-SAM',
        'ros2 launch lio_sam run.launch.py rviz:=false'
    )

    # 5. LIDAR LOCALIZATION
    lidar_localization = gnome_tab(
        'LIDAR LOCALIZATION',
        'ros2 launch lidar_localization_ros2 '
        'lidar_localization.launch.py'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                src_dir,
                "src",
                "reel_nav",
                "map",
                "harita.yaml"
            ),
            description='Full path to map file to load'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                src_dir,
                "src",
                "reel_nav",
                "config",
                "test.yaml"
            ),
            description='Full path to param file to load'
        ),

        # ============================================================
        # TERMİNAL SEKMELERİNİ SIRASIYLA AÇ
        # ============================================================

        # 1. AKS hemen açılır
        aks,

        # 2. Microstrain 
        TimerAction(
            period=0.1,
            actions=[
                microstrain
            ]
        ),

        TimerAction(
            period=0.2,
            actions=[
                rslidar
            ]
        ),

        # 4. LIO-SAM 
        TimerAction(
            period=0.3,
            actions=[
                lio_sam
            ]
        ),

        # 5. Lidar localization 
        TimerAction(
            period=0.4,
            actions=[
                lidar_localization
            ]
        ),

        # ============================================================
        # SABİT DÖNÜŞLER
        # ============================================================

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_map_to_odom',
        #     output='log',
        #     arguments=[
        #         '0', '0', '0',
        #         '0', '0', '0',
        #         'map', 'odom'
        #     ]
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_odom_to_base',
        #     output='log',
        #     arguments=[
        #         '0', '0', '0',
        #         '0', '0', '0',
        #         'odom', 'base_footprint'
        #     ]
        # ),

        # ============================================================
        # JOINT STATE PUBLISHER
        # ============================================================

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': doc.toxml()
                }
            ]
        ),

        # ============================================================
        # ROBOT STATE PUBLISHER
        # ============================================================

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': doc.toxml()
                }
            ]
        ),

        # ============================================================
        # POINTCLOUD TO LASERSCAN
        # ============================================================

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_nav',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'target_frame': 'base_footprint',
                    'transform_tolerance': 0.10,
                    'min_height': -0.05,
                    'max_height': 1.80,
                    'angle_min': -3.141592653589793,
                    'angle_max': 3.141592653589793,
                    'angle_increment': 0.008726646259972,
                    'scan_time': 0.105,
                    'range_min': 0.50,
                    'range_max': 12.0,
                    'use_inf': True
                }
            ],
            remappings=[
                ('cloud_in', '/rslidar_points'),
                ('scan', '/scan_nav')
            ]
        ),

        # ============================================================
        # NAV2 BRINGUP
        # ============================================================

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_launch_file_dir,
                    'bringup_launch.py'
                )
            ),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'autostart': 'true'
            }.items()
        ),

        # ============================================================
        # LOCAL EKF
        # ============================================================

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[
                ekf_params_file,
                {
                    'use_sim_time': use_sim_time
                }
            ],
            remappings=[
                ('odometry/filtered', '/odom')
            ],
            arguments=[
                '--ros-args',
                '--log-level',
                'info'
            ]
        ),

        # ============================================================
        # GLOBAL EKF
        # ============================================================

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[
                ekf_params_file,
                {
                    'use_sim_time': use_sim_time
                }
            ],
            remappings=[
                ('odometry/filtered', '/odometry/global')
            ]
        ),

        # ============================================================
        # NAVSAT TRANSFORM
        # ============================================================

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                ekf_params_file,
                {
                    'use_sim_time': use_sim_time
                }
            ],
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/gnss_1/llh_position'),
                ('odometry/filtered', '/odometry/global'),
                ('odometry/gps', '/odometry/gps'),
                ('gps/filtered', '/gps/filtered')
            ],
            arguments=[
                '--ros-args',
                '--log-level',
                'info'
            ]
        ),

        # ============================================================
        # ENCODER ODOMETRİ
        # ============================================================

        Node(
            package='reel_evata',
            executable='OdometerListener',
            name='encoder_odom_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time
                }
            ]
        ),

        # ============================================================
        # ZED NODE
        # ============================================================

        # Node(
        #     package='zed_wrapper',
        #     executable='zed_node',
        #     name='zed_node',
        #     output='screen',
        #     parameters=[
        #         {
        #             'use_sim_time': use_sim_time
        #         }
        #     ]
        # ),

        # ============================================================
        # RVIZ
        # ============================================================

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                rviz_config_dir
            ],
            parameters=[
                {
                    'use_sim_time': use_sim_time
                }
            ],
            output='screen'
        ),

    ])
