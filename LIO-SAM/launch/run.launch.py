import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('lio_sam')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    common_params = [
        params_file,
        {'use_sim_time': use_sim_time},
    ]

    sensor_qos_overrides = {
        'qos.imu.reliability': 'BEST_EFFORT',
        'qos.imu.history': 'KEEP_LAST',
        'qos.imu.depth': 200,

        'qos.lidar.reliability': 'BEST_EFFORT',
        'qos.lidar.history': 'KEEP_LAST',
        'qos.lidar.depth': 2000,

        'qos.odom.reliability': 'BEST_EFFORT',
        'qos.odom.history': 'KEEP_LAST',
        'qos.odom.depth': 200,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share_dir, 'config', 'params.yaml'),
            description='LIO-SAM parametre yaml dosyası'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Gerçek araçta false olmalı'
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='RViz açılsın mı?'
        ),

        # =========================================================
        # STATIC TFs - SADECE SENSOR / ROBOT SABIT DÖNÜŞÜMLERİ
        # =========================================================
        #
        # LIO-SAM şunu yayınlayacak:
        #   odom -> base_footprint
        #
        # Burada sadece şunları yayınlıyoruz:
        #   base_footprint -> base_link
        #   base_link -> rslidar
        #   base_link -> imu_link
        #
        # Sakın burada map->odom veya odom->base_footprint yayınlama.


        # =========================================================
        # LIO-SAM NODES
        # =========================================================

        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=common_params + [sensor_qos_overrides],
            output='screen'
        ),

        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=common_params + [sensor_qos_overrides],
            output='screen'
        ),

        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=common_params,
            output='screen'
        ),

        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=common_params,
            output='screen'
        ),

        # =========================================================
        # RVIZ
        # =========================================================

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
            output='screen'
        ),
    ])
