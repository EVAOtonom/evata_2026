import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    src_dir = dir_path.split('/install')[0]

    sdf_path = os.path.join(src_dir, "src", "reel_nav", "final_deneme.sdf")
    ekf_params = os.path.join(src_dir, "src", "reel_nav", "config", "ekf.yaml")
    

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    doc = xacro.parse(open(sdf_path))
    xacro.process_doc(doc)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        # map -> odom sabit TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_odom',
            output='log',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # joint_state_publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': doc.toxml()
            }]
        ),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': doc.toxml()
            }]
        ),

        # NavSat Transform
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[ekf_params],
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/gnss_1/llh_position'),
                ('odometry/filtered', '/teker'),
                ('odometry/gps', '/odometry/gps'),
                ('gps/filtered', '/gps/filtered')
            ]
        ),

    ])
