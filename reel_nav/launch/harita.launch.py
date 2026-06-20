import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    src_dir = dir_path.split('/install')[0]
    sdf_path = os.path.join(src_dir, "src", "reel_nav", 'final_deneme.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_mode = LaunchConfiguration('slam', default='True')

    rviz_config_dir = os.path.join(src_dir, "src", "reel_nav", "config", "nav2_evata_view.rviz")
    lidarslam_param_dir = os.path.join(src_dir, "src", "lidarslam_ros2", "lidarslam", "param", "lidarslam.yaml")
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    slam_params_file = os.path.join(src_dir, "src", "reel_nav", "config", "slam_toolbox_params.yaml")

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(src_dir, "src", "reel_nav", 'map', 'harita.yaml')
    )
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(src_dir, "src", "reel_nav", "config", 'test.yaml')
    )

    # Xacro dosyasını oku ve işle
    doc = xacro.parse(open(sdf_path))
    xacro.process_doc(doc)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'
        ),

        # Sabit dönüşler
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
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': doc.toxml()}]
        ),
        
	Node(
	    package='reel_evata',
	    executable='OdometerListener',
	    name='initial_pose_setter',
	    output='screen',
	    parameters=[{
		'initialpose_topic': '/initialpose',
		'map_frame': 'map',
		'odom_frame': 'odom',
		'base_frame': 'base_footprint',
		'publish_rate': 30.0
	    }]
	),  
        

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': doc.toxml()}]
        ),
        
       	    
        # Nav2 bringup launch dosyasını dahil et
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir
            }.items(),
        ),



        



        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

    ])

