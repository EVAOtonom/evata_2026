from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    def gnome_tab(title, command):
        return ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--tab', '-t', title,
                '--', 'bash', '-c', f'{command}; exec bash'
            ],
            output='screen'
        )

    return LaunchDescription([
        gnome_tab('USB & Aks', 'ros2 run reel_nav usb_settings && ros2 run reel_evata Aks'),
        gnome_tab('RSlidar',      'ros2 launch rslidar_sdk start.py'),
        gnome_tab('cmd_vel',      'ros2 run reel_evata cmd_vel'),
        gnome_tab('LaneDetection','ros2 run reel_evata teknikLaneD'),
        gnome_tab('LidarEngel',    'ros2 run reel_nav lidar_obstacle_detector_with_audio_text'),
    ])
