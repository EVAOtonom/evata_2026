from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # gnome_tab fonksiyonu terminalde yeni sekme açmak için yardımcıdır
    def gnome_tab(title, command):
        return ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--tab', '-t', title,
                '--', 'bash', '-c', f'{command}; exec bash'
            ],
            output='screen'
        )

    # Sadece ZED2i başlatma komutu
    zed2i = gnome_tab(
        'ZED2i',
        'ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i'
    )

    return LaunchDescription([
        zed2i
    ])
