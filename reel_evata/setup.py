from setuptools import find_packages, setup

package_name = 'reel_evata'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akif',
    maintainer_email='akifdlk58svs@gmail.com',
    description='ROS 2 Package for CAN Bus and Brake Control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'OdometerListener = reel_evata.OdometerListener:main',
            'amcl_pose_filter_tf = reel_evata.amcl_pose_filter_tf:main',
            'AKS_communication = reel_evata.AKS_communication:main',
            'Aks = reel_evata.Aks:main',
            'stabilThrottle = reel_evata.stabilThrottle:main',
            'FakeGps = reel_evata.FakeGps:main',
            'cmd_vel = reel_evata.cmd_vel:main',
            "laneDetection=" + package_name + ".laneDetection:main",
            'toggle_brake= reel_evata.toggle_brake:main',
            'gps_logger= reel_evata.gps_logger:main',
            "serit=" + package_name + ".serit:main",		
            "mid_lane=" + package_name + ".mid_lane:main",
            "lidar_obstacle_detector_with_audio_text=" + package_name + ".lidar_obstacle_detector_with_audio_text:main",
            "laneFollower=" + package_name + ".laneFollower:main",
            "sign_detector=" + package_name + ".sign_detector:main",
            "reel_gps=" + package_name + ".reel_gps:main",
            "bettersigndetector=" + package_name + ".bettersigndetector:main",
            "new2_control=" + package_name + ".new2_control:main",
            "levha_control=" + package_name + ".levha_control:main",
            "sign_handler=" + package_name + ".sign_handler:main",
            "zedimu=" + package_name + ".zedimu:main",
            'movement = reel_evata.movement:main',
            "teknikLaneD=" + package_name + ".teknikLaneD:main",
            "tek_control=" + package_name + ".tek_control:main",	
            "sag_engel_control=" + package_name + ".sag_engel_control:main",
            "fake_lanes=" + package_name + ".fake_lanes:main",
            "clickedP=" + package_name + ".clickedP:main",	
            "clicked_point_republisher=" + package_name + ".clicked_point_republisher:main",
            "logger=" + package_name + ".logger:main",		
            "final2_park1=" + package_name + ".final2_park1:main",	
            "final2_park2=" + package_name + ".final2_park2:main",	
            "final2_park3=" + package_name + ".final2_park3:main",	




        ],
    },
)
