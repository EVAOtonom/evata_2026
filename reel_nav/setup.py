from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'reel_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['reel_nav', 'reel_nav.*']),  # <-- Burası önemli
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='akif',
    maintainer_email='akif@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'usb_settings = reel_nav.usb_settings:main',
            'pcl_pose_to_covariance = reel_nav.pcl_pose_to_covariance:main',
        ],
    },
)

