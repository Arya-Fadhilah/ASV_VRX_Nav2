from setuptools import setup
import os
from glob import glob

package_name = 'asv_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bima Arya Fadhilah',
    maintainer_email='bimaaryafadhilah@gmail.com',
    description='Alokasi thrust dan odometry untuk ASV di VRX',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'thrust_allocator = asv_control.thrust_allocator:main',
            'odom_from_sim    = asv_control.odom_from_sim:main',
            'gps_waypoint_follower = asv_control.gps_waypoint_follower:main',
            'gps_waypoint_interactive = asv_control.gps_waypoint_interactive:main',
        ],
    },
)
