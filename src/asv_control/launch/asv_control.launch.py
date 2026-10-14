import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('asv_control'),
                       'config', 'asv_control.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_tf   = LaunchConfiguration('publish_tf')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
            description='Wajib true saat jalan dengan Gazebo'),
        DeclareLaunchArgument('publish_tf', default_value='true',
            description='true = odom dari ground truth sim; '
                        'false = ORB-SLAM3 yang mengisi TF'),

        Node(package='asv_control', executable='thrust_allocator',
             name='thrust_allocator', output='screen',
             parameters=[cfg, {'use_sim_time': use_sim_time}]),

        Node(package='asv_control', executable='odom_from_sim',
             name='odom_from_sim', output='screen',
             parameters=[cfg, {'use_sim_time': use_sim_time,
                               'publish_tf': publish_tf}]),
    ])
