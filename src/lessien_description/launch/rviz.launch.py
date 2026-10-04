"""Laptop viewer for the Pi's mock model. No controllers or command publishers."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('lessien_description')) / 'rviz/lessien_mock.rviz'
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2', name='rviz2',
             namespace='lessien_mock', arguments=['-d', str(config)],
             parameters=[{'use_sim_time': False}],
             remappings=[('/tf', '/lessien_mock/tf'),
                         ('/tf_static', '/lessien_mock/tf_static')],
             output='screen'),
    ])
