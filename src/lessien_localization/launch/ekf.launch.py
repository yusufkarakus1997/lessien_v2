"""Launch robot_localization ekf_node with lessien wheel-odom + IMU fusion config.

EKF is the SOLE owner of the odom -> base_footprint TF transform.
lessien_odometry /wheel_odom node does NOT broadcast this TF edge.

Output topic: /odometry/filtered
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    loc_share = Path(get_package_share_directory('lessien_localization'))
    ekf_yaml = str(loc_share / 'config' / 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_yaml],
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ],
        ),
    ])
