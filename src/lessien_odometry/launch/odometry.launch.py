"""Launch lessien_wheel_odometry node.

Geometry (wheel_radius, wheel_separation=track_width) is sourced from
lessien_description/config/geometry.yaml (single source of truth) and
injected as ROS parameters, overriding the package-local odometry.yaml
defaults so there is no duplicated authoritative geometry config.
"""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = Path(get_package_share_directory('lessien_description'))
    odom_share = Path(get_package_share_directory('lessien_odometry'))

    geometry = yaml.safe_load((desc_share / 'config/geometry.yaml').read_text())
    # SSoT mapping: user-facing yaml field name -> ROS parameter name
    geo_to_param = {
        'wheel_radius':      'wheel_radius',
        'wheel_separation':  'track_width',
    }

    args = [
        DeclareLaunchArgument('wheel_radius',
                              default_value=str(geometry['wheel_radius'])),
        DeclareLaunchArgument('track_width',
                              default_value=str(geometry['wheel_separation'])),
        DeclareLaunchArgument('rpm_zero_threshold', default_value='1.0'),
        DeclareLaunchArgument('rpm_filter_window',  default_value='5'),
        DeclareLaunchArgument('max_dt',              default_value='0.25'),
    ]

    def setup(context):
        # Base node parameters from odometry.yaml (frames, topic names, defaults)
        default_params = yaml.safe_load(
            (odom_share / 'config/odometry.yaml').read_text())['lessien_wheel_odometry']['ros__parameters']
        # Merge SSoT geometry + filter params from launch args (top override)
        user_params = {
            'wheel_radius':       float(LaunchConfiguration('wheel_radius').perform(context)),
            'track_width':        float(LaunchConfiguration('track_width').perform(context)),
            'rpm_zero_threshold': float(LaunchConfiguration('rpm_zero_threshold').perform(context)),
            'rpm_filter_window':  int(LaunchConfiguration('rpm_filter_window').perform(context)),
            'max_dt':             float(LaunchConfiguration('max_dt').perform(context)),
        }
        merged = {**default_params, **user_params}

        return [
            Node(
                package='lessien_odometry',
                executable='wheel_odometry_node',
                name='lessien_wheel_odometry',
                parameters=[merged],
                output='screen',
            ),
            Node(
                package='lessien_odometry',
                executable='wheel_joint_state_node',
                name='wheel_joint_state_node',
                output='screen',
            ),
        ]

    return LaunchDescription(args + [OpaqueFunction(function=setup)])
