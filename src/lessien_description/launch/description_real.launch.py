"""Production real-robot description. Publishes fixed transforms on global /tf_static
from base_footprint downwards. No controllers, no mock namespace, no tf remaps."""
from pathlib import Path
import math
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('lessien_description'))
    defaults = yaml.safe_load((share / 'config' / 'geometry.yaml').read_text())

    def setup(context):
        values = {key: LaunchConfiguration(key).perform(context) for key in defaults}
        for key, raw in values.items():
            v = float(raw)
            if not math.isfinite(v):
                raise ValueError(f'Parameter {key} must be finite (got {raw})')
            if not key.startswith('imu_mount_') and v <= 0:
                raise ValueError(f'Size/mass parameter {key} must be positive (got {v})')
        robot = xacro.process_file(
            str(share / 'urdf' / 'lessien.urdf.xacro'),
            mappings=values,
        ).toxml()
        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot}],
            ),
        ]

    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=str(v)) for k, v in defaults.items()],
        OpaqueFunction(function=setup),
    ])
