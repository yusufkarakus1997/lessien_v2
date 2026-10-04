"""Only GenericSystem, fixed namespace; no selectable hardware plugin."""
from pathlib import Path
import math
import os
import tempfile
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description = Path(get_package_share_directory('lessien_description'))
    bringup = Path(get_package_share_directory('lessien_bringup'))
    defaults = yaml.safe_load((description / 'config/geometry.yaml').read_text())

    def setup(context):
        values = {key: LaunchConfiguration(key).perform(context) for key in defaults}
        for key, raw in values.items():
            v = float(raw)
            if not math.isfinite(v):
                raise ValueError(f'Parameter {key} must be finite (got {raw})')
            if not key.startswith('imu_mount_') and v <= 0:
                raise ValueError(f'Size/mass parameter {key} must be positive (got {v})')
        robot = xacro.process_file(str(description / 'urdf/lessien.urdf.xacro'),
                                   mappings={**values, 'mock_control': 'true'}).toxml()
        config = yaml.safe_load((bringup / 'config/controllers.yaml').read_text())
        params = config['/lessien_mock/diff_drive_controller']['ros__parameters']
        params['wheel_separation'] = float(values['wheel_separation'])
        params['wheel_radius'] = float(values['wheel_radius'])
        # Spawner passes this exact config to the controller nodes as well.
        with tempfile.NamedTemporaryFile(mode='w', prefix='lessien_mock_', suffix='.yaml',
                                         delete=False) as stream:
            yaml.safe_dump(config, stream)
            config_path = stream.name

        def cleanup(context):
            if os.path.exists(config_path):
                os.unlink(config_path)
            return []

        remaps = [('/tf', '/lessien_mock/tf'), ('/tf_static', '/lessien_mock/tf_static')]
        return [
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 namespace='lessien_mock', parameters=[{'robot_description': robot,
                 'frame_prefix': 'lessien_mock/'}], remappings=remaps, output='screen'),
            Node(package='controller_manager', executable='ros2_control_node',
                 namespace='lessien_mock', parameters=[config_path],
                 output='screen'),
            Node(package='controller_manager', executable='spawner', namespace='lessien_mock',
                 arguments=['joint_state_broadcaster', 'diff_drive_controller',
                            '-c', '/lessien_mock/controller_manager', '-p', config_path,
                            '--activate-as-group', '--controller-manager-timeout', '30',
                            '--controller-ros-args', '-r /tf:=/lessien_mock/tf -r /tf_static:=/lessien_mock/tf_static'],
                 output='screen'),
        ]

    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=str(v)) for k, v in defaults.items()],
        OpaqueFunction(function=setup),
    ])
