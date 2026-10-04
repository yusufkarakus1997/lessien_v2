"""Production real-robot bringup: description + wheel odometry + EKF.

Orchestrates the three production-only sub-launch files on the global
namespace.  No mock system, no namespace, no tf remap, no frame prefix.

TF ownership contract (preserved verbatim from sub-launches):
  * robot_state_publisher (description_real)
      - base_footprint -> base_link   (fixed, /tf_static)
      - base_link       -> imu_link    (fixed, /tf_static)
      - base_link       -> 4x wheel    (continuous, /tf from /joint_states)
  * ekf_filter_node (ekf)
      - odom            -> base_footprint  (sole owner, /tf)
  * wheel_joint_state_node
      - /joint_states ONLY, no TF broadcast
  * lessien_wheel_odometry
      - /wheel_odom ONLY, no TF broadcast

This file is PRODUCTION ONLY.  Do NOT import / include:
  - lessien_description/description.launch.py   (mock variant)
  - lessien_bringup/mock.launch.py
  - any lessien_mock namespace / GenericSystem ros2_control mock
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory('lessien_description')
    odom_share = get_package_share_directory('lessien_odometry')
    loc_share = get_package_share_directory('lessien_localization')

    # Declare passthrough launch args we forward explicitly.
    # Only the ones the user may want to override at the top-level; all
    # other geometry/default values flow through the sub-launch defaults
    # (geometry.yaml SSoT, odometry.yaml defaults).
    args = [
        DeclareLaunchArgument(
            'rpm_filter_window',
            default_value='5',
            description='Moving-average RPM filter window size (>=1). '
                        'Passed through to lessien_odometry/odometry.launch.py.',
        ),
    ]

    # 1) Production robot state publisher:
    #    - global /robot_state_publisher
    #    - no frame_prefix, no namespace, no tf remap
    #    - publishes /tf_static (base_footprint -> base_link -> imu_link)
    #    - publishes /tf (base_link -> 4x wheels) driven by /joint_states
    include_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([desc_share, 'launch', 'description_real.launch.py'])
        ),
    )

    # 2) Wheel odometry + joint state publisher:
    #    - /lessien_wheel_odometry  -> publishes /wheel_odom, NO TF
    #    - /wheel_joint_state_node  -> publishes /joint_states, NO TF
    #    - rpm_filter_window top-level override forwarded verbatim
    include_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([odom_share, 'launch', 'odometry.launch.py'])
        ),
        launch_arguments={
            'rpm_filter_window': LaunchConfiguration('rpm_filter_window'),
        }.items(),
    )

    # 3) EKF filter (wheel-odom + IMU fusion):
    #    - /ekf_filter_node
    #    - SOLE owner of the odom -> base_footprint TF edge
    #    - publishes /odometry/filtered
    include_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([loc_share, 'launch', 'ekf.launch.py'])
        ),
    )

    # 4) EKF Signed Longitudinal Trip Distance
    #    - consumes /odometry/filtered (nav_msgs/Odometry) ONLY
    #    - publishes /lessien/ekf_distance_m (std_msgs/Float32)
    #    - equation: distance_m += twist.twist.linear.x * dt
    #    - forward (vx>0) -> distance increases; backward (vx<0) -> distance decreases
    #    - NO EKF config change, NO TF change, NO wheel odom change
    node_trip_distance = Node(
        package='lessien_odometry',
        executable='ekf_trip_distance_node',
        name='ekf_trip_distance_node',
        output='screen',
        parameters=[{
            'max_dt_sec': 0.5,
            'deadband_vx_mps': 0.0,
        }],
    )

    return LaunchDescription(args + [
        include_description,
        include_odometry,
        include_ekf,
        node_trip_distance,
    ])
