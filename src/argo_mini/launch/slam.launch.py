"""
Argo Mini — Mapping Session Launch
====================================
Run this ONCE to build the posegraph map of your venue.
Navigate every area the robot will ever need to reach.

Usage:
    ros2 launch argo_mini slam.launch.py

Saving the map (two options):
  A) RViz plugin  →  "Serialize Map" button  →  enter output path
  B) CLI service:
       ros2 service call /slam_toolbox/serialize_map \\
           slam_toolbox/srv/SerializePoseGraph \\
           "{filename: '/home/argo/maps/indoor_map'}"

The service creates two files:
    indoor_map.posegraph
    indoor_map.data

Pass the same base path (without extension) to nav.launch.py:
    ros2 launch argo_mini nav.launch.py map:=/home/argo/maps/indoor_map

Do NOT use ros2 run nav2_map_server map_saver — that saves a flat PGM which
SLAM Toolbox localization cannot use.  Always use the serialize service.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('argo_mini')
    slam_mapping_yaml = os.path.join(pkg, 'config', 'slam_mapping.yaml')
    urdf_file         = os.path.join(pkg, 'urdf',   'argo_mini.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    use_rviz = LaunchConfiguration('use_rviz', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz2 to visualise the map being built'),

        # ── 1. Robot State Publisher (URDF → TF tree) ───────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': False,
            }],
        ),

        # ── 2. Serial Bridge (wheel odometry needed for SLAM) ────────────────
        Node(
            package='argo_mini',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB1',
                'baud': 115200,
            }],
        ),

        # ── 3. RPLidar A1 ────────────────────────────────────────────────────
        # frame_id must match the URDF joint child frame so robot_state_publisher
        # can provide the base_link → lidar_link TF that SLAM Toolbox needs.
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port':      '/dev/ttyUSB0',
                'serial_baudrate':  115200,
                'frame_id':         'lidar_link',
                'inverted':         False,
                'angle_compensate': True,
                'scan_mode':        'Standard',
            }],
        ),

        # ── 4. Scan Relay (timestamp correction) ─────────────────────────────
        Node(
            package='argo_mini',
            executable='scan_relay',
            name='scan_relay',
            output='screen',
        ),

        # ── 5. SLAM Toolbox — async mapping ──────────────────────────────────
        # async = non-blocking; the node processes scans as fast as it can
        # without stalling the ROS executor.
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_mapping_yaml],
        ),

        # ── 6. RViz2 (optional) — load SLAM Toolbox plugin for Serialize button
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
