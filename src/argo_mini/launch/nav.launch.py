"""
Argo Mini — Navigation Launch (SLAM Toolbox localization)
==========================================================
Requires a posegraph map saved during a prior mapping session.
Pass the map base path (without extension) via the map:= argument:

    ros2 launch argo_mini nav.launch.py map:=/home/argo/maps/indoor_map

The robot will relocalize automatically from the first LiDAR scan —
no manual initial pose needed.

Topic pipeline for velocity commands:
  Nav2 controller_server  →  /cmd_vel_raw
  Nav2 velocity_smoother  →  /cmd_vel_raw  →  /cmd_vel_smoothed
  depth_safety_shield     →  /cmd_vel_smoothed  →  /cmd_vel
  serial_bridge           →  /cmd_vel  →  ESP32 motors

Depth-camera integration:
  HP60C SDK  →  /ascamera_hp60c/camera_publisher/depth0/points
  depth_safety_shield:
    • STOP / SLOW / CLEAR state machine on /cmd_vel_smoothed
    • re-publishes /depth_filtered (base_link frame) for Nav2 local costmap

Args:
  map        (required) — base path to .posegraph map (no extension)
  use_camera (default true)  — launch the EAI HP60C camera node
  use_rviz   (default true)  — launch RViz2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg = get_package_share_directory('argo_mini')
    nav2_yaml  = os.path.join(pkg, 'config', 'nav2.yaml')
    slam_yaml  = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    urdf_file  = os.path.join(pkg, 'urdf',   'argo_mini.urdf')

    # Default map path — update after your first mapping session
    default_map = os.path.join(pkg, 'maps', 'indoor_map')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    use_camera = LaunchConfiguration('use_camera', default='true')
    use_rviz   = LaunchConfiguration('use_rviz',   default='true')
    map_path   = LaunchConfiguration('map',         default=default_map)

    # Nav2 lifecycle nodes — slam_toolbox is NOT a lifecycle node; it manages itself
    nav2_nodes = [
        'behavior_server',
        'controller_server',
        'planner_server',
        'velocity_smoother',
        'bt_navigator',
    ]

    return LaunchDescription([
        # ── launch arguments ────────────────────────────────────────────────
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Base path to serialized posegraph map (no .posegraph extension)'),
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='Launch the EAI HP60C depth camera node'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz2 for visualisation'),

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

        # ── 2. Serial Bridge (ESP32 motor control + wheel odometry) ─────────
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
        # frame_id = lidar_link matches the URDF joint child frame so that
        # robot_state_publisher provides the base_link → lidar_link TF that
        # SLAM Toolbox needs for scan-matching.
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

        # ── 4. Scan Relay (LiDAR timestamp correction) ───────────────────────
        Node(
            package='argo_mini',
            executable='scan_relay',
            name='scan_relay',
            output='screen',
        ),

        # ── 5. SLAM Toolbox — localization mode ──────────────────────────────
        # Replaces both map_server and amcl:
        #   • serves /map from the serialized posegraph
        #   • broadcasts map → odom TF via scan-matching (no initial pose needed)
        # map_file_name is overridden here so the launch map:= argument takes effect.
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_yaml,
                {'map_file_name': map_path},
            ],
        ),

        # ── 6. Behavior Server (Spin / BackUp / Wait recoveries) ─────────────
        LifecycleNode(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_yaml],
            remappings=[('cmd_vel', '/cmd_vel_raw')],
        ),

        # ── 7. Controller Server → /cmd_vel_raw (remapped) ───────────────────
        LifecycleNode(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_yaml],
            remappings=[('cmd_vel', '/cmd_vel_raw')],
        ),

        # ── 8. Planner Server ────────────────────────────────────────────────
        LifecycleNode(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_yaml],
        ),

        # ── 9. Velocity Smoother  /cmd_vel_raw → /cmd_vel_smoothed ───────────
        LifecycleNode(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_yaml],
            remappings=[
                ('cmd_vel',          '/cmd_vel_raw'),
                ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
            ],
        ),

        # ── 10. BT Navigator ─────────────────────────────────────────────────
        LifecycleNode(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_yaml],
        ),

        # ── 11. Nav2 Lifecycle Manager ───────────────────────────────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart':    True,
                'node_names':   nav2_nodes,
            }],
        ),

        # ── 12. Depth Safety Shield  /cmd_vel_smoothed → /cmd_vel ────────────
        # Acts as the safety layer between Nav2's smoothed output and the motors.
        # Reads depth PointCloud2, stops/slows the robot if an obstacle is close,
        # and re-publishes a filtered cloud on /depth_filtered for the costmap.
        Node(
            package='argo_mini',
            executable='depth_safety_shield',
            name='depth_safety_shield',
            output='screen',
            parameters=[{
                'stop_distance':       0.35,
                'slow_distance':       0.65,
                'slow_factor':         0.40,
                'lateral_margin':      0.28,
                'min_obstacle_height': 0.05,
                'max_obstacle_height': 1.60,
                'depth_timeout':       3.0,
                'downsample_stride':   4,
                'input_topic':  '/cmd_vel_smoothed',
                'output_topic': '/cmd_vel',
                'depth_topic':
                    '/ascamera_hp60c/camera_publisher/depth0/points',
            }],
        ),

        # ── 13. Camera static TF bridge ──────────────────────────────────────
        # HP60C SDK publishes depth0/points with frame_id: ascamera_hp60c_color_0
        # Our URDF defines camera_depth_optical_frame at the same physical location.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_bridge',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0',
                'camera_depth_optical_frame',
                'ascamera_hp60c_color_0',
            ],
        ),

        # ── 14. HP60C Depth Camera (optional) ───────────────────────────────
        GroupAction(
            condition=IfCondition(use_camera),
            actions=[
                Node(
                    package='ascamera',
                    executable='ascamera_node',
                    name='ascamera_hp60c',
                    output='screen',
                    parameters=[{'camera_type': 'hp60c'}],
                ),
            ],
        ),

        # ── 15. RViz2 (optional) ─────────────────────────────────────────────
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
