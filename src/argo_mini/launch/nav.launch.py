"""
Argo Mini — full navigation launch
===================================
Topic pipeline for velocity commands:

  Nav2 controller_server  →  /cmd_vel_raw
  Nav2 velocity_smoother  →  /cmd_vel_raw  →  /cmd_vel_smoothed
  depth_safety_shield     →  /cmd_vel_smoothed  →  /cmd_vel
  serial_bridge           →  /cmd_vel  →  ESP32 motors

Depth-camera integration:
  HP60C SDK publishes  /ascamera_hp60c/camera_publisher/depth0/points
  depth_safety_shield:
    • filters forward zone, re-publishes  /depth_filtered  (base_link frame)
    • STOP / SLOW / CLEAR state machine on /cmd_vel_smoothed
  Nav2 local_costmap subscribes /depth_filtered  →  proactive planning

Args:
  use_camera   (default true)  — launch the EAI HP60C camera node
  use_rviz     (default true)  — launch RViz2

IMPORTANT — camera SDK frame_id:
  The HP60C SDK publishes its point cloud with a frame_id that may differ
  from the URDF's "camera_depth_optical_frame".
  Check the actual frame_id with:
      ros2 topic echo /ascamera_hp60c/camera_publisher/depth0/points --once | grep frame_id
  If it differs, update the static_transform_publisher args below so that
  TF can bridge from the SDK frame to base_link.
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
    map_yaml   = os.path.join(pkg, 'maps',   'indoor_map.yaml')
    urdf_file  = os.path.join(pkg, 'urdf',   'argo_mini.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    use_camera = LaunchConfiguration('use_camera', default='true')
    use_rviz   = LaunchConfiguration('use_rviz',   default='true')

    # Nav2 lifecycle nodes managed by lifecycle_manager_nav
    nav2_nodes = [
        'map_server',
        'amcl',
        'behavior_server',
        'controller_server',
        'planner_server',
        'velocity_smoother',
        'bt_navigator',
    ]

    return LaunchDescription([
        # ── launch arguments ────────────────────────────────────────────────
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
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port':   '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id':       'laser',
                'inverted':       False,
                'angle_compensate': True,
                'scan_mode':      'Standard',
            }],
        ),

        # ── 4. Scan Relay (LiDAR timestamp correction) ───────────────────────
        Node(
            package='argo_mini',
            executable='scan_relay',
            name='scan_relay',
            output='screen',
        ),

        # ── 5. Map Server ────────────────────────────────────────────────────
        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_yaml, {'yaml_filename': map_yaml}],
        ),

        # ── 6. AMCL (Monte-Carlo localisation) ──────────────────────────────
        LifecycleNode(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml],
        ),

        # ── 7. Controller Server → /cmd_vel_raw (remapped) ──────────────────
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

        # ── 9. Velocity Smoother  /cmd_vel_raw → /cmd_vel_smoothed ──────────
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

        # ── 11. Behavior Server (Spin / BackUp / Wait recoveries) ───────────
        LifecycleNode(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_yaml],
            remappings=[('cmd_vel', '/cmd_vel_raw')],
        ),

        # ── 12. Nav2 Lifecycle Manager ───────────────────────────────────────
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

        # ── 13. Depth Safety Shield  /cmd_vel_smoothed → /cmd_vel ────────────
        # Acts as the safety layer between Nav2's smoothed output and the motors.
        # Reads depth PointCloud2, stops/slows the robot if an obstacle is close,
        # and re-publishes a filtered cloud on /depth_filtered for the costmap.
        Node(
            package='argo_mini',
            executable='depth_safety_shield',
            name='depth_safety_shield',
            output='screen',
            parameters=[{
                'stop_distance':       0.35,   # m — hard stop
                'slow_distance':       0.65,   # m — start scaling
                'slow_factor':         0.40,   # fraction of linear velocity
                'lateral_margin':      0.28,   # m — half-width of forward zone
                'min_obstacle_height': 0.05,   # m — ignore floor
                'max_obstacle_height': 1.60,   # m — ignore ceiling
                'depth_timeout':       3.0,    # s — stale-data window
                'downsample_stride':   4,      # process every 4th point
                'input_topic':  '/cmd_vel_smoothed',
                'output_topic': '/cmd_vel',
                'depth_topic':
                    '/ascamera_hp60c/camera_publisher/depth0/points',
            }],
        ),

        # ── 14. Camera static TF bridge ──────────────────────────────────────
        # HP60C SDK publishes depth0/points with frame_id: ascamera_hp60c_color_0
        # Our URDF defines camera_depth_optical_frame at the same physical location.
        # Identity transform bridges the two so TF lookups succeed.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_bridge',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0',
                'camera_depth_optical_frame',   # parent — in URDF, child of camera_link
                'ascamera_hp60c_color_0',        # child  — frame_id published by SDK
            ],
        ),

        # ── 15. HP60C Depth Camera (optional) ───────────────────────────────
        # Only launched when use_camera:=true.
        # The EAI SDK provides its own ROS2 node; adjust package/executable names
        # to match your SDK build.  The SDK node must be sourced separately:
        #   source ~/EaiCameraSdk_v1.2.28.20241015/demo/linux_ros/ros2/install/setup.bash
        GroupAction(
            condition=IfCondition(use_camera),
            actions=[
                Node(
                    package='ascamera',
                    executable='ascamera_node',
                    name='ascamera_hp60c',
                    output='screen',
                    parameters=[{
                        'camera_type': 'hp60c',
                    }],
                ),
            ],
        ),

        # ── 16. RViz2 (optional) ─────────────────────────────────────────────
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
