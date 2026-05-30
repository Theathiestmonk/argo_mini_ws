#!/bin/bash
# Argo Mini — full nav stack with depth camera safety shield
#
# cmd_vel pipeline:
#   controller_server  →  /cmd_vel_nav
#   velocity_smoother  →  /cmd_vel_smoothed
#   depth_safety_shield→  /cmd_vel
#   serial_bridge      →  ESP32
#
# Usage:
#   ./start_argo_nav.sh           # with camera + RViz
#   ./start_argo_nav.sh --no-cam  # skip camera (lidar-only)

NO_CAM=false
for arg in "$@"; do
  [[ "$arg" == "--no-cam" ]] && NO_CAM=true
done

# ── environment ────────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source ~/argo_mini_ws/install/setup.bash

CAMERA_SDK_PATH=~/EaiCameraSdk_v1.2.28.20241015/demo/linux_ros/ros2
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CAMERA_SDK_PATH/ascamera/libs/lib/aarch64-linux-gnu

NAV_CONFIG=~/argo_mini_ws/install/argo_mini/share/argo_mini/config/nav2.yaml
MAP_FILE=~/argo_mini_ws/src/argo_mini/maps/indoor_map.yaml

# ── USB permissions (no password needed with udev rule)
# One-time setup: echo 'SUBSYSTEM=="tty",ATTRS{idVendor}=="10c4",MODE="0666"' | sudo tee /etc/udev/rules.d/99-esp32.rules && sudo udevadm control --reload
chmod 666 /dev/ttyUSB0 /dev/ttyUSB1 2>/dev/null || \
  sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1 2>/dev/null || true

# ── cleanup previous run ───────────────────────────────────────────────────
echo "[argo] Killing previous processes..."
for proc in slam_toolbox serial_bridge rplidar_composition rviz2 \
            map_server amcl planner_server controller_server \
            bt_navigator velocity_smoother scan_relay \
            robot_state_publisher depth_safety_shield ascamera_node; do
  pkill -9 -f "$proc" 2>/dev/null || true
done
sleep 3

# ── lifecycle helper: configure then activate a nav2 node ─────────────────
# Usage: lc_node <node_name>
lc_node() {
  local node=$1
  echo "[argo]   lifecycle configure $node..."
  ros2 lifecycle set "$node" configure 2>&1 | tail -1
  sleep 1
  echo "[argo]   lifecycle activate  $node..."
  ros2 lifecycle set "$node" activate  2>&1 | tail -1
  sleep 1
}

# ── 1. Robot state publisher ───────────────────────────────────────────────
echo "[argo] Starting robot_state_publisher..."
ros2 launch argo_mini robot_state_publisher.launch.py &
RSP_PID=$!
sleep 2

# ── 1b. Camera TF bridge (always publish, camera need not be running) ──────
# Links SDK frame (ascamera_hp60c_color_0) to URDF's camera_depth_optical_frame.
# Safe to run even without the camera — no cloud means no lookups happen.
echo "[argo] Starting camera TF bridge (camera_depth_optical_frame → ascamera_hp60c_color_0)..."
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id depth_camera_optical_frame \
  --child-frame-id ascamera_hp60c_camera_link_0 &
CAM_TF_PID=$!
sleep 1

# ── 2. Serial bridge (ESP32 motors + odometry) ────────────────────────────
echo "[argo] Starting serial_bridge..."
ros2 run argo_mini serial_bridge --ros-args \
  -p port:=/dev/ttyUSB1 -p baud:=115200 \
  -p left_tick_scale:=2.031 &
SERIAL_PID=$!
sleep 3

# ── 3. RPLidar A1 ─────────────────────────────────────────────────────────
echo "[argo] Starting rplidar..."
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baudrate:=115200 \
  -p frame_id:=lidar_link \
  -p angle_compensate:=true \
  -p scan_mode:=Boost &
LIDAR_PID=$!
sleep 3

# ── 4. Scan relay (timestamp correction) ──────────────────────────────────
echo "[argo] Starting scan_relay..."
ros2 run argo_mini scan_relay &
RELAY_PID=$!
sleep 2

# map→odom is published by AMCL — no static TF here (they conflict)
TF_PID=""

# ── 6. Map server ─────────────────────────────────────────────────────────
echo "[argo] Starting map_server..."
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$MAP_FILE -p use_sim_time:=false &
MAP_PID=$!
sleep 3
lc_node /map_server

# ── 7. AMCL ───────────────────────────────────────────────────────────────
echo "[argo] Starting AMCL..."
ros2 run nav2_amcl amcl --ros-args \
  --params-file $NAV_CONFIG \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p global_frame_id:=map \
  -p scan_topic:=/scan_corrected &
AMCL_PID=$!
sleep 3
lc_node /amcl

# ── 8. Planner server ─────────────────────────────────────────────────────
echo "[argo] Starting planner_server..."
ros2 run nav2_planner planner_server --ros-args --params-file $NAV_CONFIG &
PLANNER_PID=$!
sleep 3
lc_node /planner_server

# ── 9. Controller server → /cmd_vel_nav ───────────────────────────────────
echo "[argo] Starting controller_server..."
ros2 run nav2_controller controller_server --ros-args \
  --params-file $NAV_CONFIG \
  -r cmd_vel:=/cmd_vel_nav &
CONTROLLER_PID=$!
sleep 3
lc_node /controller_server

# ── 10. Velocity smoother  /cmd_vel_nav → /cmd_vel_smoothed ───────────────
echo "[argo] Starting velocity_smoother..."
ros2 run nav2_velocity_smoother velocity_smoother --ros-args \
  --params-file $NAV_CONFIG \
  -r cmd_vel:=/cmd_vel_nav \
  -r cmd_vel_smoothed:=/cmd_vel_smoothed &
SMOOTHER_PID=$!
sleep 3
lc_node /velocity_smoother

# ── 11. Behavior server (must be active before bt_navigator loads the BT) ──
echo "[argo] Starting behavior_server..."
ros2 run nav2_behaviors behavior_server --ros-args --params-file $NAV_CONFIG &
BEHAVIOR_PID=$!
sleep 3
lc_node /behavior_server

# ── 11b. BT Navigator (loads BT XML — spin/backup/wait must already exist) ─
echo "[argo] Starting bt_navigator..."
ros2 run nav2_bt_navigator bt_navigator --ros-args --params-file $NAV_CONFIG &
BT_PID=$!
sleep 3
lc_node /bt_navigator

# ── 12. HP60C Depth camera (optional) ─────────────────────────────────────
CAM_PID=""
if [ "$NO_CAM" = false ]; then
  echo "[argo] Starting HP60C camera..."
  (
    cd $CAMERA_SDK_PATH
    source install/setup.bash
    ros2 launch ascamera hp60c.launch.py
  ) &
  CAM_PID=$!
  sleep 5

else
  echo "[argo] Camera skipped (--no-cam)"
fi

# ── 13. Depth safety shield  /cmd_vel_smoothed → /cmd_vel ─────────────────
echo "[argo] Starting depth_safety_shield..."
ros2 run argo_mini depth_safety_shield --ros-args \
  -p stop_distance:=0.35 \
  -p slow_distance:=0.65 \
  -p slow_factor:=0.40 \
  -p lateral_margin:=0.28 \
  -p min_obstacle_height:=0.05 \
  -p max_obstacle_height:=1.60 \
  -p depth_timeout:=3.0 \
  -p downsample_stride:=4 \
  -p input_topic:=/cmd_vel_smoothed \
  -p output_topic:=/cmd_vel \
  -p depth_topic:=/ascamera_hp60c/camera_publisher/depth0/points &
SHIELD_PID=$!
sleep 2

# ── 14. RViz ──────────────────────────────────────────────────────────────
echo "[argo] Starting RViz..."
export DISPLAY=:1
rviz2 &
RVIZ_PID=$!

echo ""
echo "========================================="
echo "  ARGO MINI — NAV2 + DEPTH SAFETY"
echo "========================================="
echo "  pipeline: controller→/cmd_vel_nav"
echo "           smoother→/cmd_vel_smoothed"
echo "           shield→/cmd_vel (STOP<0.35m)"
echo "  camera: $([ "$NO_CAM" = false ] && echo 'enabled' || echo 'disabled (--no-cam)')"
echo "  Press Ctrl+C to stop all nodes"
echo "========================================="
echo ""

# ── One-time udev tip (run once, then sudo never needed again) ─────────────
echo "  TIP: avoid sudo on USB — run once:"
echo "  echo 'SUBSYSTEM==\"tty\",KERNEL==\"ttyUSB*\",MODE=\"0666\"' | sudo tee /etc/udev/rules.d/99-usb-serial.rules && sudo udevadm control --reload"
echo ""

trap '
  echo "[argo] Shutting down..."
  kill $RSP_PID $SERIAL_PID $LIDAR_PID $RELAY_PID $TF_PID \
       $MAP_PID $AMCL_PID $PLANNER_PID $CONTROLLER_PID \
       $SMOOTHER_PID $BT_PID $BEHAVIOR_PID $SHIELD_PID $RVIZ_PID \
       ${CAM_PID:-} $CAM_TF_PID 2>/dev/null || true
  sleep 2
  pkill -9 -f ros2 2>/dev/null || true
  exit 0
' INT TERM

wait $RVIZ_PID
