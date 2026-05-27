#!/bin/bash
# Argo Mini — SLAM mapping script
#
# Runs slam_toolbox in async mapping mode.
# Reverse odometry is handled by serial_bridge (signed hall-sensor ticks),
# so backward driving contributes correctly to the map.
#
# Scan pipeline:
#   rplidar_composition  →  /scan
#   scan_relay           →  /scan_corrected  (timestamp fix)
#   slam_toolbox         ←  /scan_corrected
#
# TF tree:
#   odom → base_link  (serial_bridge, from signed hall-sensor odometry)
#   base_link → laser (static_transform_publisher, x=0.08 z=0.15 yaw=π)
#
# Usage:
#   ./start_slam.sh           # full SLAM + RViz
#   ./start_slam.sh --no-rviz # headless (SSH sessions)
#
# Save map when done:
#   ros2 run nav2_map_server map_saver_cli \
#     -f ~/argo_mini_ws/src/argo_mini/maps/indoor_map \
#     --ros-args -p save_map_timeout:=10.0

NO_RVIZ=false
for arg in "$@"; do
  [[ "$arg" == "--no-rviz" ]] && NO_RVIZ=true
done

# ── environment ────────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source ~/argo_mini_ws/install/setup.bash

SLAM_CONFIG=~/argo_mini_ws/install/argo_mini/share/argo_mini/config/slam_toolbox.yaml

# ── USB permissions ────────────────────────────────────────────────────────
chmod 666 /dev/ttyUSB0 /dev/ttyUSB1 2>/dev/null || \
  sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1 2>/dev/null || true

# ── kill previous run ──────────────────────────────────────────────────────
echo "[slam] Killing previous processes..."
for proc in slam_toolbox serial_bridge rplidar_composition rviz2 \
            robot_state_publisher scan_relay; do
  pkill -9 -f "$proc" 2>/dev/null || true
done
sleep 3

# ── 1. Static TF: base_link → laser ───────────────────────────────────────
# Matches the URDF laser_joint: x=0.08 z=0.15 yaw=π (connector faces rear).
# Using static_transform_publisher directly avoids URDF shell-escaping issues
# that break robot_state_publisher when launched from a bash script.
echo "[slam] Starting static TF base_link→laser..."
ros2 run tf2_ros static_transform_publisher \
  --x 0.08 --y 0.0 --z 0.15 \
  --roll 0.0 --pitch 0.0 --yaw 3.14159 \
  --frame-id base_link --child-frame-id laser &
RSP_PID=$!
sleep 1

# ── 2. Serial bridge (ESP32: motors + signed wheel odometry) ──────────────
# serial_bridge tracks signed hall-sensor ticks: ticks decrement during
# reverse so odom→base_link correctly moves the pose backward.
echo "[slam] Starting serial_bridge..."
ros2 run argo_mini serial_bridge --ros-args \
  -p port:=/dev/ttyUSB1 -p baud:=115200 \
  -p forward_only:=true \
  -p left_tick_scale:=2.0 &
SERIAL_PID=$!
sleep 3

# ── 3. RPLidar A1 ─────────────────────────────────────────────────────────
# Boost mode: higher scan rate (≈10 Hz, more points) → denser map.
# Fall back to Standard if your A1 firmware doesn't support Boost.
echo "[slam] Starting rplidar (Boost mode)..."
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser \
  -p angle_compensate:=true \
  -p scan_mode:=Boost &
LIDAR_PID=$!
sleep 3

# ── 4. Scan relay (timestamp correction) ──────────────────────────────────
# rplidar_ros stamps scans at the start of each rotation; slam_toolbox is
# sensitive to stale timestamps. scan_relay re-stamps with wall-clock time.
echo "[slam] Starting scan_relay..."
ros2 run argo_mini scan_relay &
RELAY_PID=$!
sleep 2

# ── 5. SLAM Toolbox (async mapping) ───────────────────────────────────────
# Override scan_topic to use the timestamp-corrected scan.
echo "[slam] Starting slam_toolbox..."
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file "$SLAM_CONFIG" \
  -p scan_topic:=/scan_corrected &
SLAM_PID=$!
sleep 3

# ── 6. RViz (optional) ────────────────────────────────────────────────────
RVIZ_PID=""
if [ "$NO_RVIZ" = false ]; then
  echo "[slam] Starting RViz..."
  export DISPLAY=:1
  rviz2 &
  RVIZ_PID=$!
fi

echo ""
echo "========================================="
echo "  ARGO MINI — SLAM MAPPING"
echo "========================================="
echo "  Scan:   /scan → relay → /scan_corrected"
echo "  Odom:   signed ticks (fwd+, rev−)"
echo "  TF:     odom→base_link→laser (URDF)"
echo ""
echo "  Teleop: python3 ~/argo_mini_ws/slam_teleop.py"
echo "    w=fwd  a=pivot-left  d=pivot-right  s=stop"
echo ""
echo "  Save map when done:"
echo "  ros2 run nav2_map_server map_saver_cli \\"
echo "    -f ~/argo_mini_ws/src/argo_mini/maps/indoor_map \\"
echo "    --ros-args -p save_map_timeout:=10.0"
echo ""
echo "  Press Ctrl+C to stop"
echo "========================================="
echo ""

trap '
  echo "[slam] Shutting down..."
  kill $RSP_PID $SERIAL_PID $LIDAR_PID $RELAY_PID $SLAM_PID \
       ${RVIZ_PID:-} 2>/dev/null || true
  sleep 2
  pkill -9 -f "slam_toolbox\|serial_bridge\|rplidar\|scan_relay\|static_transform_publisher" 2>/dev/null || true
  exit 0
' INT TERM

if [ "$NO_RVIZ" = false ] && [ -n "$RVIZ_PID" ]; then
  wait $RVIZ_PID
else
  wait $SLAM_PID
fi
