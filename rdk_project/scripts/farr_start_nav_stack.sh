#!/usr/bin/env bash
set -euo pipefail

WORKSPACE=${WORKSPACE:-/root/farr_robot_ws}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

SENSORS_LOG=${SENSORS_LOG:-/tmp/farr_sensors.log}
SLAM_LOG=${SLAM_LOG:-/tmp/farr_slam.log}
NAV_LOG=${NAV_LOG:-/tmp/farr_nav2.log}

FARR_ENV="source /opt/ros/humble/setup.bash; \
source ${WORKSPACE}/install/setup.bash 2>/dev/null || true; \
export AMENT_PREFIX_PATH=${WORKSPACE}/install/farr_mapping:${WORKSPACE}/install/farr_bringup:\$AMENT_PREFIX_PATH; \
export PYTHONPATH=${WORKSPACE}/build/farr_mapping:${WORKSPACE}/build/farr_bringup:${WORKSPACE}/install/farr_mapping/lib/python3.10/site-packages:${WORKSPACE}/install/farr_bringup/lib/python3.10/site-packages:\$PYTHONPATH; \
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; \
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"

run_ros() {
  bash -lc "${FARR_ENV}; $*"
}

fail_startup() {
  echo "ERROR: $*"
  echo "Check logs: ${SENSORS_LOG}, ${SLAM_LOG}, ${NAV_LOG}"
  echo "Stopping partially started stack for safety."
  stop_old_processes >/dev/null 2>&1 || true
  exit 1
}

stop_old_processes() {
  echo "[1/7] Stop old FARR navigation stack processes"
  pkill -f "ros2 launch farr_bringup farr_nav2_amcl.launch.py" 2>/dev/null || true
  pkill -f "ros2 launch farr_bringup mapping_2_5d.launch.py" 2>/dev/null || true
  pkill -f "ros2 launch farr_bringup slam.launch.py" 2>/dev/null || true
  pkill -f "ros2 launch farr_bringup sensors.launch.py" 2>/dev/null || true
  pkill -f "cmd_vel_bridge" 2>/dev/null || true
  pkill -f "nav2_" 2>/dev/null || true
  pkill -f "amcl" 2>/dev/null || true
  pkill -f "map_server" 2>/dev/null || true
  pkill -f "pointcloud_to_laserscan" 2>/dev/null || true
  pkill -f "slice_mapper" 2>/dev/null || true
  pkill -f "fastlio_mapping" 2>/dev/null || true
  pkill -f "livox_ros_driver2_node" 2>/dev/null || true
  sleep 4
}

wait_for_topic_once() {
  local topic=$1
  local timeout_s=$2
  echo "      wait for ${topic} ..."
  run_ros "timeout ${timeout_s} ros2 topic echo ${topic} --once >/tmp/farr_topic_wait.out"
}

wait_for_hz() {
  local topic=$1
  local timeout_s=$2
  local window=$3
  local hz_log="/tmp/farr_hz_${topic//\//_}.log"
  echo "      check hz ${topic} ..."
  set +e
  run_ros "timeout ${timeout_s} ros2 topic hz ${topic} --window ${window}" >"${hz_log}" 2>&1
  local rc=$?
  set -e
  cat "${hz_log}"

  if grep -q "average rate:" "${hz_log}"; then
    return 0
  fi

  echo "ERROR: ${topic} has no measurable frequency within ${timeout_s}s, ros2 topic hz rc=${rc}"
  fail_startup "Topic ${topic} frequency check failed. Extra log: ${hz_log}"
}

check_slam_log_health() {
  local no_effective_count=0
  local not_synced_count=0
  local voxel_overflow_count=0

  no_effective_count=$(grep -c "No Effective Points" "${SLAM_LOG}" 2>/dev/null || true)
  not_synced_count=$(grep -c "IMU and LiDAR not Synced" "${SLAM_LOG}" 2>/dev/null || true)
  voxel_overflow_count=$(grep -c "VoxelGrid::applyFilter" "${SLAM_LOG}" 2>/dev/null || true)

  echo "      slam log health: NoEffective=${no_effective_count}, NotSynced=${not_synced_count}, VoxelOverflow=${voxel_overflow_count}"

  if (( voxel_overflow_count > 0 )); then
    fail_startup "FAST-LIO voxel overflow detected. Odometry has diverged."
  fi

  if (( not_synced_count > 2 )); then
    fail_startup "FAST-LIO IMU/LiDAR time sync is unstable."
  fi

  if (( no_effective_count > 50 )); then
    fail_startup "FAST-LIO has too many 'No Effective Points' messages."
  fi
}

check_fastlio_tf_sanity() {
  local tf_log=/tmp/farr_tf_camera_init_base_link.log
  echo "      check TF sanity camera_init -> base_link ..."

  set +e
  run_ros "timeout 6 ros2 run tf2_ros tf2_echo camera_init base_link" >"${tf_log}" 2>&1
  local rc=$?
  set -e

  python3 - "${tf_log}" <<'PY'
import math
import re
import sys

path = sys.argv[1]
text = open(path, "r", errors="ignore").read()
matches = re.findall(r"Translation:\s*\[([^\]]+)\]", text)
if not matches:
    print(f"ERROR: no camera_init -> base_link translation found in {path}")
    sys.exit(2)

values = [float(item.strip()) for item in matches[-1].split(",")]
norm = math.sqrt(sum(value * value for value in values))
print(f"      camera_init->base_link translation={values}, norm={norm:.3f} m")

if not math.isfinite(norm) or norm > 20.0:
    sys.exit(1)
PY
  local py_rc=$?

  if (( py_rc != 0 )); then
    echo "      tf2_echo rc=${rc}, log=${tf_log}"
    fail_startup "FAST-LIO TF is unreasonable; refusing to start Nav2."
  fi
}

start_sensors() {
  echo "[2/7] Start MID360S Livox driver"
  cd "${WORKSPACE}"
  nohup bash -lc "${FARR_ENV}; ros2 launch farr_bringup sensors.launch.py" \
    >"${SENSORS_LOG}" 2>&1 < /dev/null &
  sleep 5

  echo "[3/7] Confirm LiDAR and IMU publish before FAST-LIO starts"
  wait_for_topic_once /livox/imu 10
  wait_for_topic_once /livox/lidar 10
  wait_for_hz /livox/imu 8 5
  wait_for_hz /livox/lidar 10 5
}

start_slam() {
  echo "[4/7] Start FAST-LIO. Keep robot still during static IMU initialization"
  cd "${WORKSPACE}"
  nohup bash -lc "${FARR_ENV}; ros2 launch farr_bringup slam.launch.py" \
    >"${SLAM_LOG}" 2>&1 < /dev/null &

  echo "      waiting for slam launch static calibration and first registered cloud ..."
  sleep 14
  wait_for_topic_once /cloud_registered 20
  wait_for_hz /cloud_registered 12 3
  sleep 5
  check_slam_log_health
  check_fastlio_tf_sanity
}

start_nav() {
  echo "[5/7] Start Nav2 + AMCL + obstacle scan + STM32 cmd_vel bridge"
  cd "${WORKSPACE}"
  nohup bash -lc "${FARR_ENV}; ros2 launch farr_bringup farr_nav2_amcl.launch.py" \
    >"${NAV_LOG}" 2>&1 < /dev/null &

  echo "[6/7] Confirm Nav2 topics"
  sleep 18
  wait_for_topic_once /map 15
  wait_for_topic_once /scan 15
  run_ros "timeout 8 ros2 topic echo /amcl_pose --once >/tmp/farr_amcl_pose.out" || true
}

print_status() {
  echo "[7/7] FARR navigation stack status"
  ps -eo pid,args | grep -E "[s]ensors.launch|[s]lam.launch|[f]arr_nav2_amcl.launch|[l]ivox_ros_driver2_node|[f]astlio_mapping|[a]mcl|[m]ap_server|[c]md_vel_bridge|[p]ointcloud_to_laserscan|[s]lice_mapper" || true
  echo
  echo "Logs:"
  echo "  sensors: ${SENSORS_LOG}"
  echo "  slam:    ${SLAM_LOG}"
  echo "  nav2:    ${NAV_LOG}"
  echo
  echo "RViz:"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  echo "  export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
  echo "  rviz2 -d /home/pzh/farr_rviz/farr_nav_debug.rviz"
}

stop_old_processes
start_sensors
start_slam
start_nav
print_status
