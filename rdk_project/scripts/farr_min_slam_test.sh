#!/usr/bin/env bash
set -euo pipefail

WORKSPACE=${WORKSPACE:-/root/farr_robot_ws}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

SENSORS_LOG=${SENSORS_LOG:-/tmp/farr_min_sensors.log}
SLAM_LOG=${SLAM_LOG:-/tmp/farr_min_slam.log}
MONITOR_LOG=${MONITOR_LOG:-/tmp/farr_min_monitor.log}

source_ros() {
  set +u
  source /opt/ros/humble/setup.bash
  source "${WORKSPACE}/install/setup.bash"
  set -u
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}"
}

stop_all() {
  patterns=(
    "ros2 launch farr_bringup"
    "livox_ros_driver2_node"
    "fastlio_mapping"
    "slice_mapper"
    "pointcloud_to_laserscan"
    "cmd_vel_bridge"
    "nav2_"
    "amcl"
    "map_server"
    "rviz2"
  )

  for pattern in "${patterns[@]}"; do
    pkill -9 -f "${pattern}" 2>/dev/null || true
  done
  sleep 3
}

wait_for_topic() {
  local topic=$1
  local timeout_s=$2
  echo "wait for ${topic} ..."
  bash -lc "source /opt/ros/humble/setup.bash; source ${WORKSPACE}/install/setup.bash; export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}; timeout ${timeout_s} ros2 topic echo ${topic} --once >/tmp/farr_min_topic_wait.log"
}

print_hz() {
  local topic=$1
  local timeout_s=$2
  local window=$3
  echo "--- hz ${topic} ---"
  set +e
  bash -lc "source /opt/ros/humble/setup.bash; source ${WORKSPACE}/install/setup.bash; export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}; timeout ${timeout_s} ros2 topic hz ${topic} --window ${window}"
  set -e
}

monitor_fastlio() {
  source_ros
  : >"${MONITOR_LOG}"
  echo "--- monitor start $(date) ---" | tee -a "${MONITOR_LOG}"

  for i in $(seq 1 60); do
    no_effective=$(grep -c "No Effective Points" "${SLAM_LOG}" 2>/dev/null || true)
    not_synced=$(grep -c "IMU and LiDAR not Synced" "${SLAM_LOG}" 2>/dev/null || true)
    voxel_overflow=$(grep -c "VoxelGrid::applyFilter" "${SLAM_LOG}" 2>/dev/null || true)

    tf_line=$(timeout 2 ros2 run tf2_ros tf2_echo camera_init base_link 2>/dev/null | grep -m1 "Translation:" || true)
    odom_line=$(timeout 2 ros2 topic echo /Odometry --once --no-arr 2>/dev/null | grep -E "position:|x:|y:|z:" -m4 | tr '\n' ' ' || true)

    printf "[%03d] NoEffective=%s NotSynced=%s VoxelOverflow=%s TF='%s' Odom='%s'\n" \
      "${i}" "${no_effective}" "${not_synced}" "${voxel_overflow}" "${tf_line}" "${odom_line}" | tee -a "${MONITOR_LOG}"

    if (( voxel_overflow > 0 || not_synced > 2 || no_effective > 50 )); then
      echo "FAST-LIO unhealthy, stop monitor." | tee -a "${MONITOR_LOG}"
      return 1
    fi

    sleep 2
  done
}

echo "[1/5] Stop all FARR ROS processes"
stop_all

echo "[2/5] Start only MID360S Livox driver"
cd "${WORKSPACE}"
nohup bash -lc "source /opt/ros/humble/setup.bash; source ${WORKSPACE}/install/setup.bash; export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}; ros2 launch farr_bringup sensors.launch.py" \
  >"${SENSORS_LOG}" 2>&1 < /dev/null &
sleep 8

wait_for_topic /livox/imu 10
wait_for_topic /livox/lidar 10
print_hz /livox/imu 6 5

echo "[3/5] Start only FAST-LIO. Keep robot completely still."
nohup bash -lc "source /opt/ros/humble/setup.bash; source ${WORKSPACE}/install/setup.bash; export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}; ros2 launch farr_bringup slam.launch.py" \
  >"${SLAM_LOG}" 2>&1 < /dev/null &

echo "[4/5] Wait for static IMU initialization and first odometry"
sleep 16
wait_for_topic /Odometry 15

echo "[5/5] Monitor FAST-LIO for 120 seconds"
set +e
monitor_fastlio
monitor_rc=$?
set -e

echo
echo "Logs:"
echo "  sensors: ${SENSORS_LOG}"
echo "  slam:    ${SLAM_LOG}"
echo "  monitor: ${MONITOR_LOG}"
echo
echo "Process status:"
ps -eo pid,args | grep -E "[s]ensors.launch|[l]ivox_ros_driver2_node|[s]lam.launch|[f]astlio_mapping" || true

exit "${monitor_rc}"
