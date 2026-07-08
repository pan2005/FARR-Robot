# FARR - Four-Arm Rescue Robot

FARR means **Four-Arm Rescue Robot**. This repository contains the competition release code for an RDK X5 based four-arm rescue robot designed for collapsed buildings, underground spaces and other unstructured rescue environments.

The system combines a rocker-arm mobile chassis, STM32 real-time motor control, Livox MID360S LiDAR mapping, RDK X5 edge AI inference, ROS2 navigation and a rescue command frontend. The current codebase has already implemented LiDAR mapping, 2.5D occupancy map generation, AMCL/Nav2 navigation, STM32 serial chassis control, arm control interfaces and YOLO/BPU vision integration scaffolding.

## Highlights

- Four-arm rescue chassis inspired by planetary rovers, designed for rubble crossing and terrain adaptation.
- STM32 low-level controller for DJI 3508 chassis motors and four DM4340 arm motors.
- RDK X5 ROS2 autonomy stack with Livox MID360S, FAST-LIO, 2.5D mapping, AMCL and Nav2.
- 2.5D obstacle extraction with self-body filtering, ground-plane fitting and noise-resistant occupancy updates.
- Global elevation map demo for terrain visualization and future autonomous obstacle-crossing learning.
- Frontend-oriented ROS topic outputs for rescue command visualization.
- Planned imitation-learning obstacle-crossing module using LiDAR terrain features and expert demonstrations.

## System Architecture

```text
Livox MID360S + IMU
        |
        v
Livox ROS Driver -> FAST-LIO
        |              |
        |              +--> /cloud_registered
        |              +--> /Odometry
        v
2.5D terrain filtering
        |
        +--> /farr_2_5d_map       static occupancy map
        +--> /scan                AMCL/Nav2 laser input
        +--> /farr/global_elevation_cloud

RGB Camera -> YOLO/BPU inference -> rescue target topics

Nav2 / keyboard / future AI policy
        |
        v
/cmd_vel -> farr_chassis_bridge -> UART -> STM32
        |
        v
DJI 3508 chassis motors + DM4340 arm motors
```

## Repository Layout

```text
stm32/RM_Engineer_26_V1.0/        STM32 firmware for CAN motors, chassis, arms and serial control
rdk_project/src/farr_bringup/     ROS2 launch files, Nav2 params and system startup scripts
rdk_project/src/farr_mapping/     2.5D mapper, obstacle filtering and global elevation mapper
rdk_project/src/farr_chassis_bridge/
                                  RDK-to-STM32 serial bridge and keyboard/cmd_vel control
rdk_project/src/farr_vision_bpu/  Vision/BPU integration package scaffold
rdk_project/src/FAST_LIO_ROS2/    FAST-LIO LiDAR-inertial odometry
rdk_project/src/livox_ros_driver2/
                                  Livox MID360S ROS2 driver
rviz_configs/                     RViz debug and visualization configs
docs/                             Runbooks, project summaries and workflow notes
```

## Core Hardware

- Horizon Robotics RDK X5 edge AI board
- STM32 C board for low-level real-time control
- Livox MID360S LiDAR
- RGB camera for victim and hazard recognition
- DJI 3508 chassis motors
- Four DM4340 arm motors
- Four-arm rocker rescue chassis

## Current ROS2 Data Flow

| Function | Topic / Interface |
| --- | --- |
| Raw LiDAR | `/livox/lidar` |
| LiDAR IMU | `/livox/imu` |
| FAST-LIO registered cloud | `/cloud_registered` |
| FAST-LIO odometry | `/Odometry` |
| 2.5D occupancy map | `/farr_2_5d_map` |
| Nav2 scan input | `/scan` |
| Global elevation cloud | `/farr/global_elevation_cloud` |
| Global elevation grid | `/farr/global_elevation_grid` |
| Navigation velocity | `/cmd_vel` |
| STM32 serial bridge | `farr_chassis_bridge` |

## Quick Start On RDK X5

```bash
cd /root/farr_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

Start LiDAR, SLAM and 2.5D mapping:

```bash
ros2 launch farr_bringup sensors.launch.py
ros2 launch farr_bringup slam.launch.py
ros2 launch farr_bringup mapping_2_5d.launch.py
```

Save the 2.5D map:

```bash
ros2 service call /save_2_5d_map std_srvs/srv/Trigger '{}'
```

Start navigation after a map has been saved:

```bash
ros2 launch farr_bringup farr_nav2_amcl.launch.py
```

Start the global elevation map demo:

```bash
ros2 launch farr_bringup global_elevation_map.launch.py
ros2 service call /save_global_elevation_map std_srvs/srv/Trigger '{}'
```

## Development Status

Implemented:

- STM32 chassis and arm motor control
- RDK-to-STM32 UART control protocol
- Keyboard and `/cmd_vel` chassis control
- MID360S driver setup
- FAST-LIO odometry and registered point cloud
- 2.5D obstacle mapping and map saving
- AMCL/Nav2 navigation demo
- RViz mapping/navigation/elevation debug configs
- Global elevation map visualization and saving
- Frontend communication protocol preparation

In progress:

- Stable rescue-command frontend integration
- YOLO/BPU target topic publication
- More robust terrain-aware local planning

Planned for the national competition:

- Demonstration recorder for expert rocker-arm operation
- Local heightmap terrain feature extraction
- Behavior Cloning policy for autonomous obstacle crossing
- DAgger dataset expansion from failure cases
- Optional lightweight offline RL fine-tuning

## Imitation Learning Roadmap

The next-stage obstacle-crossing module will use a layered architecture:

```text
Mid360S point cloud + IMU + arm state
        |
        v
local 2m terrain heightmap / slope / obstacle features
        |
        v
Behavior Cloning policy on RDK X5
        |
        v
target arm angles
        |
        v
STM32 PID motor control
```

This design keeps motor safety and PID control on STM32 while allowing RDK X5 to learn high-level arm posture decisions from human demonstrations.

## Documentation

- [Project context summary](docs/FARR_PROJECT_CONTEXT_SUMMARY.md)
- [RDK operation runbook](docs/FARR_RDK_OPERATION_RUNBOOK.md)
- [Full startup and future development notes](docs/FARR_全链路启动与后续开发说明.md)
- [RDK X5 Codex development workflow](docs/RDK_X5_Codex_开发工作流.md)

## License

The FARR team code in this repository is released under the MIT License. Third-party components such as FAST-LIO, Livox ROS Driver and vendor firmware libraries retain their original licenses.
