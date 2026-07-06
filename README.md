# FARR - Four-Arm Rescue Robot

FARR is a four-arm rescue robot project built around an STM32 low-level controller and an RDK X5 ROS2 autonomy stack.

## Repository Layout

```text
stm32/RM_Engineer_26_V1.0/   STM32 motor, CAN, chassis, arm and serial control firmware
rdk_project/src/             RDK X5 ROS2 packages for chassis bridge, mapping, bringup and vision
rviz_configs/                RViz debug configurations
docs/                        Project notes, runbooks and context summaries
```

## Core Hardware

- RDK X5 edge AI board
- STM32 C board
- Livox MID360S LiDAR
- DJI 3508 chassis motors
- Four DM4340 arm motors
- RGB camera with YOLO/BPU inference

## Current Main Stack

- STM32 handles motor PID, CAN motor control, four-arm commands and serial communication.
- RDK X5 handles MID360S, FAST-LIO, 2.5D mapping, Nav2/AMCL, YOLO inference and ROS2 communication.
- The rescue command frontend consumes ROS topics for maps, robot state and visual detection results.

## Notes

This repository is a clean release copy for team collaboration. It intentionally excludes local build outputs, ROS2 install/log directories, temporary files and reference projects.

See `docs/FARR_PROJECT_CONTEXT_SUMMARY.md` and `docs/FARR_全链路启动与后续开发说明.md` for the full project context and operating workflow.
