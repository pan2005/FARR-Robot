# FARR 四摇臂救援机器人

FARR 是 **Four-Arm Rescue Robot** 的缩写，意为“四摇臂救援机器人”。本仓库是面向 RDK 赛道的参赛作品发布仓库，包含 STM32 底层控制代码、RDK X5 ROS2 建图导航代码、RViz 调试配置和队内协作文档。

项目面向地震废墟、地下空间、坍塌建筑等复杂非结构化救援场景，目标是构建一套“能进入、能建图、能识别、能越障、能回传态势”的智能搜救机器人系统。

## 项目亮点

- 四摇臂式救援底盘，具备碎石、台阶和复杂地形通过能力。
- STM32 负责实时电机控制，驱动 DJI 3508 底盘电机和四个 DM4340 摇臂电机。
- RDK X5 负责 ROS2 上层计算，包括 MID360S 雷达、FAST-LIO、2.5D 建图、AMCL/Nav2 导航和视觉 AI。
- 已实现 2.5D 障碍建图：包含自体点云过滤、地面拟合、障碍高度切片和抗瞬时噪声占据更新。
- 已实现全局高程图演示功能，为后续自主越障和模仿学习提供地形表达。
- 预留救援指挥前端通信接口，可展示地图、机器人位置、目标识别结果和现场态势。
- 后续将使用模仿学习，让机器人根据前方地形自主调整四个摇臂姿态完成越障。

## 总体架构

```text
Livox MID360S 雷达 + 内置 IMU
        |
        v
Livox ROS Driver -> FAST-LIO
        |              |
        |              +--> /cloud_registered  注册点云
        |              +--> /Odometry           里程计
        v
2.5D 地形过滤与建图
        |
        +--> /farr_2_5d_map       静态占据地图
        +--> /scan                AMCL/Nav2 输入
        +--> /farr/global_elevation_cloud  高程图显示

RGB 相机 -> YOLO/BPU 推理 -> 伤员/危险目标话题

Nav2 / 键盘控制 / 未来 AI 越障策略
        |
        v
/cmd_vel -> farr_chassis_bridge -> 串口 -> STM32
        |
        v
3508 底盘电机 + DM4340 四摇臂电机
```

## 仓库结构

```text
stm32/RM_Engineer_26_V1.0/        STM32 固件：CAN 电机、底盘、摇臂、串口控制
rdk_project/src/farr_bringup/     ROS2 启动文件、Nav2 参数、一键启动脚本
rdk_project/src/farr_mapping/     2.5D 建图、障碍过滤、全局高程图节点
rdk_project/src/farr_chassis_bridge/
                                  RDK 到 STM32 的串口桥接、键盘控制、cmd_vel 控制
rdk_project/src/farr_vision_bpu/  视觉和 BPU 推理相关包
rdk_project/src/FAST_LIO_ROS2/    FAST-LIO 激光惯性里程计
rdk_project/src/livox_ros_driver2/
                                  Livox MID360S ROS2 驱动
rviz_configs/                     RViz 建图、导航、高程图调试配置
docs/                             队内说明、启动手册、项目总结
```

## 硬件组成

- 地平线 RDK X5 边缘 AI 计算板
- STM32 C 板底层控制器
- Livox MID360S 激光雷达
- RGB 摄像头
- DJI 3508 底盘电机
- 四个 DM4340 摇臂关节电机
- 四摇臂救援机器人机械底盘

## 主要 ROS2 话题

| 功能 | 话题 / 接口 |
| --- | --- |
| 原始雷达数据 | `/livox/lidar` |
| 雷达 IMU | `/livox/imu` |
| FAST-LIO 注册点云 | `/cloud_registered` |
| FAST-LIO 里程计 | `/Odometry` |
| 2.5D 占据地图 | `/farr_2_5d_map` |
| 导航用 LaserScan | `/scan` |
| 全局高程点云 | `/farr/global_elevation_cloud` |
| 全局高程栅格 | `/farr/global_elevation_grid` |
| 导航速度指令 | `/cmd_vel` |
| STM32 串口桥接 | `farr_chassis_bridge` |

## RDK X5 快速启动

```bash
cd /root/farr_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

启动雷达、SLAM 和 2.5D 建图：

```bash
ros2 launch farr_bringup sensors.launch.py
ros2 launch farr_bringup slam.launch.py
ros2 launch farr_bringup mapping_2_5d.launch.py
```

保存 2.5D 地图：

```bash
ros2 service call /save_2_5d_map std_srvs/srv/Trigger '{}'
```

加载地图并启动导航：

```bash
ros2 launch farr_bringup farr_nav2_amcl.launch.py
```

启动全局高程图演示：

```bash
ros2 launch farr_bringup global_elevation_map.launch.py
ros2 service call /save_global_elevation_map std_srvs/srv/Trigger '{}'
```

## 当前完成度

已经完成：

- STM32 底盘和摇臂电机控制
- RDK 与 STM32 串口通信协议
- 键盘控制和 `/cmd_vel` 底盘控制
- MID360S 雷达驱动配置
- FAST-LIO 里程计和注册点云输出
- 2.5D 障碍建图与地图保存
- AMCL/Nav2 导航 Demo
- RViz 建图、导航、高程图调试配置
- 全局高程图显示与保存
- 前端通信协议准备

正在完善：

- 救援指挥前端联调
- YOLO/BPU 识别结果话题发布
- 更稳定的地形感知局部规划

国赛计划：

- 采集人工遥控摇臂越障数据
- 构建机器人前方局部高程图特征
- 使用 Behavior Cloning 训练自主越障策略
- 使用 DAgger 迭代失败案例
- 视时间进行轻量化 Offline RL 微调

## 自主越障技术路线

后续自主越障模块采用分层方案：

```text
MID360S 点云 + IMU + 摇臂状态
        |
        v
前方 2m 局部高程图 / 坡度 / 障碍高度特征
        |
        v
RDK X5 上运行模仿学习策略网络
        |
        v
输出四个摇臂目标角度
        |
        v
STM32 PID 闭环控制电机执行
```

这种设计不会让神经网络直接输出电机 PWM，而是让 AI 负责高层姿态决策，STM32 继续负责安全可靠的底层控制。

## 文档入口

- [项目上下文总结](docs/FARR_PROJECT_CONTEXT_SUMMARY.md)
- [RDK 操作手册](docs/FARR_RDK_OPERATION_RUNBOOK.md)
- [全链路启动与后续开发说明](docs/FARR_全链路启动与后续开发说明.md)
- [RDK X5 Codex 开发工作流](docs/RDK_X5_Codex_开发工作流.md)

## 开源协议

本仓库中 FARR 团队自研代码采用 MIT License。FAST-LIO、Livox ROS Driver、STM32 厂商库等第三方组件保留其原始开源协议。
