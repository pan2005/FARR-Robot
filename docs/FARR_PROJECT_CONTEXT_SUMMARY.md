# FARR 四摇臂救援机器人项目上下文总览

本文档用于在新的 Codex 对话中快速恢复项目上下文。项目代号 **FARR**，含义为 **Four-Arm Rescue Robot**。

## 1. 项目目标

本项目面向地震废墟、地下空间等复杂非结构化环境，开发一台基于 **RDK X5 + STM32 + MID360S + YOLO + 四摇臂底盘** 的智能搜救机器人。

总体目标：

- STM32 负责底层电机控制、底盘运动、四个摇臂 DM4340 控制。
- RDK X5 负责激光雷达、视觉 AI、ROS2、建图导航、自主越障算法。
- 前端救援指挥终端负责显示地图、机器人位置、目标识别结果、语义信息。
- 省赛阶段优先完成：雷达建图、基础导航、YOLO 识别、前端显示、人工驾驶搜救 Demo。
- 国赛阶段目标：加入基于模仿学习的自主越障控制。

## 2. 硬件组成

- 主控计算：RDK X5
- 底层控制：STM32 C 板
- 激光雷达：Livox MID360S
- 视觉：RGB 相机，RDK 上运行 YOLO/BPU 推理
- 底盘：四轮/履带式底盘，轮电机为 DJI 3508
- 摇臂：四个 DM4340 达妙关节电机
- 通信：
  - RDK X5 与 STM32 通过串口通信
  - RDK X5 与 PC/Ubuntu/RViz 通过局域网 ROS2 通信

## 3. 代码与工作区

### STM32 工程

主工程：

```text
E:\RM_STM32_Project\RM_Engineer_26_V1.0
```

重要原则：

- 不修改 `E:\RM_STM32_Project\RM_Engineer_26_V1.0` 以外的 STM32 工程代码。
- DJI 3508 底盘电机参考 HERO chassis 工程。
- DM4340 达妙电机参考 HERO gimbal 工程。

已完成/曾处理内容：

- 移植了 CAN 和达妙电机相关驱动。
- 初始化 4 个 DM4340，CAN1，总线 ID 为 1、2、3、4。
- DM 电机 enable 指令需要加 `osDelay(1)`，否则 2、4 号电机可能初始化失败。
- 麦轮/底盘控制中曾发现左右方向反，需要注意 `vy` 或遥控器通道映射。
- RDK 串口控制 STM32 已经跑通过。
- `cmd_vel` 到 STM32 的 vx/vy/w 控制链路已经验证过。
- 后来发现旋转方向 `w` 反了，修正后 Nav2 转向成功。

### RDK ROS2 工作区

主工作区：

```text
/root/farr_robot_ws
```

本地备份/开发目录：

```text
E:\RM_STM32_Project\rdk_project
```

重要包：

```text
/root/farr_robot_ws/src/
  farr_chassis_bridge      # RDK 到 STM32 串口通信，键盘控制，cmd_vel 桥接
  farr_bringup             # 统一 launch
  farr_mapping             # 2.5D 建图、点云切片、地面拟合等
  farr_vision_bpu          # 相机与 YOLO/BPU 推理
  farr_web_gateway         # 曾为前端 websocket 测试建立，后续不一定主用
  livox_ros_driver2        # Livox 驱动，第三方包名不改
  FAST_LIO_ROS2            # FAST-LIO，第三方包名不改
```

开发建议：

- 本地修改代码，再同步到 RDK。
- 避免长时间依赖交互式 SSH 管道。
- 长时间任务建议在 RDK 上用 tmux。

## 4. 网络与 SSH

RDK 曾用 IP：

```text
192.168.43.115
192.168.1.84
192.168.140.233
```

当前最近一次局域网中 RDK IP：

```text
192.168.140.233
```

Ubuntu/RViz 机器：

```text
用户：pzh
地址：100.117.20.28 或局域网地址
```

ROS2 通信统一习惯：

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

## 5. MID360S 雷达网络配置

新雷达是 **MID360S**，不是普通 MID360。这个点非常关键。

雷达最终应改到独立雷达网段：

```text
RDK eth0:      192.168.10.50/24
MID360S:       192.168.10.136
host ip:       192.168.10.50
```

Ubuntu 上曾用 Livox-SDK2 成功修改过雷达 IP。关键现象：

- 普通 MID360 的工具/配置不完全适配 MID360S。
- MID360S `dev_type=35`。
- 成功日志里出现：

```text
found lidar handle=... ip=192.168.1.149 sn=... dev_type=35
set point host callback: status=0 ... ret_code=0
set imu host callback: status=0 ... ret_code=0
set lidar ip callback: status=0 ... ret_code=0
reboot callback: status=0 ... ret_code=0
```

雷达通断验证：

```bash
ping 192.168.10.136
```

## 6. 坐标系结论

这是项目中最关键、最容易出问题的部分。

最终原则：

```text
map -> camera_init -> base_link -> laser_link
```

说明：

- `map`：Nav2/AMCL 全局地图坐标系。
- `camera_init`：FAST-LIO 的里程计/初始坐标系。
- `base_link`：机器人车体中心坐标系。
- `laser_link`：雷达坐标系，来自 URDF。

URDF 中雷达安装关系：

```xml
base_link -> laser_link
xyz="0.03169 -0.05418 0.35690"
rpy="pi 0 -pi/2"
```

重要原则：

- `base_link` 和 `laser_link` 以 URDF 为准。
- 不要再通过盲目改 yaw、翻图、反号来修镜像。
- 建图和导航必须使用同一条点云处理链路。
- 雷达反装后，要确保 FAST-LIO 的 body/laser_link 关系明确，不能让 `body` 同时承担车体坐标含义。

已验证：

- 车体前进、左移、右移的 FAST-LIO/点云方向经过实车运动测试。
- `cmd_vel` 的前后左右测试通过。
- 后来发现 `w` 旋转方向反，修正后导航转向成功。

## 7. 建图链路

目前使用的是 2.5D 建图方案，不是传统纯 2D 激光。

主要流程：

```text
MID360S 点云
  -> livox_ros_driver2
  -> FAST-LIO
  -> /cloud_registered
  -> 点云切片/地面拟合/自体过滤
  -> /farr_obstacle_cloud
  -> /farr_2_5d_map
  -> 保存为 Nav2 可用地图
```

曾经调过的重要参数/策略：

- 雷达 blind 距离用于过滤机器人自身近距离结构。
- 需要过滤机器人四个摇臂，否则会把自己画成障碍。
- 单纯高度切片容易把地面回波扫进去。
- 更好的方法是地面拟合，然后根据离地高度判断障碍。
- 曾经因为螺丝松动导致点云倾斜，后来修正系数应清零。
- 保存地图前应确认 `/farr_obstacle_cloud` 没有大面积吃进地面。

建图启动常用流程：

```bash
cd /root/farr_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch farr_bringup sensors.launch.py
ros2 launch farr_bringup slam.launch.py
ros2 launch farr_bringup slice_cloud_cpp.launch.py
ros2 launch farr_bringup mapping_2_5d.launch.py
```

清空旧地图：

```bash
ros2 service call /reset_2_5d_map std_srvs/srv/Trigger '{}'
```

保存地图：

```bash
ros2 service call /save_2_5d_map std_srvs/srv/Trigger '{}'
```

## 8. Nav2 导航状态

Nav2 Demo 已经成功过：

- 地图能显示。
- AMCL 能定位。
- 点目标后车能移动。
- 修正旋转方向后，两次转向成功。
- 仍存在不完善点：车体尺寸、膨胀半径、局部代价地图、贴墙卡住等还需要微调。

导航启动大致流程：

```bash
cd /root/farr_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch farr_bringup sensors.launch.py
ros2 launch farr_bringup slam.launch.py
ros2 launch farr_bringup slice_cloud_cpp.launch.py
ros2 launch farr_bringup farr_nav2_amcl.launch.py
ros2 run farr_chassis_bridge cmd_vel_bridge
```

注意：

- 遥控器需要拨到允许上位机控制的档位，否则 `/cmd_vel` 有输出但车不动。
- STM32 串口桥可能偶发进入错误状态，重启 RDK 或底盘桥接节点后恢复。
- `ros2 topic echo /cmd_vel` 曾因 rclpy 状态异常报错，重启 ROS 进程可恢复。
- 如果 Nav2 进入错误状态，可以尝试 lifecycle reset/重新 launch，最稳妥是杀掉导航相关进程后干净重启。

## 9. RViz 配置

Ubuntu 端 RViz 配置目录：

```text
/home/pzh/farr_rviz
```

常用配置：

```text
farr_mapping_check.rviz
farr_nav_debug.rviz
farr_nav2_debug.rviz
```

RViz 启动：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
rviz2 -d /home/pzh/farr_rviz/farr_nav_debug.rviz
```

RViz 中需要显示：

- `/map`
- `/scan`
- `/farr_obstacle_cloud`
- `/cloud_registered`
- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/plan`
- `/local_plan`
- `/amcl_pose`
- `/particle_cloud`
- TF

如果 RViz 报：

```text
frame map does not exist
```

通常是 map_server/Nav2/AMCL 没有正常启动，或 `/map` 没有发布。

## 10. 前端救援指挥终端

省赛需要做一个“救援指挥终端”：

- 显示地图。
- 显示机器人位置。
- 显示 YOLO 发现的伤员/目标位置。
- 显示 RDK 回传的识别结果与状态。
- 可展示 URDF/机器人模型，提高演示效果。

队友已经完成前端部分。当前约定是前端通过 ROS 话题接收 RDK 数据，不需要我们再做前端页面。

视觉输出协议文件：

```text
F:\wechat_files\xwechat_files\wxid_u9yxmt2lj8wf12_2c6e\msg\file\2026-07\RDK_VISION_OUTPUT_PROTOCOL.md
```

协议要求 RDK 发布：

```text
/camera/image_raw              sensor_msgs/msg/Image
/person_pose/result_image      sensor_msgs/msg/Image
/person_pose/detections        std_msgs/msg/String，JSON
/person_pose/status            std_msgs/msg/String，JSON
```

当前提醒：

- 最近一次用户明确说“先不要管视觉”，所以视觉话题尚未继续实现。
- 后续要做时，应在 `farr_vision_bpu` 的 YOLO 节点中增加 `/person_pose/detections` 和 `/person_pose/status` 发布。

## 11. 视觉与 YOLO

已知：

- RDK 上 YOLO/BPU 推理已经跑通过。
- `/person_pose/result_image` 话题曾经存在。
- 前端只需要 ROS 话题数据。

待做：

- 将 YOLO 检测结果整理为 JSON。
- 发布到 `/person_pose/detections`。
- 发布运行状态到 `/person_pose/status`。
- 保持无检测时也发布空数组。

示例 detection JSON：

```json
{
  "timestamp": "2026-07-05T12:00:00+08:00",
  "frame_id": "hik_camera",
  "detections": [
    {
      "id": "Victim-01",
      "label": "person",
      "confidence": 0.91,
      "bbox": {"x1": 120, "y1": 80, "x2": 260, "y2": 360}
    }
  ]
}
```

示例 status JSON：

```json
{
  "timestamp": "2026-07-05T12:00:00+08:00",
  "camera": {"online": true},
  "inference": {"running": true}
}
```

## 12. 自主越障路线

长期目标是使用 **Behavior Cloning + DAgger + 可选 Offline RL** 实现自主越障。

建议路线：

```text
MID360 点云 / 2.5D 高程图
IMU 姿态
摇臂角度
底盘速度
  -> 地形特征提取
  -> 模仿学习策略网络
  -> 四摇臂目标角度
  -> STM32 PID/DM4340 控制
```

省赛阶段建议做小 Demo：

- 人工遥控机器人上一个台阶。
- 同时记录点云高程图、IMU、摇臂角度、人工目标。
- 展示“具备自主越障学习能力的雏形”。

国赛阶段：

- 扩大数据集。
- 做 DAgger：自主运行、失败人工纠正、加入数据集、重新训练。
- 模型部署到 RDK X5，最好能转 ONNX/BPU。

## 13. 当前最重要的已解决问题

1. RDK 与 STM32 串口通信跑通。
2. 键盘控制、cmd_vel 控制底盘跑通。
3. MID360S 新雷达网络配置跑通。
4. FAST-LIO + 点云处理 + 2.5D 建图跑通。
5. Nav2 + AMCL 基础导航 Demo 跑通。
6. 坐标系大方向已经理顺，尤其是 `base_link`、`laser_link`、`camera_init` 的关系。
7. 旋转方向问题已定位为 `w` 映射问题，并通过实车验证修正。

## 14. 当前主要风险与坑

- 雷达安装机械松动会导致点云倾斜，软件修正不能替代机械固定。
- 雷达自身回波、地面回波、机器人摇臂自体点云会污染地图。
- 建图和导航必须走同一条点云处理链路，否则会出现“建图正常，导航镜像/错位”。
- Nav2 参数还没有完全适配四摇臂机器人，车体尺寸和膨胀半径需要继续调。
- 底盘速度太小会因为死区动不了，不能盲目用保守速度。
- RDK 上 Python/colcon 环境曾有 setuptools 版本坑，必要时固定为 58.2.0。
- 长时间 SSH 管道容易出问题，推荐本地开发、同步部署、tmux 运行。

## 15. 新对话建议优先任务

如果继续省赛方向，建议顺序：

1. 确认 RDK 当前 IP 和 SSH。
2. 启动建图链路，检查雷达和 `/farr_2_5d_map`。
3. 启动 Nav2，复现基础导航 Demo。
4. 调整 Nav2 车体尺寸、膨胀半径、局部代价地图。
5. 按前端协议补齐视觉 ROS 话题：
   - `/person_pose/detections`
   - `/person_pose/status`
6. 做省赛演示链路：
   - 前端地图显示
   - YOLO 识别伤员
   - 地图上标出目标
   - 人工驾驶救援
7. 开始录制越障数据，为模仿学习 Demo 做准备。

## 16. 给新对话的开场提示

可以直接把下面这段发给新对话：

```text
你现在接手 FARR 四摇臂救援机器人项目。请先阅读 FARR_PROJECT_CONTEXT_SUMMARY.md。当前项目重点是 RDK X5 上的 ROS2 链路，包括 MID360S、FAST-LIO、2.5D 建图、Nav2、STM32 串口底盘桥接、YOLO 视觉输出和前端 ROS 话题协议。注意：base_link 和 laser_link 以 URDF 为准，建图和导航必须使用同一条点云处理链路，不要靠盲目翻图/改 yaw 修坐标。最近用户明确说先不要管视觉，除非我重新要求。
```
