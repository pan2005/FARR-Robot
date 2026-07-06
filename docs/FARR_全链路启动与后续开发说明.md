# FARR 四摇臂救援机器人全链路启动与开发说明

本文档记录当前已经跑通的导航 Demo 启动流程、核心代码位置、已确认的问题修复，以及后续从 2D/2.5D 导航转向越障数据录制和模仿学习的开发方向。

## 1. 当前系统状态

当前已经打通的链路：

```text
MID360S
  -> livox_ros_driver2
  -> FAST-LIO
  -> /cloud_registered + /Odometry
  -> 2.5D 障碍切片 /farr_nav_obstacle_cloud
  -> /scan
  -> AMCL + Nav2
  -> /cmd_vel
  -> farr_chassis_bridge
  -> STM32
  -> 底盘运动
```

已经验证：

- 雷达 MID360S 网络已固定到 `192.168.10.136`。
- RDK X5 有线网口使用 `192.168.10.50`。
- `base_link +X` 是车体真实前方。
- `base_link +Y` 是车体真实左方。
- `/cmd_vel.linear.x > 0` 对应车体前进。
- `/cmd_vel.angular.z > 0` 最终已修正为车体逆时针左转。
- Nav2 可以完成至少两次连续转向导航，导航 Demo 算跑通，但仍需要调 footprint、膨胀层和局部规划参数。

## 2. RDK X5 启动准备

SSH 到 RDK：

```bash
ssh root@192.168.1.84
```

进入工作区并设置环境：

```bash
cd /root/farr_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export AMENT_PREFIX_PATH=/root/farr_robot_ws/install/farr_mapping:/root/farr_robot_ws/install/farr_bringup:$AMENT_PREFIX_PATH
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

检查雷达网络：

```bash
ip -br addr show eth0
ping 192.168.10.136
```

期望：

```text
eth0 有 192.168.10.50
192.168.10.136 可以 ping 通
```

## 3. 完整冷启动流程

### 3.1 清理旧进程

当前已有脚本：

```bash
python3 /tmp/farr_clean_all.py
```

该脚本会清理：

- livox driver
- FAST-LIO
- slice mapper
- pointcloud_to_laserscan
- AMCL / Nav2
- cmd_vel_bridge
- robot_state_publisher
- odom_tf_broadcaster

如果只是 Nav2 卡死，后续建议单独写 `restart_nav2_only.sh`，避免重启雷达和 FAST-LIO。

### 3.2 启动雷达和 FAST-LIO

```bash
bash /tmp/start_sensors_slam.sh
```

该脚本会启动：

```bash
ros2 launch farr_bringup sensors.launch.py
ros2 launch farr_bringup slam.launch.py
```

检查话题：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /cloud_registered
ros2 topic echo /Odometry --once
```

正常频率：

```text
/livox/lidar       约 10 Hz
/cloud_registered 约 10 Hz
/Odometry          正常输出，frame_id=camera_init, child_frame_id=body
```

### 3.3 启动 Nav2

```bash
bash /tmp/start_nav2.sh
```

该脚本会启动：

- `robot_state_publisher`
- `odom_tf_broadcaster`
- `farr_nav_obstacle_filter`
- `pointcloud_to_laserscan`
- `map_server`
- `amcl`
- Nav2 controller/planner/BT navigator
- `cmd_vel_bridge`

等待日志出现：

```text
Managed nodes are active
```

注意：在没有设置 AMCL 初始位姿前，日志中出现下面内容是正常的：

```text
AMCL cannot publish a pose or update the transform. Please set the initial pose...
Timed out waiting for transform from base_link to map...
```

因为此时还没有 `map -> odom`。

## 4. Ubuntu RViz2 启动

Ubuntu 电脑终端执行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
rviz2 -d /home/pzh/farr_rviz/farr_nav2_debug.rviz
```

如果 RViz2 显示异常，可以清缓存：

```bash
pkill -f rviz2
rm -rf ~/.rviz2 ~/.cache/rviz2 ~/.cache/OGRE ~/.ros/rviz2
```

如果只是想看点云，不依赖 AMCL，可以把 RViz 左侧：

```text
Global Options -> Fixed Frame
```

临时改为：

```text
odom
```

正式导航时改回：

```text
map
```

## 5. 导航操作流程

启动完成后，RViz 中按顺序操作：

1. 确认遥控器拨到上挡，使 STM32 接受 RDK 控制。
2. 不要先点 Goal。
3. 使用 `2D Pose Estimate` 设置初始位姿。
4. 箭头方向必须指向机器人真实前方。
5. 等 `/particle_cloud` 收敛。
6. 确认 `/scan` 或 `/farr_nav_obstacle_cloud` 与地图边缘基本贴合。
7. 使用 `2D Goal Pose` 点一个近距离目标，先从 0.5m 到 1m 开始。

如果 Nav2 进入错误状态：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

然后重启 Nav2 半链路：

```bash
bash /tmp/start_nav2.sh
```

更推荐后续整理成：

```bash
/root/farr_robot_ws/restart_nav2_only.sh
```

## 6. 当前关键坐标系

当前坐标系原则：

```text
map -> odom -> base_link -> laser_link
odom -> camera_init -> body
```

含义：

- `base_link`：机器人车体中心，+X 前，+Y 左，+Z 上。
- `laser_link`：雷达坐标系，来自 URDF 静态安装关系。
- `body`：FAST-LIO 内部雷达 body frame，目前按原则认为和 `laser_link` 一致。
- `camera_init`：FAST-LIO 初始化坐标系。
- `odom`：由 `odom_tf_broadcaster` 从 FAST-LIO 里程计转换得到。
- `map`：AMCL 根据地图和 `/scan` 发布。

URDF 中雷达安装关系：

```text
base_link -> laser_link
xyz = 0.03169 -0.05418 0.35690
rpy = pi 0 -pi/2
```

## 7. 核心代码位置

RDK 工作区：

```text
/root/farr_robot_ws
```

核心包：

```text
/root/farr_robot_ws/src/farr_bringup
/root/farr_robot_ws/src/farr_mapping
/root/farr_robot_ws/src/farr_chassis_bridge
/root/farr_robot_ws/src/livox_ros_driver2
/root/farr_robot_ws/src/FAST_LIO_ROS2
```

重要文件：

```text
/root/farr_robot_ws/src/farr_bringup/launch/sensors.launch.py
/root/farr_robot_ws/src/farr_bringup/launch/slam.launch.py
/root/farr_robot_ws/src/farr_bringup/launch/farr_nav2_amcl.launch.py
/root/farr_robot_ws/src/farr_bringup/config/farr_nav2_params.yaml
/root/farr_robot_ws/src/farr_bringup/config/farr_scan_from_mid360.yaml
/root/farr_robot_ws/src/farr_bringup/farr_bringup/odom_tf_broadcaster.py
/root/farr_robot_ws/src/farr_mapping/farr_mapping/slice_mapper.py
/root/farr_robot_ws/src/farr_chassis_bridge/farr_chassis_bridge/cmd_vel_bridge_node.py
/root/farr_robot_ws/src/farr_chassis_bridge/farr_chassis_bridge/keyboard_control_node.py
```

地图位置：

```text
/root/farr_maps/farr_2_5d_map.yaml
/root/farr_maps/farr_2_5d_map.pgm
```

RViz 配置：

```text
/home/pzh/farr_rviz/farr_nav2_debug.rviz
/home/pzh/farr_rviz/farr_mapping_check.rviz
```

## 8. 已修复的重要问题

### 8.1 角速度方向修复

文件：

```text
/root/farr_robot_ws/src/farr_chassis_bridge/farr_chassis_bridge/cmd_vel_bridge_node.py
```

当前逻辑：

```python
self.cmd.w = self.clamp(-msg.angular.z * self.angular_scale, self.max_w)
```

原因：

- ROS 约定：`angular.z > 0` 是逆时针左转。
- 实测原始桥接会让车右转。
- 因此桥接发送到 STM32 前需要对 `w` 取负号。

### 8.2 TF 时间戳修复

文件：

```text
/root/farr_robot_ws/src/farr_bringup/farr_bringup/odom_tf_broadcaster.py
```

当前逻辑：

```python
stamp = self.get_clock().now().to_msg()
```

原因：

- 原来使用 FAST-LIO `/Odometry` 的消息时间戳发布 TF。
- RViz 发送 `2D Pose Estimate` 时 AMCL 查询当前时间 TF，容易出现 `future extrapolation`。
- 改为当前时间发布 TF 后，AMCL 初始位姿更稳定。

## 9. 当前 Demo 还不完善的点

### 9.1 容易贴墙或卡墙

优先调整：

```text
Nav2 footprint / robot_radius
global_costmap inflation_radius
local_costmap inflation_radius
cost_scaling_factor
```

需要按机器人真实外廓设置，尤其四摇臂伸出后尺寸会变大。

### 9.2 地图仍然是 2D 占据表达

当前地图是 2.5D 点云经过阈值化后的 2D 占据栅格，适合 Nav2 Demo，但不能表达：

- 台阶高度
- 坡度
- 凹坑
- 可越障但不可普通通行的区域
- 摇臂需要提前调整的地形

所以后续越障不能只依赖当前 2D 地图。

### 9.3 AMCL 初始位姿仍需要人工给

Demo 阶段可以接受。

后续如果要自主探索，需要考虑：

- SLAM 模式下直接使用 FAST-LIO 里程计和局部高程图
- 不依赖预先保存的静态地图
- 或使用建图和定位分离的全局系统

## 10. 后续方向：从 2D 地图转向 2.5D 高程图

当前 Nav2 Demo 的地图适合“走过去”，但四摇臂救援机器人的核心能力应该是“看见障碍并调整摇臂越过去”。

推荐下一阶段改为局部 2.5D 高程图：

```text
/cloud_registered
  -> base_link/odom 下局部点云
  -> 地面拟合
  -> 局部高程图
  -> 地形特征
  -> 摇臂目标角
```

局部高程图建议范围：

```text
前方 2.0m
左右 1.0m
分辨率 0.05m 或 0.08m
```

输出可以是：

```python
heightmap: H x W
valid_mask: H x W
slope_map: H x W
obstacle_height
obstacle_distance
pitch
roll
current_arm_angle
```

## 11. 近期 Demo 建议：做数据录制

下一阶段 Demo 不建议一上来做完全自主越障。更稳的方案：

```text
人工遥控越障
  -> 记录雷达局部高程图
  -> 记录 IMU
  -> 记录底盘速度
  -> 记录四摇臂角度
  -> 记录人工给出的摇臂目标
```

保存为：

```text
dataset/
  episode_001.npz
  episode_002.npz
  episode_003.npz
```

单帧数据建议：

```python
{
    "timestamp": float,
    "heightmap": np.ndarray,
    "valid_mask": np.ndarray,
    "pitch": float,
    "roll": float,
    "vx": float,
    "w": float,
    "arm_angle": np.ndarray,          # 当前四摇臂角度
    "expert_arm_target": np.ndarray,  # 人工示教目标角
}
```

这样初审视频可以展示：

- 机器人能导航 Demo。
- 能看到局部 2.5D 地形。
- 能录制越障专家数据。
- 国赛前基于这些数据训练模仿学习模型。

## 12. 国赛方向：模仿学习

推荐路线：

```text
Behavior Cloning
  -> DAgger
  -> 可选 Offline RL
```

输入：

```python
state = [
    heightmap,
    valid_mask,
    pitch,
    roll,
    current_arm_angle,
    vx,
    w,
]
```

输出：

```python
target_arm_angle = [lf, rf, lr, rr]
```

模型建议：

- 不要端到端图像到 PWM。
- 不要在线 RL 起步。
- 使用轻量 CNN + MLP。
- 输出四个摇臂目标角，底层仍由 STM32 PID 执行。

部署方向：

- PyTorch 训练。
- 导出 ONNX。
- 转换到 RDK X5 BPU 可部署格式。
- 如果 BPU 部署短期卡住，先 CPU 跑轻量模型做 Demo，再继续优化 BPU。

## 13. 下一步优先级

建议按下面顺序推进：

1. 固化当前导航启动脚本，整理 `restart_nav2_only.sh`。
2. 调整 footprint 和 inflation，减少贴墙/卡墙。
3. 把当前 2.5D 切片节点扩展为局部高程图输出节点。
4. 写 `ExpertDataRecorder`，开始采集人工越障数据。
5. 写离线可视化脚本，检查每条 episode 的高程图和摇臂动作是否对齐。
6. 训练第一版 BC 模型。
7. 在 RDK 上跑在线推理，只输出摇臂目标角，底盘仍先人工或 Nav2 控制。
8. 国赛前做 DAgger：机器人自动跑，人工纠错，继续补数据。

## 14. 常用命令速查

RDK 环境：

```bash
cd /root/farr_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export AMENT_PREFIX_PATH=/root/farr_robot_ws/install/farr_mapping:/root/farr_robot_ws/install/farr_bringup:$AMENT_PREFIX_PATH
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

完整启动：

```bash
python3 /tmp/farr_clean_all.py
bash /tmp/start_sensors_slam.sh
bash /tmp/start_nav2.sh
```

Ubuntu RViz：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
rviz2 -d /home/pzh/farr_rviz/farr_nav2_debug.rviz
```

停止底盘：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

检查话题：

```bash
ros2 topic list
ros2 topic hz /cloud_registered
ros2 topic hz /scan
ros2 topic echo /cmd_vel
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link laser_link
```

重新编译：

```bash
cd /root/farr_robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

