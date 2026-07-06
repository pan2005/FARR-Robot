# FARR RViz 配置

本文件夹包含 FARR 四摇臂救援机器人常用 RViz2 配置。

## 文件说明

```text
farr_nav2_debug.rviz      # Nav2/AMCL 导航调试配置
farr_mapping_check.rviz   # 2.5D 建图/点云切片检查配置
```

## Ubuntu 端启动命令

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
rviz2 -d /path/to/farr_nav2_debug.rviz
```

如果文件放在 Ubuntu 的 `/home/pzh/farr_rviz/`：

```bash
rviz2 -d /home/pzh/farr_rviz/farr_nav2_debug.rviz
```

## Nav2 调试配置包含

- `/map` 静态地图
- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/scan`
- `/farr_nav_obstacle_cloud`
- `/cloud_registered`
- `/amcl_pose`
- `/particle_cloud`
- `/farr_base_odom`
- `/plan`
- `/local_plan`
- `TF`
- `2D Pose Estimate`
- `2D Goal Pose`

## 使用顺序

1. RDK 端启动雷达、FAST-LIO、Nav2。
2. Ubuntu 端打开 `farr_nav2_debug.rviz`。
3. 如果还没有设置 AMCL 初始位姿，`map` 固定坐标系下可能看不到点云，这是正常的。
4. 使用 `2D Pose Estimate` 设置机器人初始位姿，箭头指向车真实前方。
5. 等 `/particle_cloud` 收敛，确认 `/scan` 和地图边缘贴合。
6. 再使用 `2D Goal Pose` 点近距离目标。

## 只看点云的方法

如果只是想确认点云是否正常，不依赖 AMCL，可以在 RViz 左侧：

```text
Global Options -> Fixed Frame
```

临时改成：

```text
odom
```

正式导航时再改回：

```text
map
```

