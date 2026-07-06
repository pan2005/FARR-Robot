# FARR RDK X5 现场启动与操作手册

本文档用于现场从零清理环境、启动 MID360S 雷达、开启 FAST-LIO/2.5D 建图、保存地图、开启 Nav2 导航，以及开启前端 Web Gateway。

RDK 地址：

```text
192.168.140.233
```

RDK 工作区：

```bash
/root/farr_robot_ws
```

前端网关端口：

```text
8080
```

## 1. 登录 RDK

在电脑终端执行：

```bash
ssh root@192.168.140.233
```

每个新终端都先执行：

```bash
cd /root/farr_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
./fix_farr_env.sh
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

如果 `source` 过程中出现类似：

```text
source: filename argument required
```

但后续 `ros2 pkg list` 正常，一般是历史环境 hook 噪声，可以先忽略。

检查包是否可见：

```bash
ros2 pkg prefix farr_bringup
ros2 pkg prefix farr_mapping
ros2 pkg prefix farr_web_gateway
```

## 2. 清理旧环境

如果现场状态混乱、节点重复、topic 有旧数据，先清理。这个清理不会杀 SSH，只清 ROS/FARR 相关进程。

```bash
pkill -f "ros2 launch farr_bringup sensors.launch.py" || true
pkill -f "ros2 launch farr_bringup slam.launch.py" || true
pkill -f "ros2 launch farr_bringup mapping_2_5d.launch.py" || true
pkill -f "ros2 launch farr_bringup farr_nav2_amcl.launch.py" || true
pkill -f "ros2 launch farr_bringup web_gateway.launch.py" || true
pkill -f "ros2 run farr_bringup odom_tf_broadcaster" || true

pkill -f "livox_ros_driver2" || true
pkill -f "fastlio_mapping" || true
pkill -f "slice_mapper" || true
pkill -f "odom_tf_broadcaster" || true
pkill -f "pointcloud_to_laserscan" || true
pkill -f "farr_cmd_vel_bridge" || true
pkill -f "web_gateway" || true
```

重启 ROS2 daemon：

```bash
ros2 daemon stop || true
ros2 daemon start || true
```

确认没有残留：

```bash
ps -eo pid,comm,args | grep -E "livox|fastlio|slice_mapper|odom_tf|nav2|amcl|map_server|web_gateway|farr_bringup" | grep -v grep
```

如果没有输出，表示清理干净。

## 3. 启动雷达驱动

终端 1：

```bash
cd /root/farr_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch farr_bringup sensors.launch.py
```

正常日志应出现：

```text
successfully set lidar attitude, ip: 192.168.10.136
successfully enable Livox Lidar imu
livox/lidar publish use livox custom format
```

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source /root/farr_robot_ws/install/setup.bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

正常现象：

```text
/livox/lidar 约 10 Hz
/livox/imu   持续输出
```

如果 `/livox/lidar` 没有数据，先检查雷达网络：

```bash
ping 192.168.10.136
```

## 4. 启动 FAST-LIO

终端 2：

```bash
cd /root/farr_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch farr_bringup slam.launch.py
```

检查 FAST-LIO 输出：

```bash
ros2 topic hz /cloud_registered
ros2 topic hz /Odometry
```

正常现象：

```text
/cloud_registered 持续输出
/Odometry          持续输出
```

说明：

- FAST-LIO 刚启动时偶尔出现 `No point, skip this scan` 可以接受。
- 如果一直刷 `No point`，先确认 `/livox/lidar` 是否稳定输出。
- 如果 Ctrl-C 后 `/cloud_registered` 仍有数据，说明后台可能还有旧 FAST-LIO 进程，应回到第 2 节清理。

## 5. 启动 odom TF 转换

FAST-LIO 输出 `/Odometry` 后，需要把它转换为导航/建图使用的 `/farr_base_odom` 和 TF。

终端 3：

```bash
cd /root/farr_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 run farr_bringup odom_tf_broadcaster
```

检查：

```bash
ros2 topic hz /farr_base_odom
ros2 topic echo /farr_base_odom --once
```

正常日志：

```text
first TF sent: odom->camera_init and odom->base_link
```

## 6. 开启 2.5D 建图

终端 4：

```bash
cd /root/farr_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch farr_bringup mapping_2_5d.launch.py
```

检查：

```bash
ros2 topic hz /farr_obstacle_cloud
ros2 topic hz /farr_2_5d_map
```

正常现象：

```text
/farr_obstacle_cloud 有数据
/farr_2_5d_map       有地图输出
```

如果日志出现：

```text
skip cloud: no TF odom <- camera_init
```

说明第 5 节 `odom_tf_broadcaster` 没有启动或没有正常收到 `/Odometry`。

清空旧地图：

```bash
ros2 service call /reset_2_5d_map std_srvs/srv/Trigger '{}'
```

保存地图：

```bash
ros2 service call /save_2_5d_map std_srvs/srv/Trigger '{}'
```

默认保存位置：

```text
/root/farr_maps/farr_2_5d_map.yaml
/root/farr_maps/farr_2_5d_map.pgm
```

保存后检查：

```bash
ls -lh /root/farr_maps
```

保存地图前建议确认 `/farr_obstacle_cloud` 没有大面积把地面或机器人自身扫进去。

## 7. 开启导航

导航依赖已保存地图：

```text
/root/farr_maps/farr_2_5d_map.yaml
```

启动导航前，建议先清理旧 Nav2：

```bash
pkill -f "farr_nav2_amcl.launch.py" || true
pkill -f "nav2_" || true
pkill -f "amcl" || true
pkill -f "map_server" || true
pkill -f "controller_server" || true
pkill -f "planner_server" || true
pkill -f "bt_navigator" || true
pkill -f "farr_cmd_vel_bridge" || true
```

终端 5：

```bash
cd /root/farr_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch farr_bringup farr_nav2_amcl.launch.py
```

这个 launch 会启动：

```text
robot_state_publisher
odom_tf_broadcaster
障碍点云过滤
pointcloud_to_laserscan
map_server
amcl
Nav2
farr_cmd_vel_bridge
```

检查：

```bash
ros2 topic list | grep -E "/map|/scan|/amcl_pose|/cmd_vel|/farr_base_odom"
ros2 topic echo /cmd_vel
```

注意：

- 遥控器需要拨到允许上位机控制的档位，否则 `/cmd_vel` 有输出但车不动。
- 如果 Nav2 状态异常，最稳妥是杀掉相关进程后重新启动 `farr_nav2_amcl.launch.py`。
- 前端发导航目标时，Web Gateway 会通过 `/navigate_to_pose` 转给 Nav2。

## 8. 开启前端 Web Gateway

如果只想让队友前端显示状态和点云，启动网关即可。

终端 6：

```bash
cd /root/farr_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch farr_bringup web_gateway.launch.py
```

检查 RDK 本机：

```bash
curl http://127.0.0.1:8080/health
curl -I http://127.0.0.1:8080/video/stream
ss -ltnp | grep ':8080'
```

正常结果：

```text
farr_web_gateway ok
0.0.0.0:8080 LISTEN
```

电脑端检查：

```powershell
Test-NetConnection 192.168.140.233 -Port 8080
```

队友前端在电脑上启动：

```powershell
cd 'E:\RM_STM32_Project\RDK_web\RDK_web'
python -m http.server 5500
```

浏览器打开：

```text
http://127.0.0.1:5500/?rdk=192.168.140.233:8080
```

如果队友访问你电脑上启动的前端，把 `127.0.0.1` 换成你电脑的局域网 IP。例如：

```text
http://192.168.140.109:5500/?rdk=192.168.140.233:8080
```

不要在浏览器访问：

```text
http://0.0.0.0:8080/
```

`0.0.0.0` 只是 RDK 的监听地址，不是浏览器访问地址。

## 9. 推荐现场启动顺序

建图流程：

```text
1. 清理旧环境
2. sensors.launch.py
3. slam.launch.py
4. odom_tf_broadcaster
5. mapping_2_5d.launch.py
6. web_gateway.launch.py
7. 前端打开 ?rdk=192.168.140.233:8080
8. 确认点云正常
9. reset_2_5d_map
10. 推车/遥控建图
11. save_2_5d_map
```

导航流程：

```text
1. 确认 /root/farr_maps/farr_2_5d_map.yaml 存在
2. 清理旧 Nav2
3. sensors.launch.py
4. slam.launch.py
5. farr_nav2_amcl.launch.py
6. web_gateway.launch.py
7. 前端打开 ?rdk=192.168.140.233:8080
8. 在前端或 RViz 设置目标
```

## 10. 常用排查

查看关键进程：

```bash
ps -eo pid,comm,args | grep -E "livox|fastlio|slice_mapper|odom_tf|nav2|amcl|map_server|web_gateway" | grep -v grep
```

查看关键 topic：

```bash
ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic hz /cloud_registered
ros2 topic hz /farr_obstacle_cloud
ros2 topic hz /farr_base_odom
```

查看日志：

```bash
tail -80 /tmp/farr_sensors.log
tail -80 /tmp/farr_slam.log
tail -80 /tmp/farr_odom_tf.log
tail -80 /tmp/farr_mapping.log
tail -80 /tmp/farr_gateway.log
```

如果 `ros2 topic list` 或 `ros2 node list` 报：

```text
!rclpy.ok()
```

重启 ROS2 daemon：

```bash
ros2 daemon stop || true
ros2 daemon start || true
```

如果前端状态连接成功但没有点云：

```bash
ros2 topic info /farr_obstacle_cloud -v
ros2 topic info /cloud_registered -v
```

如果 publisher count 为 0，说明点云上游没有启动，不是前端问题。

如果 `/ws/pointcloud` 有数据，前端应显示：

```text
PCV1 点云帧
point_count > 0
```

如果没有摄像头，前端显示 `camera=offline`、`yolo=offline` 是正常的。

