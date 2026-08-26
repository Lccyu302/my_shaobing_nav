# 新竹战队 26赛季哨兵导航

主要的创新：

1. 实现FAST-LIO2的重定位模式，不需要另外运行重定位算法。

2. 改进navigation2的故障恢复行为，卡住时机器人会向无碰撞的方向运动。

3. 采用决策树进行行为控制。

4. 中南视觉开源中新增导航数据包

5. 视觉导航联合实现追击功能


## 包说明

| Package Name | Description |
|--------------|-------------|
| auto_aim_interfaces | 自瞄接口 |
| [autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment) | 地形分析包terrain_analysis和terrain_analysis_ext |
| [BehaviourTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) | 行为树包|
| cmd_chassis | 底盘解算控制 |
| control_panel | 模仿裁判系统发消息 |
| FAST_LIO | 修改版fastlio，具备建图和重定位功能（须配合icp_relocalizatiion使用） |
| icp_relocalization | 基于icp实现的重定位，须配合修改版FAST_LIO使用 |
| livox_ros_driver2 | livox雷达驱动 |
| nav2_plugins |
| rm_decision_cpp | 烧饼决策系统 |
| rm_interfaces | 通讯协议 |
| sentry_bringup | 哨兵启动文件 |
| rm_bringup | 哨兵视觉启动文件 |
| sentry_description | 烧饼urdf |
| rm_robor_description | 烧饼视觉urdf |
| rm_auto_aim | 自瞄主程序|
| rm_interfaces | 裁判系统自定义数据包|
| rm_utils | 数学工具包|
| rm_hardware_driver | 串口与相机驱动包|
## 配置流程
Docker Image Quick Start
Get the image
```bash
docker pull polarisxq/scurm:sentry_nav
```bash
cd SCURM_SentryNavigation
 ```bash
docker build -t scurm:sentry_nav .

#run
 This image support both DockerDesktop and Docker in Linux.

 for Linux user, run
```bash
sudo xhost + && sudo docker run -dit --network=host --pid=host --privileged -v /dev:/dev -e DISPLAY=${DISPLAY} scurm:sentry_nav

#Insatll Groot2

 You must install groot2 manually inside the container.
 ```bash
chmod +x . Groot2-1.3.1-linux-installer.run
 ```bash
./Groot2-1.3.1-linux-installer.run
 ```bash
groot2
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```bash
rosdep install --from-paths src --ignore-src -r -y

报错差什么包补什么包，livox_driver的包报错的话先将他单独编译，再整体编译，文件较多，编译时采用三或四线程
## LAUNCH

### MAPPING

- launch mapping node

```bash
ros2 launch sentry_bringup mapping.launch.py
```

- save map

# occupancy grid map
```bash
ros2 run nav2_map_server map_saver_cli -t /projected_map -f test_map --fmt png
```
# save pcd
```bash
ros2 service call /map_save std_srvs/srv/Trigger
```
# view pcd
```bash
pcl_viewer -fc 255,255,0 -ax 3 < filename >
```

### LAUNCH ALL

```bash
ros2 launch sentry_bringup bringup_all_in_one.launch.py
ros2 launch rm_bringup bringup.launch.py
ros2 run control_panel control_panel
```


mono pinhole calibration...
D = [-0.08583039435866688, 0.1372747251909091, 0.00038324501890096965, 1.6706078998886512e-06, 0.0]
K = [1285.900456215922, 0.0, 633.2682886840132, 0.0, 1286.35971367529, 494.0650796905105, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [1270.37841796875, 0.0, 632.8807573533413, 0.0, 0.0, 1271.9102783203125, 493.57928865727445, 0.0, 0.0, 0.0, 1.0, 0.0]
None
# oST version 5.0 parameters


[image]

width
1280

height
1024

[narrow_stereo]

camera matrix
1285.900456 0.000000 633.268289
0.000000 1286.359714 494.065080
0.000000 0.000000 1.000000

distortion
-0.085830 0.137275 0.000383 0.000002 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
1270.378418 0.000000 632.880757 0.000000
0.000000 1271.910278 493.579289 0.000000
0.000000 0.000000 1.000000 0.000000

('Wrote calibration data to', '/tmp/calibrationdata.tar.gz')
D = [-0.08583039435866688, 0.1372747251909091, 0.00038324501890096965, 1.6706078998886512e-06, 0.0]
K = [1285.900456215922, 0.0, 633.2682886840132, 0.0, 1286.35971367529, 494.0650796905105, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [1270.37841796875, 0.0, 632.8807573533413, 0.0, 0.0, 1271.9102783203125, 493.57928865727445, 0.0, 0.0, 0.0, 1.0, 0.0]
# oST version 5.0 parameters


[image]

width
1280

height
1024

[narrow_stereo]

camera matrix
1285.900456 0.000000 633.268289
0.000000 1286.359714 494.065080
0.000000 0.000000 1.000000

distortion
-0.085830 0.137275 0.000383 0.000002 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
1270.378418 0.000000 632.880757 0.000000
0.000000 1271.910278 493.579289 0.000000
0.000000 0.000000 1.000000 0.000000


