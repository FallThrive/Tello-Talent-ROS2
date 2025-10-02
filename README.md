# RMTT-ROS2

此项目是基于[tello_ros](https://github.com/clydemcqueen/tello_ros)项目的二次开发，适应大疆RoboMaster Tello Talent (RMTT)无人机的拓展配件特性，并在ROS2-humble环境中进行测试

[tello_ros](https://github.com/clydemcqueen/tello_ros)项目的详细说明请参阅[src/tello_ros/README.md](src/tello_ros/README.md)

## 安装

此项目在Ubuntu22.04，ROS2-humble环境中进行开发，因此需要首先安装ROS2，对于中国大陆的ROS初学者，推荐使用[鱼香ROS一键安装](https://github.com/fishros/install)工具

```
wget http://fishros.com/install -O fishros && . fishros
```

在项目首次编译前请如下运行命令安装相关依赖，[tello_ros](https://github.com/clydemcqueen/tello_ros)项目并不直接基于[DJITelloPy](https://github.com/damiafuentes/DJITelloPy)实现，而是使用UDP与RMTT无人机通信，因此无需额外安装`djitellopy`

```
sudo apt install libasio-dev
sudo apt install ros-humble-cv-bridge ros-humble-camera-calibration-parsers
```

将此项目仓库下载到本地

```
cd ~/WorkSpace	# 以自己的下载目录为准
git clone https://github.com/FallThrive/RMTT-ROS2.git
```

在项目目录路径打开终端

```
cd ~/WorkSpace/RMTT-ROS2	# 以自己的项目目录为准
chmod +x src/tello_ros/tello_description/src/replace.py
```

原项目的`tello_gazebo`功能包适配的是Gazabo Classic版本，因此未安装Gazabo Classic（例如安装的为Gazebo Harmonic）建议跳过此功能包的编译

此外，如未完成`ORB-SLAM3`部署或是不需要该功能，请跳过`orbslam3`功能包的编译

```
cd ~/WorkSpace/RMTT-ROS2	# 以自己的项目目录为准
colcon build --packages-select ros2_shared tello_description tello_driver tello_msgs
```

## 新特性

### 使用键盘控制

在`tello_driver`功能包的`tello_key_main`节点，可以使用键盘控制无人机，采用美国手布局方式，即左手控制垂直运动与左右转向，右手控制前进后退与左右平移，与手柄控制保持一致

```
---------------------------
T: Takeoff, B: Land, Q: Quit
---------------------------
Left Hand    Right Hand
   W             I    
 A S D         J K L  
---------------------------
W/S: Up/Down, A/D: Yaw Left/Right
I/K: Forward/Backward, J/L: Left/Right
Hold a button to move, release to stop.
---------------------------
```

每10ms会发布一次`/cmd_vel`话题，如果键盘没有输入则将所有速度置零，因此需要长按来控制无人机运动

如果需要修改键盘控制的无人机速度，可以修改[tello_key_node.hpp](src/tello_ros/tello_driver/include/tello_key_node.hpp)中的`LINEAR_SPEED`与`ANGULAR_SPEED`

在一个终端运行`tello_driver_main`节点，另一个终端运行`tello_key_main`节点以进行使用

```
# 终端1
ros2 run tello_driver tello_driver_main

# 终端2
ros2 run tello_driver tello_key_main
```

## 鸣谢

- `ros2_shared` 来自 https://github.com/ptrmu/ros2_shared
- `tello_ros` 修改自 https://github.com/clydemcqueen/tello_ros
- `tello_slam` 修改自 https://github.com/zang09/ORB_SLAM3_ROS2

## ToDoList

- [x] 编译`tello_ros`原项目，在ROS2-humble中成功实现基本功能
- [x] 使用键盘控制无人机起降与运动
- [ ] 部署`ORB-SLAM3`并实现建图
  - [x] 部署`ORB-SLAM3`以及`ORB_SLAM3_ROS2`项目并成功编译
  - [x] 完成tello相机的标定
  - [x] 精简功能包，移除不需要的配置文件等
  - [ ] 测试SLAM建图效果
- [ ] 使用大疆RMTT开源控制器拓展配件，使用路由器模式连接其他路由器控制
- [ ] 重构`tello_gazebo`功能包，适配Gazebo Harmonic
- [ ] 基于YOLO实现目标跟随