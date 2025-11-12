# Tello-Talent-ROS2

此项目是基于[tello_ros](https://github.com/clydemcqueen/tello_ros)项目的二次开发，使用大疆RoboMaster Tello Talent (RMTT)无人机，集成ORB_SLAM3，并在ROS2-humble环境中进行测试。

[tello_ros](https://github.com/clydemcqueen/tello_ros)项目的详细说明请参阅[src/tello_ros/README.md](src/tello_ros/README.md).



## 安装

此项目在Ubuntu22.04，ROS2-humble环境中进行开发，因此需要首先安装ROS2，对于中国大陆的ROS初学者，推荐使用[鱼香ROS一键安装](https://github.com/fishros/install)工具：

```
wget http://fishros.com/install -O fishros && . fishros
```

正式部署之前需要安装相关依赖：

```
# 安装asio
sudo apt install libasio-dev

# 安装ros附加功能包
sudo apt install ros-$ROS_DISTRO-cv-bridge/
ros-$ROS_DISTRO-camera-calibration-parsers/
ros-$ROS_DISTRO-robot_state_publisher/
ros-$ROS_DISTRO-joint_state_publisher/

# 确保当前Python环境下已经安装以下库
pip3 install catkin_pkg empy==3.3.4 lark numpy	# 也可使用conda等工具

# 下载仓库
cd ~/WorkSpace	# 自定义一个下载目录
git clone https://github.com/FallThrive/Tello-Talent-ROS2.git tello_ws
```

### 基本安装 tello_ros

编译`ros2_shared` `tello_description` `tello_driver` `tello_msgs`四个功能包即可实现无人机基本飞行功能。

原项目的`tello_gazebo`功能包适配的是Gazabo Classic版本，因此未安装Gazabo Classic（例如安装的为Gazebo Harmonic）建议跳过此功能包的编译。

```
cd ~/WorkSpace/tello_ws	# 以自己的项目目录为准
chmod +x src/tello_ros/tello_description/src/replace.py
colcon build --symlink-install --packages-select ros2_shared tello_description tello_driver tello_msgs
```

当然，如果ubuntu系统支持Gazebo Classic，也可以使用以下命令安装（提示：目前Gazebo Classic已经停止维护）：

```
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs
cd ~/WorkSpace/tello_ws	# 以自己的项目目录为准
colcon build --symlink-install --packages-select tello_gazebo
```

### 可选安装 tello_extras

#### tello_slam

##### 1. 安装Pangolin

下载`Pangolin`到自定义目录，`Pangolin`被拆分为几个组件，安装时可以只包含所需的组件，大多数依赖项都是可选的，详细的说明可以直接参阅[官方文档](https://github.com/stevenlovegrove/Pangolin)。

```
# 下载仓库
cd ~/Source	# 自定义一个下载目录
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin

# 安装依赖项
./scripts/install_prerequisites.sh recommended	# 这里只安装了必需的

# 配置与编译（也可使用Ninja提高编译速度）
cmake -B build
cmake --build build
sudo cmake --build build --target install

# 安装Python绑定
cmake --build build -t pypangolin_pip_install

# 添加到LD_LIBRARY_PATH环境变量
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
```

##### 2. 安装ORB_SLAM3

首先下载ORB_SLAM3的仓库，详细说明可以查阅[官方文档](https://github.com/UZ-SLAMLab/ORB_SLAM3)，这里推荐使用开发者zang09修改后的[仓库](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED)，从C++11支持到了C++14，在ROS2的foxy（zang09测试）和humble环境中通过了测试。

```
cd ~/Source	# 自定义一个安装目录
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3				# 原版
git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3	# 推荐

cd ORB_SLAM3
chmod +x build.sh
./build.sh

# 安装Sophus
cd Thirdparty/Sophus/build
sudo make install
```

##### 3. 安装OctoMap

tello_map使用OctoMap将点云数据发布到地图，因此，需要安装OctoMap：

```
sudo apt install ros-$ROS_DISTRO-octomap*
```

该命令将会安装以下功能包：

```
octomap
octomap_mapping
octomap_msgs
octomap_ros
octomap_rviz_plugins
octomap_server
```

##### 4. 编译tello_slam功能包

设置[FindORB_SLAM3.cmake](src/tello_slam/CMakeModules/FindORB_SLAM3.cmake)里第8行`ORB_SLAM3_ROOT_DIR`为自己的ORB_SLAM3路径。

设置[CMakeLists.txt](src/tello_slam/CMakeLists.txt)里第5行`ENV{PYTHONPATH}`为自己的ROS Python功能包路径。

```
cd ~/WorkSpace/tello_ws	# 以自己的项目目录为准
cd tello/extras/tello_slam/vocabulary
tar -xf ORBvoc.txt.tar.gz
colcon build --symlink-install --packages-select tello_slam
```



## 新特性

项目功能包整体路径如下：

```
tello_ws/
├── src/
│   ├── ros2_shared/      # ROS2共享工具包
│   ├── tello_ros/        # Tello无人机ROS驱动主包
│   │   ├── tello_description/  # URDF模型描述文件
│   │   ├── tello_driver/       # 无人机驱动节点
│   │   ├── tello_gazebo/       # Gazebo仿真支持
│   │   └── tello_msgs/         # 自定义消息类型
│   └── tello_extras/     # 扩展功能包
│       ├── tello_slam/         # SLAM功能实现包
│       └── tello_yolo/			# YOLO目标跟随（计划中）
├── build/                # 编译输出目录（已忽略）
├── install/              # 安装目录（已忽略）
└── log/                  # 日志目录（已忽略）
```

### 使用键盘控制

在`tello_driver`功能包的`tello_key_main`节点，可以使用键盘控制无人机，采用美国手布局方式，即左手控制垂直运动与左右转向，右手控制前进后退与左右平移，与手柄控制保持一致：

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

节点每10ms会发布一次`/cmd_vel`话题，如果键盘没有输入则将所有速度置零，因此需要长按来控制无人机运动。

如果需要修改键盘控制的无人机速度，可以修改[tello_key_node.hpp](src/tello_ros/tello_driver/include/tello_key_node.hpp)中的`LINEAR_SPEED`与`ANGULAR_SPEED`.

在项目目录打开终端，一个终端运行`tello_driver_main`节点，另一个终端运行`tello_key_main`节点以进行使用：

```
# 终端1
. install/setup.bash
ros2 run tello_driver tello_driver_main

# 终端2
. install/setup.bash
ros2 run tello_driver tello_key_main
```

### SLAM建图

该功能参考[ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2)项目，部署在`tello_extras`文件夹下`tello_slam`功能包，使用[ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)进行纯视觉单目相机建图，使用[OctoMap](https://github.com/OctoMap/octomap)建立栅格地图。

tello系列无人机搭载的为单目相机，并且SDK提供的并不是完整IMU数据，因此这里只保留了单目相机`mono`模式，默认使用ORB词汇表[ORBvoc.txt](src/tello_extras/tello_slam/vocabulary/ORBvoc.txt.tar.gz)和tello相机配置文件[tello_cam.yaml](src/tello_extras/tello_slam/config/tello_cam.yaml).

`mono`节点将点云数据发布到`~pointcloud`话题，消息类型为`sensor_msgs/PointCloud2`；同时，实现`map -> base_link`的坐标系映射。

在终端中运行如下命令，即可运行ORB_SLAM3和无人机模型[tello.urdf](src/tello_ros/tello_description/urdf/tello.urdf)；同时启动手柄控制节点，并使用配置文件[slam.rviz](src/tello_extras/tello_slam/config/slam.rviz)加载RViz2：

```
. install/setup.bash
ros2 launch tello_slam slam_launch.py
```

以上，将发布的tf坐标变换为：

```
map -> base_link -> camera_link
```

获取点云话题`~pointcloud`后，可以使用OctoMap建立栅格地图，终端运行如下命令，可以在启用ORB_SLAM3的同时，调用配置文件[tello_octomap.yaml](src/tello_extras/tello_slam/config/tello_octomap.yaml)以启用OctoMap：

```
. install/setup.bash
ros2 launch tello_slam slam_map_launch.py
# 无需额外再启动slam_launch.py
```

OctoMap将会发布以下话题：

- `~octomap_full` - 完整概率三维占用地图  
- `~octomap_binary` - 精简二进制三维占用地图  
- `~octomap_point_cloud_centers` - 占用体素中心点云  
- `~occupied_cells_vis_array` - 占用体素可视化方块  
- `~free_cells_vis_array` - 空闲体素可视化方块  
- `~projected_map` - 二维投影占用栅格



## 鸣谢

- `ros2_shared` 来自 https://github.com/ptrmu/ros2_shared
- `tello_ros` 修改自 https://github.com/clydemcqueen/tello_ros
- `tello_slam` 修改自 https://github.com/zang09/ORB_SLAM3_ROS2



## ToDoList

- [x] 编译`tello_ros`原项目，在ROS2-humble中成功实现基本功能
- [x] 使用键盘控制无人机运动
- [x] 使用ORB_SLAM3发布点云话题
- [x] 使用OctoMap建立栅格地图
- [ ] 使用YOLO+PID进行目标检测与跟随
- [ ] 使用强化学习进行运动控制