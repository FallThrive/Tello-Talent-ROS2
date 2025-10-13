# Tello-Talent-ROS2

此项目是基于[tello_ros](https://github.com/clydemcqueen/tello_ros)项目的二次开发，使用大疆RoboMaster Tello Talent (RMTT)无人机，集成ORB_SLAM3，并在ROS2-humble环境中进行测试

[tello_ros](https://github.com/clydemcqueen/tello_ros)项目的详细说明请参阅[src/tello_ros/README.md](src/tello_ros/README.md)

## 安装

此项目在Ubuntu22.04，ROS2-humble环境中进行开发，因此需要首先安装ROS2，对于中国大陆的ROS初学者，推荐使用[鱼香ROS一键安装](https://github.com/fishros/install)工具

```
wget http://fishros.com/install -O fishros && . fishros
```

正式部署之前需要安装相关依赖

```
# 安装asio
sudo apt install libasio-dev

# 安装ros附加功能包
sudo apt install ros-humble-cv-bridge ros-humble-camera-calibration-parsers

# 确保当前Python环境下已经安装以下库
pip3 install catkin_pkg empy==3.3.4 lark numpy	# 也可使用conda等工具

# 下载仓库
cd ~/WorkSpace	# 自定义一个下载目录
git clone https://github.com/FallThrive/Tello-Talent-ROS2.git tello_ws
```

### 基本安装

编译`ros2_shared` `tello_description` `tello_driver` `tello_msgs`四个功能包即可实现无人机基本飞行功能

原项目的`tello_gazebo`功能包适配的是Gazabo Classic版本，因此未安装Gazabo Classic（例如安装的为Gazebo Harmonic）建议跳过此功能包的编译

```
cd ~/WorkSpace/tello_ws	# 以自己的项目目录为准
chmod +x src/tello_ros/tello_description/src/replace.py
colcon build --symlink-install --packages-select ros2_shared tello_description tello_driver tello_msgs
```

### 可选安装：tello_slam

#### 1. 安装Pangolin

下载`Pangolin`到自定义目录，`Pangolin`被拆分为几个组件，安装时可以只包含所需的组件，大多数依赖项都是可选的，详细的说明可以直接参阅[官方文档](https://github.com/stevenlovegrove/Pangolin)

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
```

#### 2. 安装ORB_SLAM3

首先下载ORB_SLAM3的仓库，详细说明可以查阅[官方文档](https://github.com/UZ-SLAMLab/ORB_SLAM3)，这里推荐使用开发者zang09修改后的[仓库](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED)，从C++11支持到了C++14，在ROS2的foxy（zang09测试）和humble环境中通过了测试

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

#### 3. 编译tello_slam功能包

设置[FindORB_SLAM3.cmake](src/tello_slam/CMakeModules/FindORB_SLAM3.cmake)里第8行`ORB_SLAM3_ROOT_DIR`为自己的ORB_SLAM3路径

设置[CMakeLists.txt](src/tello_slam/CMakeLists.txt)里第5行`ENV{PYTHONPATH}`为自己的ROS Python功能包路径

```
cd ~/WorkSpace/tello_ws	# 以自己的项目目录为准
tar -xf src/tello_slam/vocabulary/ORBvoc.txt.tar.gz -C src/tello_slam/vocabulary
colcon build --symlink-install --packages-select tello_slam
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
  - [ ] 将点云数据发布到ROS话题
- [ ] 使用大疆RMTT开源控制器拓展配件，使用路由器模式连接其他路由器控制
- [ ] 重构`tello_gazebo`功能包，适配Gazebo Harmonic
- [ ] 基于YOLO实现目标跟随