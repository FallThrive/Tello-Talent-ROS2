#!/usr/bin/env python3

"""
用于在rviz中显示tello无人机模型、启动SLAM节点和SLAM可视化配置
此启动文件发布Tello URDF并在RViz2中显示，同时启动SLAM节点和SLAM相关的RViz配置
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 获取包共享目录
    pkg_tello_description = get_package_share_directory('tello_description')
    pkg_tello_driver = get_package_share_directory('tello_driver')
    pkg_tello_slam = get_package_share_directory('tello_slam')

    # 使用tello.urdf文件定义机器人描述
    urdf_file_path = os.path.join(pkg_tello_description, 'urdf', 'tello.urdf')
    
    # 读取URDF文件内容
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # 声明启动参数
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='如果设置为true则启动RVIZ'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='如果为true则使用仿真(Gazebo)时钟'
    )

    # 机器人状态发布节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_desc, value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # 关节状态发布节点（用于在RViz中手动控制关节）
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # RViz节点，使用SLAM专用配置
    rviz_config_file = os.path.join(pkg_tello_slam, 'config', 'slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # SLAM节点
    vocabulary_file = os.path.join(pkg_tello_slam, 'vocabulary', 'ORBvoc.txt')
    settings_file = os.path.join(pkg_tello_slam, 'config', 'tello_cam.yaml')
    
    mono_node = Node(
        package='tello_slam',
        executable='mono',
        name='tello_slam_mono',
        output='screen',
        arguments=[vocabulary_file, settings_file]
    )

    # 包含来自tello_driver的teleop启动文件
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tello_driver, 'launch', 'teleop_launch.py')
        )
    )

    # 创建并返回启动描述
    return LaunchDescription([
        use_rviz_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        mono_node,
        teleop_launch
    ])