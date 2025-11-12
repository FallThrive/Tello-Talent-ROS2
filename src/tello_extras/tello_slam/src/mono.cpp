#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"

#include "System.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char **argv)
{
    // 如果没有提供参数，则使用默认值
    std::string vocabulary_file;
    std::string settings_file;

    // 获取tello_slam包的共享目录
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("tello_slam");
    
    // 构建默认文件路径
    vocabulary_file = package_share_directory + "/vocabulary/ORBvoc.txt";
    settings_file = package_share_directory + "/config/tello_cam.yaml";

    if (argc >= 2) {
        vocabulary_file = argv[1];
    }
    if (argc >= 3) {
        settings_file = argv[2];
    }
    
    if (argc < 3)
    {
        std::cerr << "\nUsage: ros2 run tello_slam mono path_to_vocabulary path_to_settings" << std::endl;
        std::cerr << "Using default vocabulary file: " << vocabulary_file << std::endl;
        std::cerr << "Using default settings file: " << settings_file << std::endl;
    }

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool visualization = true;
    ORB_SLAM3::System SLAM(vocabulary_file, settings_file, ORB_SLAM3::System::MONOCULAR, visualization);

    auto node = std::make_shared<MonocularSlamNode>(&SLAM);
    std::cout << "============================ " << std::endl;\

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}