#include "monocular-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Core>
#include <limits>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System *pSLAM)
    : Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "image_raw",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

    // 创建点云发布器
    m_pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 1);

    // 创建TF广播器
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 创建定时器，定期发布点云数据和TF变换
    m_pointcloud_timer = this->create_wall_timer(
        std::chrono::milliseconds(100), // 10Hz
        std::bind(&MonocularSlamNode::PublishMapPoints, this));

    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // 生成带时间戳的文件名
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    std::string timestamp = ss.str();

    // Save key frame trajectories with timestamp
    std::string kf_file_path = "src/tello_slam/results/KeyFrameTrajectory_" + timestamp + ".txt";
    m_SLAM->SaveKeyFrameTrajectoryTUM(kf_file_path);
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout << "one frame has been sent" << std::endl;
    // 跟踪当前图像，并缓存最新位姿（仅此处调用TrackMonocular）
    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    if (!Tcw.matrix().isZero(0))
    {
        std::lock_guard<std::mutex> lk(m_pose_mutex);
        m_last_Tcw = Tcw;
        m_has_pose = true;
        m_last_stamp = msg->header.stamp;
    }
}

void MonocularSlamNode::PublishMapPoints()
{
    builtin_interfaces::msg::Time stamp;
    {
        std::lock_guard<std::mutex> lk(m_pose_mutex);
        if (!m_has_pose)
        {
            return; // 尚无有效位姿/时间戳
        }
        stamp = m_last_stamp;
    }
    PublishMapTF(stamp);
    PublishPointCloud(stamp);
}

void MonocularSlamNode::PublishMapTF(const builtin_interfaces::msg::Time &stamp)
{
    // 从缓存读取最近一次TrackMonocular得到的相机位姿
    Sophus::SE3f Tcw;
    {
        std::lock_guard<std::mutex> lk(m_pose_mutex);
        if (!m_has_pose)
        {
            return; // 还没有有效位姿，直接返回
        }
        Tcw = m_last_Tcw;
    }

    // 检查位姿是否有效
    if (!Tcw.matrix().isZero(0))
    {
        // 计算世界到相机的变换 Twc = Tcw.inverse()
        Sophus::SE3f Twc = Tcw.inverse();
        auto world_translation = Twc.translation();
        auto world_rotation = Twc.so3().matrix();

        // ORB-SLAM3使用计算机视觉坐标系 (X右, Y下, Z前)
        // ROS使用机器人坐标系 (X前, Y左, Z上)
        // 需要进行坐标系转换
        Eigen::Matrix3f R_cv_to_ros;
        R_cv_to_ros << 0, 0, 1,
                      -1, 0, 0,
                       0,-1, 0;

        // 将世界到相机的变换从CV坐标系转换到ROS坐标系
        // 对于位置：p_ros = R_cv_to_ros * p_cv
        // 对于旋转：R_ros = R_cv_to_ros * R_cv * R_cv_to_ros^T
        Eigen::Vector3f ros_world_position = R_cv_to_ros * world_translation;
        Eigen::Matrix3f ros_world_rotation = R_cv_to_ros * world_rotation * R_cv_to_ros.transpose();

        // 根据URDF定义，camera_link在base_link前方0.035m处
        // 所以base_link在camera_link后方-0.035m处（在camera_link坐标系中）
        Eigen::Vector3f camera_to_base(-0.035, 0.0, 0.0); // 在camera_link坐标系中
        
        // 转换到world坐标系：base_link位置 = world位置 + world旋转 * (camera到base的偏移)
        auto base_link_translation = ros_world_position + ros_world_rotation * camera_to_base;

        geometry_msgs::msg::TransformStamped baseTransform;
        baseTransform.header.stamp = stamp;
        baseTransform.header.frame_id = "map";
        baseTransform.child_frame_id = "base_link";

        baseTransform.transform.translation.x = base_link_translation.x();
        baseTransform.transform.translation.y = base_link_translation.y();
        baseTransform.transform.translation.z = base_link_translation.z();

        // 使用转换后的旋转作为base_link的旋转
        Eigen::Quaternionf q(ros_world_rotation);
        baseTransform.transform.rotation.x = q.x();
        baseTransform.transform.rotation.y = q.y();
        baseTransform.transform.rotation.z = q.z();
        baseTransform.transform.rotation.w = q.w();

        // 广播TF变换
        m_tf_broadcaster->sendTransform(baseTransform);
    }
}

void MonocularSlamNode::PublishPointCloud(const builtin_interfaces::msg::Time &stamp)
{
    // 获取ORB-SLAM3的地图点
    auto map_points = m_SLAM->GetTrackedMapPoints();

    // 创建点云消息
    auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // 设置点云的基本信息
    cloud_msg->header.stamp = stamp;
    cloud_msg->header.frame_id = "map";

    // ORB-SLAM3使用计算机视觉坐标系 (X右, Y下, Z前)
    // ROS使用机器人坐标系 (X前, Y左, Z上)
    // 需要进行坐标系转换
    Eigen::Matrix3f R_cv_to_ros;
    R_cv_to_ros << 0, 0, 1,
                  -1, 0, 0,
                   0,-1, 0;

    // 至少有一个点才处理
    if (!map_points.empty())
    {
        // 调整点云大小
        cloud_msg->height = 1;
        cloud_msg->width = map_points.size();
        cloud_msg->is_dense = false;

        // 定义点云字段
        sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");

        // 填充点云数据
        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

        for (size_t i = 0; i < map_points.size(); ++i)
        {
            if (map_points[i] && !map_points[i]->isBad())
            {
                // 获取地图点的世界坐标
                auto pos = map_points[i]->GetWorldPos();
                // 应用坐标系变换
                Eigen::Vector3f ros_pos = R_cv_to_ros * pos;
                *iter_x = ros_pos(0);
                *iter_y = ros_pos(1);
                *iter_z = ros_pos(2);
            }
            else
            {
                *iter_x = std::numeric_limits<float>::quiet_NaN();
                *iter_y = std::numeric_limits<float>::quiet_NaN();
                *iter_z = std::numeric_limits<float>::quiet_NaN();
            }
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
    }

    // 发布点云
    m_pointcloud_publisher->publish(*cloud_msg);
}