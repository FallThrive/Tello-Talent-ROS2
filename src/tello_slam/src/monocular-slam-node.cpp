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
    m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
}

void MonocularSlamNode::PublishMapPoints()
{
    builtin_interfaces::msg::Time stamp = this->now();
    PublishMapTF(stamp);
    PublishPointCloud(stamp);
}

void MonocularSlamNode::PublishMapTF(const builtin_interfaces::msg::Time &stamp)
{
    // 获取ORB-SLAM3的相机位姿 (从跟踪器获取当前帧的位姿)
    auto Tcw = m_SLAM->TrackMonocular(cv::Mat(), 0); // 获取最新位姿，传入空图像和0时间戳

    // 检查位姿是否有效
    if (!Tcw.matrix().isZero(0))
    {
        // 获取相机在map坐标系中的位置和姿态
        auto camera_translation = Tcw.translation();
        auto camera_rotation = Tcw.so3().matrix();

        // 创建map->base_link的变换
        // base_link位置 = 相机位置 + 相机旋转 * (base_link相对于camera_link的位置)
        Eigen::Vector3f base_to_camera(-0.035, 0.0, 0.0); // base_link在camera_link后方0.035m处
        auto base_link_translation = camera_translation + camera_rotation * base_to_camera;

        geometry_msgs::msg::TransformStamped baseTransform;
        baseTransform.header.stamp = stamp;
        baseTransform.header.frame_id = "map";
        baseTransform.child_frame_id = "base_link";

        baseTransform.transform.translation.x = base_link_translation.x();
        baseTransform.transform.translation.y = base_link_translation.y();
        baseTransform.transform.translation.z = base_link_translation.z();

        // 使用相机的旋转作为base_link的旋转
        Eigen::Quaternionf q(camera_rotation);
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
                *iter_x = pos(0);
                *iter_y = pos(1);
                *iter_z = pos(2);
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