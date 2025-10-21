#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "MapPoint.h"

#include "utility.hpp"

#include <vector>

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System *pSLAM);

    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void PublishMapPoints();
    void PublishMapTF(const builtin_interfaces::msg::Time &stamp);
    void PublishPointCloud(const builtin_interfaces::msg::Time &stamp);

    ORB_SLAM3::System *m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher;
    rclcpp::TimerBase::SharedPtr m_pointcloud_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
};

#endif