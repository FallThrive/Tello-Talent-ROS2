#include "tello_driver_node.hpp"

#include <libavutil/frame.h>
#include <opencv2/highgui.hpp>

#include "camera_calibration_parsers/parse.hpp"

namespace tello_driver
{

  // Notes on Tello video:
  // -- frames are always 960x720.
  // -- frames are split into UDP packets of length 1460.
  // -- normal frames are ~10k, or about 8 UDP packets.
  // -- keyframes are ~35k, or about 25 UDP packets.
  // -- keyframes are always preceded by an 8-byte UDP packet and a 13-byte UDP packet -- markers?
  // -- the h264 parser will consume the 8-byte packet, the 13-byte packet and the entire keyframe without
  //    generating a frame. Presumably the keyframe is stored in the parser and referenced later.

  VideoSocket::VideoSocket(TelloDriverNode *driver, unsigned short video_port, const std::string &camera_info_path) :
    TelloSocket(driver, video_port)
  {
    std::string camera_name;
    if (camera_calibration_parsers::readCalibration(camera_info_path, camera_name, camera_info_msg_)) {
      RCLCPP_INFO(driver_->get_logger(), "Parsed camera info for '%s'", camera_name.c_str());
    } else {
      RCLCPP_ERROR(driver_->get_logger(), "Cannot get camera info");
    }

    buffer_ = std::vector<unsigned char>(2048);
    seq_buffer_ = std::vector<unsigned char>(65536);
    listen();
  }

  // Process a video packet from the drone
  void VideoSocket::process_packet(size_t r)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    receive_time_ = driver_->now();

    if (!receiving_) {
      // First packet
      RCLCPP_INFO(driver_->get_logger(), "Receiving video");
      receiving_ = true;
      seq_buffer_next_ = 0;
      seq_buffer_num_packets_ = 0;
    }

    if (seq_buffer_next_ + r >= seq_buffer_.size()) {
      RCLCPP_ERROR(driver_->get_logger(), "Video buffer overflow, dropping sequence");
      seq_buffer_next_ = 0;
      seq_buffer_num_packets_ = 0;
      return;
    }

    std::copy(buffer_.begin(), buffer_.begin() + r, seq_buffer_.begin() + seq_buffer_next_);
    seq_buffer_next_ += r;
    seq_buffer_num_packets_++;

    // If the packet is < 1460 bytes then it's the last packet in the sequence
    if (r < 1460) {
      decode_frames();

      seq_buffer_next_ = 0;
      seq_buffer_num_packets_ = 0;
    }
  }

  // Decode frames
  void VideoSocket::decode_frames()
  {
    size_t next = 0;

    try {
      while (next < seq_buffer_next_) {
        // Parse h264
        ssize_t consumed = decoder_.parse(seq_buffer_.data() + next, seq_buffer_next_ - next);

        // Is a frame available?
        if (decoder_.is_frame_available()) {
          // Decode the frame
          const AVFrame &frame = decoder_.decode_frame();

          // Convert pixels from YUV420P to BGR24
          int size = converter_.predict_size(frame.width, frame.height);
          unsigned char bgr24[size];
          converter_.convert(frame, bgr24);

          // Convert to cv::Mat
          cv::Mat mat{frame.height, frame.width, CV_8UC3, bgr24};

          // Synchronize ROS messages
          auto stamp = driver_->now();

          if (driver_->count_subscribers(driver_->image_pub_->get_topic_name()) > 0) {
            std_msgs::msg::Header header{};
            header.frame_id = "camera_frame";
            header.stamp = stamp;
            cv_bridge::CvImage cv_image{header, sensor_msgs::image_encodings::BGR8, mat};
            sensor_msgs::msg::Image sensor_image_msg;
            cv_image.toImageMsg(sensor_image_msg);
            driver_->image_pub_->publish(sensor_image_msg);
          }

          if (driver_->count_subscribers(driver_->camera_info_pub_->get_topic_name()) > 0) {
            camera_info_msg_.header.stamp = stamp;
            driver_->camera_info_pub_->publish(camera_info_msg_);
          }

          // Draw battery and ToF height
          std::string bat_text = "Bat: " + std::to_string(driver_->battery_percentage_) + "%";
          std::string tof_text = "ToF: " + std::to_string(driver_->tof_) + "cm";

          // Use green color
          cv::Scalar color(0, 255, 0);
          int font = cv::FONT_HERSHEY_SIMPLEX;
          double scale = 1.0;
          int thickness = 2;
          int baseline = 0;

          cv::Size bat_size = cv::getTextSize(bat_text, font, scale, thickness, &baseline);
          cv::Size tof_size = cv::getTextSize(tof_text, font, scale, thickness, &baseline);

          // Left bottom
          cv::putText(mat, bat_text, cv::Point(10, mat.rows - 10), font, scale, color, thickness);

          // Right bottom
          cv::putText(mat, tof_text, cv::Point(mat.cols - tof_size.width - 10, mat.rows - 10), font, scale, color, thickness);

          // Display
          cv::imshow("frame", mat);
          cv::waitKey(1);
        }

        next += consumed;
      }
    }
    catch (std::runtime_error e) {
      RCLCPP_ERROR(driver_->get_logger(), e.what());
    }
  }

} // namespace tello_driver