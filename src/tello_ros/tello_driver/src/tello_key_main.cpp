#include "tello_key_node.hpp"

// Launch TelloKey with use_intra_process_comms=true

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create single-threaded executor
  rclcpp::executors::SingleThreadedExecutor executor;

  // Use IPC
  // Note: this is a NOP, as there's only 1 node in this process
  rclcpp::NodeOptions options{};
  options.use_intra_process_comms(true);

  // Create and add key node
  auto key_node = std::make_shared<tello_key::TelloKeyNode>(options);
  executor.add_node(key_node);

  // Spin until rclcpp::ok() returns false
  executor.spin();

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}