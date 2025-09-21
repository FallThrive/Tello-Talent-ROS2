#include "tello_key_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tello_msgs/srv/tello_action.hpp"
#include <iostream>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

namespace tello_key
{

    TelloKeyNode::TelloKeyNode(const rclcpp::NodeOptions &options) :
        Node("tello_key", options)
    {
        // 创建速度指令发布者
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    
        // 创建无人机动作服务客户端
        tello_client_ = this->create_client<tello_msgs::srv::TelloAction>("tello_action");
    
        // 启动键盘输入读取线程
        input_thread_ = std::thread(&TelloKeyNode::readKeyboardInput, this);
    
        RCLCPP_INFO(this->get_logger(), "Tello keyboard control node initialized.");
        RCLCPP_INFO(this->get_logger(), "---------------------------");
        RCLCPP_INFO(this->get_logger(), "T: Takeoff, B: Land, Q: Quit");
        RCLCPP_INFO(this->get_logger(), "---------------------------");
        RCLCPP_INFO(this->get_logger(), "Left Hand    Right Hand");
        RCLCPP_INFO(this->get_logger(), "   W             I    ");
        RCLCPP_INFO(this->get_logger(), " A S D         J K L  ");
        RCLCPP_INFO(this->get_logger(), "---------------------------");
        RCLCPP_INFO(this->get_logger(), "W/S: Up/Down, A/D: Yaw Left/Right");
        RCLCPP_INFO(this->get_logger(), "I/K: Forward/Backward, J/L: Left/Right");
        RCLCPP_INFO(this->get_logger(), "Hold a button to move, release to stop.");
        RCLCPP_INFO(this->get_logger(), "---------------------------");
    }

    TelloKeyNode::~TelloKeyNode()
    {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }
  
    int TelloKeyNode::kbhit()
    {
        struct termios oldt, newt;
        int ch;
        int oldf;

        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

        ch = getchar();

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);

        if(ch != EOF)
        {
            ungetc(ch, stdin);
            return 1;
        }

        return 0;
    }
    
    char TelloKeyNode::getKey()
    {
        char ch = 0;
        if (kbhit()) {
            ch = getchar();
        }
        return ch;
    }

    void TelloKeyNode::readKeyboardInput()
    {
        while (rclcpp::ok()) {
        char key = getKey();
        
        // 转换为小写
        key = tolower(key);
        
        // 处理退出命令
        if (key == KEY_QUIT) {
            RCLCPP_INFO(this->get_logger(), "Quit command received, shutting down...");
            rclcpp::shutdown();
            return;
        }
        
        // 处理起飞和降落命令
        if (key == KEY_TAKEOFF) {
            auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
            request->cmd = "takeoff";
            tello_client_->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "Sending takeoff command...");
        } else if (key == KEY_LAND) {
            auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
            request->cmd = "land";
            tello_client_->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "Sending land command...");
        } else {
            // 创建速度消息
            geometry_msgs::msg::Twist twist_msg;
            
            // 根据按键设置速度
            switch (key) {
                case KEY_FORWARD:  // 右手 前进
                    twist_msg.linear.x = LINEAR_SPEED;
                    twist_msg.linear.y = 0;
                    twist_msg.linear.z = 0;
                    twist_msg.angular.z = 0;
                    break;
                case KEY_BACKWARD: // 右手 后退
                    twist_msg.linear.x = -LINEAR_SPEED;
                    twist_msg.linear.y = 0;
                    twist_msg.linear.z = 0;
                    twist_msg.angular.z = 0;
                    break;
                case KEY_STRAFE_LEFT: // 右手 左移
                    twist_msg.linear.x = 0;
                    twist_msg.linear.y = LINEAR_SPEED;
                    twist_msg.linear.z = 0;
                    twist_msg.angular.z = 0;
                    break;
                case KEY_STRAFE_RIGHT: // 右手 右移
                    twist_msg.linear.x = 0;
                    twist_msg.linear.y = -LINEAR_SPEED;
                    twist_msg.linear.z = 0;
                    twist_msg.angular.z = 0;
                    break;
                case KEY_THROTTLE_UP: // 左手 上升
                    twist_msg.linear.x = 0;
                    twist_msg.linear.y = 0;
                    twist_msg.linear.z = LINEAR_SPEED;
                    twist_msg.angular.z = 0;
                    break;
                case KEY_THROTTLE_DOWN: // 左手 下降
                    twist_msg.linear.x = 0;
                    twist_msg.linear.y = 0;
                    twist_msg.linear.z = -LINEAR_SPEED;
                    twist_msg.angular.z = 0;
                    break;
                case KEY_YAW_LEFT: // 左手 左转
                    twist_msg.linear.x = 0;
                    twist_msg.linear.y = 0;
                    twist_msg.linear.z = 0;
                    twist_msg.angular.z = ANGULAR_SPEED;
                    break;
                case KEY_YAW_RIGHT: // 左手 右转
                    twist_msg.linear.x = 0;
                    twist_msg.linear.y = 0;
                    twist_msg.linear.z = 0;
                    twist_msg.angular.z = -ANGULAR_SPEED;
                    break;
                default: // 未定义键：速度信息清零
                    twist_msg.linear.x = 0;
                    twist_msg.linear.y = 0;
                    twist_msg.linear.z = 0;
                    twist_msg.angular.z = 0;
            }
            
            // 发布速度指令
            cmd_vel_pub_->publish(twist_msg);
        }
        
        usleep(10000); // 10ms 延迟以减少CPU使用
        }
    }

} // namespace tello_key

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tello_key::TelloKeyNode)