#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tello_msgs/srv/tello_action.hpp"
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

namespace tello_key
{

// 键盘控制参数
constexpr double LINEAR_SPEED = 1;  // 前进/后退/上升/下降速度
constexpr double ANGULAR_SPEED = 1; // 旋转速度

// 键盘按键映射 (采用左手WASD和右手IJKL控制)
// 左手控制: WASD -> 垂直运动和偏航
// 右手控制: IJKL -> 前后左右平移
constexpr char KEY_THROTTLE_UP = 'w';       // 左手 上升
constexpr char KEY_THROTTLE_DOWN = 's';     // 左手 下降
constexpr char KEY_YAW_LEFT = 'a';          // 左手 左转
constexpr char KEY_YAW_RIGHT = 'd';         // 左手 右转
constexpr char KEY_FORWARD = 'i';           // 右手 前进
constexpr char KEY_BACKWARD = 'k';          // 右手 后退
constexpr char KEY_STRAFE_LEFT = 'j';       // 右手 左移
constexpr char KEY_STRAFE_RIGHT = 'l';      // 右手 右移
constexpr char KEY_TAKEOFF = 't';           // 起飞
constexpr char KEY_LAND = 'b';              // 降落
constexpr char KEY_QUIT = 'q';              // 退出

    class TelloKeyNode : public rclcpp::Node
    {
    public:
        explicit TelloKeyNode(const rclcpp::NodeOptions &options);

        ~TelloKeyNode();

    private:
        // 发布速度指令
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        
        // 调用无人机动作服务
        rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr tello_client_;
        
        // 键盘输入读取线程
        std::thread input_thread_;
        
        // 读取键盘输入的函数
        void readKeyboardInput();
        
        // 检查是否有键盘输入的函数
        int kbhit();
        
        // 获取按键输入
        char getKey();
    };

} // namespace tello_key