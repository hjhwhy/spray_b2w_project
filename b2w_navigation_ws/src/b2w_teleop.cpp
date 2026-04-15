#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <unitree/robot/b2/sport/sport_client.hpp>
using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;

class B2TeleopNode : public rclcpp::Node
{
public:
    B2TeleopNode(): Node("b2w_teleop_node") {
        sport_client_.SetTimeout(25.0f);
        sport_client_.Init();
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10,
            std::bind(&B2TeleopNode::joy_callback, this, std::placeholders::_1));
    }
    ~B2TeleopNode(){}

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
            //case 0x0004: joy_msg->axes[1] =  speed; break; // 前进
            //case 0x0005: joy_msg->axes[1] = -speed; break; // 后退
            //case 0x0006: joy_msg->axes[0] = -speed; break; // 左移
            //case 0x0007: joy_msg->axes[0] =  speed; break; // 右移
            //case 0x0008: joy_msg->axes[2] = -speed; break; // 左转
            //case 0x0009: joy_msg->axes[2] =  speed; break; // 右转
        float linear_x = 0.0f, linear_y = 0.0f, linear_vyaw = 0.0f;
        if (msg->axes.size() > 1) {
            linear_x = msg->axes[1];  // 前后：左摇杆 Y 轴
            linear_y = msg->axes[0];
            linear_vyaw = msg->axes[2];
        }
        int res = 1;
        if (std::abs(linear_x) > 0.1f || std::abs(linear_y) > 0.1f || std::abs(linear_vyaw) > 0.1f) {
            res = sport_client_.Move(linear_x, linear_y, linear_vyaw);
        } else {
            res = sport_client_.StopMove();
        }
        if (res < 0) {
            RCLCPP_WARN(this->get_logger(), "SportClient command failed, code: %d", res);
        }
    }

    std::string interface_name_;
    unitree::robot::b2::SportClient sport_client_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("b2_teleop"), "Usage: %s <networkInterface>", argv[0]);
        return -1;
    }
    ChannelFactory::Instance()->Init(0, argv[1]);
    auto node = std::make_shared<B2TeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}