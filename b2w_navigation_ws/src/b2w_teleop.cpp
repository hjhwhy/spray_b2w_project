#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <unitree/robot/b2/sport/sport_client.hpp>
#include <algorithm>
#include <cmath>
using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;

class B2TeleopNode : public rclcpp::Node
{
public:
    B2TeleopNode(): Node("b2w_teleop_node") {
        this->declare_parameter("teleop_linear_x_speed", 0.5);
        this->declare_parameter("teleop_linear_y_speed", 0.5);
        this->declare_parameter("teleop_yaw_speed", 0.5);
        this->declare_parameter("teleop_deadzone", 0.1);

        this->get_parameter("teleop_linear_x_speed", teleop_linear_x_speed_);
        this->get_parameter("teleop_linear_y_speed", teleop_linear_y_speed_);
        this->get_parameter("teleop_yaw_speed", teleop_yaw_speed_);
        this->get_parameter("teleop_deadzone", teleop_deadzone_);

        teleop_linear_x_speed_ = std::max(0.0, teleop_linear_x_speed_);
        teleop_linear_y_speed_ = std::max(0.0, teleop_linear_y_speed_);
        teleop_yaw_speed_ = std::max(0.0, teleop_yaw_speed_);
        teleop_deadzone_ = std::clamp(teleop_deadzone_, 0.0, 1.0);

        sport_client_.SetTimeout(25.0f);
        sport_client_.Init();
        RCLCPP_INFO(
            this->get_logger(),
            "Teleop speed config: linear_x=%.3f m/s, linear_y=%.3f m/s, yaw=%.3f rad/s, deadzone=%.3f",
            teleop_linear_x_speed_, teleop_linear_y_speed_, teleop_yaw_speed_, teleop_deadzone_);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10,
            std::bind(&B2TeleopNode::joy_callback, this, std::placeholders::_1));
    }
    ~B2TeleopNode(){}

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        float linear_x = 0.0f, linear_y = 0.0f, linear_vyaw = 0.0f;
        if (msg->axes.size() > 2) {
            linear_x = static_cast<float>(std::clamp(static_cast<double>(msg->axes[1]), -1.0, 1.0) * teleop_linear_x_speed_);
            linear_y = static_cast<float>(std::clamp(static_cast<double>(msg->axes[0]), -1.0, 1.0) * teleop_linear_y_speed_);
            linear_vyaw = static_cast<float>(std::clamp(static_cast<double>(msg->axes[2]), -1.0, 1.0) * teleop_yaw_speed_);
        }
        int res = 1;
        if (std::abs(linear_x) > teleop_deadzone_ ||
            std::abs(linear_y) > teleop_deadzone_ ||
            std::abs(linear_vyaw) > teleop_deadzone_) {
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
    double teleop_linear_x_speed_;
    double teleop_linear_y_speed_;
    double teleop_yaw_speed_;
    double teleop_deadzone_;
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
