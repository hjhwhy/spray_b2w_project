// src/z1_arm_controller_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>  
#include <tf2_ros/transform_listener.h>        
#include <tf2_ros/buffer.h>                    
#include "z1_arm_controller_cpp/srv/move_arm.hpp"
#include "z1_arm_controller_cpp/srv/move_arm_with_rpy.hpp"
#include "unitree_arm_sdk/control/unitreeArm.h"

#include <thread>
#include <chrono>

using namespace UNITREE_ARM;
using MoveArm = z1_arm_controller_cpp::srv::MoveArm;
using MoveArmWithRPY = z1_arm_controller_cpp::srv::MoveArmWithRPY;

class Z1ArmController : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_pub_;
    rclcpp::Service<MoveArm>::SharedPtr move_service_;
    rclcpp::Service<MoveArm>::SharedPtr reset_service_; 
    rclcpp::Service<MoveArmWithRPY>::SharedPtr move_in_base_service_; // 新增：基于 base_link 的移动服务
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;            //TF 缓冲区
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_; 

    UNITREE_ARM::unitreeArm arm_;

    double max_speed_;
    double gripper_pos = 0.0;

public:
    Z1ArmController()
        : Node("z1_arm_controller"),
          arm_(true),
          max_speed_(1.0)
    {
        this->declare_parameter("max_speed", 1.0);
        this->get_parameter("max_speed", max_speed_);
        //初始化 TF
        tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        // 创建移动服务（只执行动作）
        move_service_ = this->create_service<MoveArm>(
            "/z1_move_to_target",
            std::bind(&Z1ArmController::handleMoveRequest, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        reset_service_ = this->create_service<MoveArm>(
            "/z1_reset_arm",
            std::bind(&Z1ArmController::handleResetRequest, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        complete_pub_ = this->create_publisher<std_msgs::msg::Bool>("/z1_complete", 10);

        //新增服务：接收 base_link 下的目标，转到 z1_base
        move_in_base_service_ = this->create_service<MoveArmWithRPY>(
            "/z1_move_in_base_frame",
            std::bind(&Z1ArmController::handleMoveInBaseRequest, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        arm_.sendRecvThread->start();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(this->get_logger(), "机械臂已就绪，等待服务请求...");
    }

    ~Z1ArmController() {
        RCLCPP_INFO(this->get_logger(), "关闭机械臂控制...");
        arm_.setFsm(ArmFSMState::PASSIVE);
        arm_.sendRecvThread->shutdown();
    }

private:

    void handleMoveInBaseRequest(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<MoveArmWithRPY::Request> request,
        std::shared_ptr<MoveArmWithRPY::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "收到 base_link 坐标系下的移动请求...");
        // 解析输入
        double x_in = request->position.x;
        double y_in = request->position.y;
        double z_in = request->position.z;
        double roll_in = request->rpy[0];
        double pitch_in = request->rpy[1];
        double yaw_in = request->rpy[2];

        RCLCPP_INFO(this->get_logger(),
        "目标 (base_link): pos(%.3f, %.3f, %.3f), rpy(%.3f, %.3f, %.3f)",
        x_in, y_in, z_in, roll_in, pitch_in, yaw_in);

        // 1. 构造 PoseStamped（base_link）
        geometry_msgs::msg::PoseStamped pose_in_base;
        pose_in_base.header.frame_id = "base_link";
        pose_in_base.header.stamp = this->now();

        pose_in_base.pose.position.x = x_in;
        pose_in_base.pose.position.y = y_in;
        pose_in_base.pose.position.z = z_in;
        // 2. 将 RPY 转为四元数
        tf2::Quaternion quat;
        quat.setRPY(roll_in, pitch_in, yaw_in); // 输入为弧度
        pose_in_base.pose.orientation = tf2::toMsg(quat);

        // 3. 转换到 z1_base
        geometry_msgs::msg::PoseStamped pose_in_z1;
        try {
            tf2_buffer_->transform(pose_in_base, pose_in_z1, "z1_base", tf2::durationFromSec(1.0));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF 变换失败: %s", ex.what());
            response->success = false;
            response->message = std::string("TF transform failed: ") + ex.what();
            return;
        }

    // 4. 从变换后的 pose 提取 position 和 orientation
        double x = pose_in_z1.pose.position.x;
        double y = pose_in_z1.pose.position.y;
        double z = pose_in_z1.pose.position.z;

        tf2::Quaternion quat_out;
        tf2::fromMsg(pose_in_z1.pose.orientation, quat_out);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat_out).getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(),
                    "执行移动（来自 base_link）: x=%.3f, y=%.3f, z=%.3f | r=%.3f, p=%.3f, y=%.3f",
                    x, y, z, roll, pitch, yaw);
        try {
            Vec6 target_pose;
            target_pose << roll, pitch, yaw, x, y, z;
            arm_.labelRun("forward");
            bool move_success = arm_.MoveJ(target_pose, gripper_pos, max_speed_);
            if (!move_success) {
                RCLCPP_ERROR(this->get_logger(), "MoveJ failed: target posture is not reachable (IK failed).");
                response->success = false;
                response->message = "Target unreachable (inverse kinematics failed)";
                return;
            }
            RCLCPP_INFO(this->get_logger(), "机械臂已到达目标位置");
            response->success = true;
            response->message = "Movement completed from base_link frame";
        } catch (const std::exception &e) {

            RCLCPP_ERROR(this->get_logger(), "执行移动时发生异常: %s", e.what());
            response->success = false;
            response->message = std::string("Exception during movement: ") + e.what();
        }
    }
    void handleMoveRequest(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<MoveArm::Request> request,
        std::shared_ptr<MoveArm::Response> response)
    {
        const auto& msg = request->target_pose;
        RCLCPP_INFO(this->get_logger(), "收到移动请求，处理目标位姿...");
        double x = msg.position.x;
        double y = msg.position.y;
        double z = msg.position.z;
        tf2::Quaternion quat;
        try {
            tf2::fromMsg(msg.orientation, quat);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "四元数转换失败: %s", ex.what());
            response->success = false;
            response->message = "Invalid orientation";
            return;
        }
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(),
                    "执行移动: x=%.3f, y=%.3f, z=%.3f | r=%.3f, p=%.3f, y=%.3f",
                    x, y, z, roll, pitch, yaw);
        try {
            Vec6 target_pose;
            target_pose << roll, pitch, yaw, x, y, z;
            arm_.labelRun("forward");
            bool move_success = arm_.MoveJ(target_pose, gripper_pos, max_speed_);
            if (!move_success) {
                RCLCPP_ERROR(this->get_logger(), "MoveJ failed: target posture is not reachable (IK failed).");
                response->success = false;
                response->message = "Target unreachable (inverse kinematics failed)";
                return;
            }
            RCLCPP_INFO(this->get_logger(), "机械臂已到达目标位置");
            // 发布完成信号
            auto complete_msg = std::make_unique<std_msgs::msg::Bool>();
            complete_msg->data = true;
            complete_pub_->publish(std::move(complete_msg));
            response->success = true;
            response->message = "Movement completed (no reset)";
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "执行移动时发生异常: %s", e.what());
            auto err_msg = std::make_unique<std_msgs::msg::Bool>();
            err_msg->data = false;
            complete_pub_->publish(std::move(err_msg));
            response->success = false;
            response->message = std::string("Exception during movement: ") + e.what();
        }
    }

    // 新增：处理复位请求的回调函数
    void handleResetRequest(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<MoveArm::Request> request,
        std::shared_ptr<MoveArm::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "收到复位请求，正在复位机械臂...");

        try {
            // 可以加个延时，等待用户准备（可选）
            RCLCPP_INFO(this->get_logger(), "等待 2 秒后复位...");
            std::this_thread::sleep_for(std::chrono::seconds(2));

            arm_.backToStart(); // 回到开机位置
            RCLCPP_INFO(this->get_logger(), "机械臂已成功复位");
            // 发布复位完成信号
            auto complete_msg = std::make_unique<std_msgs::msg::Bool>();
            complete_msg->data = true;
            complete_pub_->publish(std::move(complete_msg));
            response->success = true;
            response->message = "Arm reset successfully";
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "复位时发生异常: %s", e.what());
            auto err_msg = std::make_unique<std_msgs::msg::Bool>();
            err_msg->data = false;
            complete_pub_->publish(std::move(err_msg));
            response->success = false;
            response->message = std::string("Exception during reset: ") + e.what();
        }
    }



};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    try {
        RCLCPP_INFO(rclcpp::get_logger("z1_arm_controller"), "启动 Z1 机械臂控制器...");
        auto controller = std::make_shared<Z1ArmController>();
        rclcpp::spin(controller);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("z1_arm_controller"), "异常: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }
    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("z1_arm_controller"), "控制器已退出");
    return 0;
}