//### 业务逻辑代码 -- 定位导航部分 ###
#include <iostream>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp> 
#include <std_srvs/srv/set_bool.hpp> 
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "z1_arm_controller_cpp/srv/move_arm.hpp"  
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/b2/sport/sport_client.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;
using namespace std::chrono_literals;

#define TOPIC_LOWSTATE "rt/lowstate"
#define DT_IMU 0.004f                   // IMU周期：4ms (250Hz)

class B2WNavigationController : public rclcpp::Node
{
public:
    struct Waypoint {
        std::string id;
        double x;
        double y;
        double z;
    };

    enum State {
        WAITING_FOR_WAYPOINT,
        ALIGNING_YAW,
        MOVING_TO_TARGET,
        REPOSITIONING_BACKWARD,
        WAITING_FOR_FRESH_RTK,
        AVOID_TURNING,
        AVOID_MOVING,
        EXECUTING_ARM_TASK,
        RETRYING_ARM_AFTER_FORWARD,
        RETRYING_ARM_AFTER_BACKUP,
        TRIGGERING_RELAY,
        RESETTING_ARM,
        GET_NEXT_WAYPOINT,
        FINISH_ALL_POINTS
    };

    explicit B2WNavigationController()
        : Node("b2w_navigation_controller"),
          moving_(false), state_(WAITING_FOR_WAYPOINT),
          last_rtk_update_time_(this->now())
    {
        // 定义参数并设置默认值（可在 YAML 中覆盖）
        this->declare_parameter("moving_to_target_forward_speed", 0.8);
        this->declare_parameter("heading_alignment_threshold", 0.2); // 弧度，约10.14度
        this->declare_parameter("z1_arm_end_height", 0.0);
        this->declare_parameter("arrive_distance", 0.8);
        this->declare_parameter("min_distance_for_arm_task", 0.6);
        this->declare_parameter("reposition_back_distance", 0.35);
        this->declare_parameter("avoid_distance", 0.8);
        this->declare_parameter<double>("obstacle_detection_range", 1.2); 
        this->declare_parameter("arm_offset_x", 0.3487);
        this->declare_parameter("rtk_hz", 10); //hz
        this->declare_parameter("distance_to_slow_down", 2.5);
        this->declare_parameter("min_useful_vyaw", 0.4);
        this->declare_parameter("max_vyaw", 0.6);
        this->declare_parameter("waypoint_file_path", std::string("gnss_waypoints.txt"));
        
        this->get_parameter("heading_alignment_threshold", heading_alignment_threshold_);
        this->get_parameter("moving_to_target_forward_speed", moving_to_target_forward_speed_);
        this->get_parameter("z1_arm_end_height", z1_arm_end_height_);
        this->get_parameter("arrive_distance", arrive_distance_);
        this->get_parameter("min_distance_for_arm_task", min_distance_for_arm_task_);
        this->get_parameter("reposition_back_distance", reposition_back_distance_);
        this->get_parameter("avoid_distance", avoid_dist_);
        this->get_parameter("obstacle_detection_range", obs_range_);
        this->get_parameter("arm_offset_x", arm_offset_x_);
        this->get_parameter("rtk_hz", rtk_hz_);
        this->get_parameter("distance_to_slow_down", distance_to_slow_down_);
        this->get_parameter("min_useful_vyaw", min_useful_vyaw_);
        this->get_parameter("max_vyaw", max_vyaw_);
        this->get_parameter("waypoint_file_path", waypoint_file_path_);

        if (!LoadWaypointsFromFile(waypoint_file_path_)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load waypoints. Check waypoint_file_path: %s", waypoint_file_path_.c_str());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded parameters:");
        RCLCPP_INFO(this->get_logger(), "  heading_alignment_threshold: %.3f rad", heading_alignment_threshold_);
        RCLCPP_INFO(this->get_logger(), " moving_to_target_forward_speed: %.3f m/s", moving_to_target_forward_speed_);
        RCLCPP_INFO(this->get_logger(), " z1_arm_end_height: %.3f m", z1_arm_end_height_);
        RCLCPP_INFO(this->get_logger(), " arrive_distance: %.3f m", arrive_distance_);
        RCLCPP_INFO(this->get_logger(), " min_distance_for_arm_task: %.3f m", min_distance_for_arm_task_);
        RCLCPP_INFO(this->get_logger(), " reposition_back_distance: %.3f m", reposition_back_distance_);
        RCLCPP_INFO(this->get_logger(), " avoid_distance: %.3f m", avoid_dist_);
        RCLCPP_INFO(this->get_logger(), " arm_offset_x: %.4f m", arm_offset_x_);
        RCLCPP_INFO(this->get_logger(), "rtk_hz: %d", rtk_hz_);
        RCLCPP_INFO(this->get_logger(), "distance_to_slow_down: %.3f", distance_to_slow_down_);
        RCLCPP_INFO(this->get_logger(), "min_useful_vyaw: %.3f", min_useful_vyaw_);
        RCLCPP_INFO(this->get_logger(), "max_vyaw: %.3f", max_vyaw_);
        RCLCPP_INFO(this->get_logger(), "waypoint_file_path: %s", waypoint_file_path_.c_str());

        current_vx_ = 0.0;
        current_vy_ = 0.0;
        last_time_ = this->now(); // 记录第一次调用的时间
        // 发布融合后的IMU消息（可选）
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        battery_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/battery", 10);
        motor_temp_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/motors_temperatures", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/b2w_odom", 50);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/b2w_path", 10);
        acquired_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/acquired_points", 10);
        unacquired_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/unacquired_points", 10);
        path_msg_.header.frame_id = "map";
        odom_rtk_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/epsg_position", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                 this->RtkOdomCallback(msg);
        });
        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&B2WNavigationController::scan_callback, this, std::placeholders::_1));

        // Channel Subscriber for B2W IMU
        lowstate_subscriber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
        lowstate_subscriber_->InitChannel(std::bind(&B2WNavigationController::channel_callback, 
            this, std::placeholders::_1), 1);
        z1_move_to_target_client_ = this->create_client<z1_arm_controller_cpp::srv::MoveArm>("/z1_move_to_target");
        trigger_relay_client_ = this->create_client<std_srvs::srv::Trigger>("/trigger_valve_ch1");
        z1_reset_arm_client_ = this->create_client<z1_arm_controller_cpp::srv::MoveArm>("/z1_reset_arm");
        emergency_stop_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/emergency_stop",
            std::bind(&B2WNavigationController::HandleEmergencyStop, this, std::placeholders::_1, std::placeholders::_2));
        erase_emergency_stop_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/erase_emergency_stop",
            std::bind(&B2WNavigationController::HandleEraseEmergencyStop, this, std::placeholders::_1, std::placeholders::_2));

        sport_client_.SetTimeout(25.0f);
        sport_client_.Init();
        // 控制主循环（50ms）, 提高方向的纠偏频率
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&B2WNavigationController::ControlLoop, this));
        // IMU处理定时器：4ms
        imu_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(4),
            std::bind(&B2WNavigationController::ImuUpdate, this));
        RCLCPP_INFO(this->get_logger(), "B2W Navigation Controller started with Yaw fusion.");
    }

    ~B2WNavigationController() override
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down: stopping robot and cleaning up...");
        // 停止定时器，防止析构期间回调继续触发
        if (control_timer_) control_timer_->cancel();
        if (imu_timer_) imu_timer_->cancel();
        // 发送停止命令给机器人
        try {
            sport_client_.Move(0, 0, 0);
            sport_client_.StandDown();
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Exception during sport_client cleanup.");
        }
        RCLCPP_INFO(this->get_logger(), "Cleanup complete.");
    }

private:
    sensor_msgs::msg::LaserScan last_scan_;
    bool has_scan_data_ = false;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_ = *msg;
        has_scan_data_ = true;
    }
    // 检查前方是否有障碍物
    bool is_front_obstacle() {
        if (!has_scan_data_) return false;
        int size = last_scan_.ranges.size();
        if (size == 0) return false;

        double angle_step = last_scan_.angle_increment;
        int center_idx = size / 2;
        // 扫描前方 -50度 到 +50度
        int span = (int)((50.0 * M_PI / 180.0) / angle_step); // 50度转弧度
        int start_idx = std::max(0, center_idx - span);
        int end_idx = std::min(size - 1, center_idx + span);

        for (int i = start_idx; i <= end_idx; ++i) {
            float dist = last_scan_.ranges[i];
            if (dist > last_scan_.range_min && dist < last_scan_.range_max) {
                if (dist < obs_range_) { 
                    return true;
                }
            }
        }
        return false;
    }
    // 寻找空旷方向 (返回建议的角速度)
    double find_clear_direction() {
        return -0.5; // 默认始终右转，避免在左右空间接近时来回切换
    }

    // 计算角度误差 [修正版，处理 PI 跳变]
    double calculate_heading_error(double target, double current) {
        double error = target - current;
        while (error > M_PI) error -= 2.0 * M_PI;
        while (error < -M_PI) error += 2.0 * M_PI;
        return error;
    }

    void ImuUpdate()
    {
        // Keep IMU timer focused on IMU-related work only.
    }

    void channel_callback(const void* msg_raw)
    {
        const unitree_go::msg::dds_::LowState_* msg = 
                static_cast<const unitree_go::msg::dds_::LowState_*>(msg_raw);
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds(); // 计算时间间隔，单位：秒
        // 检查时间间隔，防止 dt 过小或为负（例如节点刚启动或时钟异常）
        if (dt <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid time interval: %f seconds. Skipping integration.", dt);
            return;
        }

        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";
        imu_msg.angular_velocity.x = msg->imu_state().gyroscope()[0];
        imu_msg.angular_velocity.y = msg->imu_state().gyroscope()[1];
        imu_msg.angular_velocity.z = msg->imu_state().gyroscope()[2];;

        imu_msg.linear_acceleration.x = msg->imu_state().accelerometer()[0];
        imu_msg.linear_acceleration.y = msg->imu_state().accelerometer()[1];
        double ax = msg->imu_state().accelerometer()[0];
        double ay = msg->imu_state().accelerometer()[1];
        double az = msg->imu_state().accelerometer()[2];
        Eigen::Vector3d gyro(
            msg->imu_state().gyroscope()[0],
            msg->imu_state().gyroscope()[1],
            msg->imu_state().gyroscope()[2]
        );
        Eigen::Vector3d acc(
            msg->imu_state().accelerometer()[0],
            msg->imu_state().accelerometer()[1],
            msg->imu_state().accelerometer()[2]
        );

        // 使用简单的欧拉积分方法更新速度   v_new = v_old + a * dt
        current_vx_ += ax * dt;
        current_vy_ += ay * dt;
        last_time_ = current_time;
        imu_publisher_->publish(imu_msg);
        // ---------------- 新增：电池状态解析 ----------------
        const auto& bms = msg->bms_state();
        uint8_t battery_soc = bms.soc();     // 电池电量（1~100）
        uint8_t battery_status = bms.status();  // 电池状态（SAFE, CHG, DCHG 等）
        /*RCLCPP_INFO_THROTTLE( this->get_logger(),  *this->get_clock(), 
            3000, "Battery: %u%%, Status=%u", battery_soc, battery_status
        );*/
        sensor_msgs::msg::BatteryState battery_msg;
        battery_msg.header.stamp = this->now();
        battery_msg.percentage = battery_soc / 100.0;   // BatteryState 使用 0~1 范围
        battery_msg.voltage = 0.0;
        battery_msg.design_capacity = 1.0;  
        battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        battery_publisher_->publish(battery_msg);
        // ---------------- 新增：获取电机温度 ----------------
        const auto& motors = msg->motor_state();  // motor_state() 是一个数组（通常长度为 4 或 12 或 18）
        size_t motor_count = motors.size();  // 自动获取电机数量
        std::vector<int8_t> motor_temperatures;
        motor_temperatures.reserve(motor_count);
        for (size_t i = 0; i < motor_count; i++) {
            int8_t temp = motors[i].temperature();  // 电机温度（-100 ~ 150°C）
            motor_temperatures.push_back(temp);
        }

        std_msgs::msg::Float32MultiArray temp_msg;
        for (auto t : motor_temperatures) {
            temp_msg.data.push_back(static_cast<float>(t));
        }
        motor_temp_publisher_->publish(temp_msg);

    }
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw; 
    }
    void RtkOdomCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // ===== 1. RTK 在 base_link 中的外参 =====
        constexpr double RTK_X = -0.4477;  
        constexpr double RTK_Z =  0.3762;
        // ===== 2. 姿态快照（避免回调竞争）=====
        double rtk_x = msg->pose.position.x;
        double rtk_y = msg->pose.position.y;
        current_z_ = msg->pose.position.z;
        double yaw_ros = get_yaw_from_quaternion(msg->pose.orientation);
        current_yaw_ = yaw_ros;
        double effective_offset = std::abs(RTK_X) ;
        double base_x = rtk_x + effective_offset * std::cos(current_yaw_);
        double base_y = rtk_y + effective_offset * std::sin(current_yaw_);
        
        current_x_ = base_x;
        current_y_ = base_y;
        ++rtk_update_seq_;
        last_rtk_update_time_ = this->now();
        PublishOdom();
    }

    double NormalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    bool HasValidPose() const
    {
        return std::isfinite(current_x_) &&
               std::isfinite(current_y_) &&
               std::isfinite(current_yaw_);
    }

    const char *StateToString(State state) const
    {
        switch (state) {
            case WAITING_FOR_WAYPOINT: return "WAITING_FOR_WAYPOINT";
            case ALIGNING_YAW: return "ALIGNING_YAW";
            case MOVING_TO_TARGET: return "MOVING_TO_TARGET";
            case REPOSITIONING_BACKWARD: return "REPOSITIONING_BACKWARD";
            case WAITING_FOR_FRESH_RTK: return "WAITING_FOR_FRESH_RTK";
            case AVOID_TURNING: return "AVOID_TURNING";
            case AVOID_MOVING: return "AVOID_MOVING";
            case EXECUTING_ARM_TASK: return "EXECUTING_ARM_TASK";
            case RETRYING_ARM_AFTER_FORWARD: return "RETRYING_ARM_AFTER_FORWARD";
            case RETRYING_ARM_AFTER_BACKUP: return "RETRYING_ARM_AFTER_BACKUP";
            case TRIGGERING_RELAY: return "TRIGGERING_RELAY";
            case RESETTING_ARM: return "RESETTING_ARM";
            case GET_NEXT_WAYPOINT: return "GET_NEXT_WAYPOINT";
            case FINISH_ALL_POINTS: return "FINISH_ALL_POINTS";
            default: return "UNKNOWN";
        }
    }

    void HandleEmergencyStop(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (paused_) {
            response->success = true;
            response->message = std::string("Task already paused at state: ") + StateToString(state_);
            return;
        }

        paused_ = true;
        pause_stop_latched_ = false;
        moving_ = false;
        try {
            sport_client_.Move(0, 0, 0);
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Exception while stopping robot during emergency stop.");
        }

        response->success = true;
        response->message = std::string("Task paused at state: ") + StateToString(state_);
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    }

    void HandleEraseEmergencyStop(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!paused_) {
            response->success = true;
            response->message = std::string("Task is not paused. Current state: ") + StateToString(state_);
            return;
        }

        paused_ = false;
        pause_stop_latched_ = false;
        response->success = true;
        response->message = std::string("Task resumed from state: ") + StateToString(state_);
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void ControlLoop()
    {

        if (paused_) {
            if (!pause_stop_latched_) {
                try {
                    sport_client_.Move(0, 0, 0);
                } catch (...) {
                    RCLCPP_WARN(this->get_logger(), "Exception while holding robot stop in paused state.");
                }
                moving_ = false;
                pause_stop_latched_ = true;
            }
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Emergency pause active. Controller frozen at state: %s",
                StateToString(state_));
            return;
        }
        pause_stop_latched_ = false;

        if (!HasValidPose()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for valid RTK pose before executing navigation state machine.");
            try {
                sport_client_.Move(0, 0, 0);
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Exception while holding robot stop before valid RTK pose.");
            }
            moving_ = false;
            return;
        }

        constexpr double kAngularKp = 1.2;        
        const double dx_to_target = target_x_ - current_x_;
        const double dy_to_target = target_y_ - current_y_;
        const double distance_to_target = std::hypot(dx_to_target, dy_to_target);
        const double current_yaw = current_yaw_;
        const double target_heading = std::atan2(dy_to_target, dx_to_target);
        const double heading_error = calculate_heading_error(target_heading, current_yaw);

        switch (state_)
        {
        case WAITING_FOR_WAYPOINT:
        {
            if (current_waypoint_index_ >= waypoints_.size()) {
                RCLCPP_INFO(this->get_logger(), "No more waypoints from local file.");
                sport_client_.Move(0, 0, 0);
                state_ = FINISH_ALL_POINTS;
                break;
            }

            const auto &next = waypoints_[current_waypoint_index_++];
            target_x_ = next.x;
            target_y_ = next.y;
            RCLCPP_INFO(this->get_logger(), "Received next waypoint: (%.2f, %.2f)", target_x_, target_y_);
            state_ = ALIGNING_YAW;
            RCLCPP_INFO(this->get_logger(), "Waypoint received. Aligning yaw before moving to target.");
            moving_ = false;
            break;
        }

        case ALIGNING_YAW:
        {
            if (std::abs(heading_error) <= heading_alignment_threshold_) {
                RCLCPP_INFO(this->get_logger(), "Initial yaw alignment complete. Error: %.2f rad", heading_error);
                sport_client_.Move(0, 0, 0);
                state_ = MOVING_TO_TARGET;  
                return;
            }

            double yaw_cmd = std::clamp(heading_error * kAngularKp, -max_vyaw_, max_vyaw_);
            if (std::abs(yaw_cmd) > 1e-6 && std::abs(yaw_cmd) < min_useful_vyaw_) {
                yaw_cmd = std::copysign(min_useful_vyaw_, yaw_cmd);
            }
            int32_t ret = sport_client_.Move(0.0, 0.0, yaw_cmd);
            if (ret != 0) {
                RCLCPP_ERROR(this->get_logger(), "Rotate command failed in ALIGNING_YAW: %d", ret);
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "Aligning initial yaw: error=%.3f rad (%.1f°), vyaw=%.3f", heading_error, heading_error * 180.0 / M_PI, yaw_cmd);
            break;
        }
        case AVOID_TURNING: {
            if (!is_front_obstacle()) {
                RCLCPP_INFO(this->get_logger(), "Direction clear. Starting forward avoidance.");
                avoid_start_x_ = current_x_;
                avoid_start_y_ = current_y_;
                state_ = AVOID_MOVING;
            } else {
                double yaw_cmd = find_clear_direction();
                sport_client_.Move(0.0, 0.0, yaw_cmd);
            }
            break;
        }
        case AVOID_MOVING: {                
            // 计算相对于绕障起点的直线距离
            double avoid_dx = current_x_ - avoid_start_x_;
            double avoid_dy = current_y_ - avoid_start_y_;
            double traveled_dist = std::hypot(avoid_dx, avoid_dy);

            if (traveled_dist >= avoid_dist_) {
                RCLCPP_INFO(this->get_logger(), "Avoidance complete (Traveled %.2fm). Resuming navigation.", traveled_dist);
                state_ = ALIGNING_YAW;
            } else {
                double v_cmd = moving_to_target_forward_speed_ * 0.6;
                sport_client_.Move(v_cmd, 0.0, 0.0);
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                        "Avoiding: Traveled %.2f / %.2f m", traveled_dist, avoid_dist_);
            }
            break;
        }
        case MOVING_TO_TARGET:
        {
            if (is_front_obstacle()) {
                RCLCPP_WARN(this->get_logger(), "Obstacle detected! Switching to Avoidance.");
                sport_client_.Move(0.0, 0.0, 0.0);
                state_ = AVOID_TURNING;
                break; 
            }
            if (distance_to_target <= arrive_distance_) {
                if (distance_to_target < min_distance_for_arm_task_) {
                    RCLCPP_WARN(this->get_logger(),
                        "Too close to waypoint for arm task (dist=%.4f m < %.4f m). Backing up to reposition.",
                        distance_to_target, min_distance_for_arm_task_);
                    reposition_start_x_ = current_x_;
                    reposition_start_y_ = current_y_;
                    sport_client_.Move(0, 0, 0);
                    state_ = REPOSITIONING_BACKWARD;
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "=========================================");
                RCLCPP_INFO(this->get_logger(), "✅ WAYPOINT REACHED");
                RCLCPP_INFO(this->get_logger(), "Target Pos (Local): (%.4f, %.4f)", target_x_, target_y_);
                RCLCPP_INFO(this->get_logger(), "Actual Pos (Local): (%.4f, %.4f)", current_x_, current_y_);
                RCLCPP_INFO(this->get_logger(), "Actual Yaw:         %.2f°", current_yaw_ * 180.0 / M_PI);
                RCLCPP_INFO(this->get_logger(), "Distance Error:     %.4f m", distance_to_target);
                RCLCPP_INFO(this->get_logger(), "=========================================");
                arm_wait_rtk_seq_ = rtk_update_seq_;
                state_ = WAITING_FOR_FRESH_RTK;
                sport_client_.Move(0, 0, 0); 
                return; 
            }
            double v_cmd = 0.0, yaw_cmd = 0.0 , desired_w = 0.0;
            if (distance_to_target > distance_to_slow_down_) {
                v_cmd = moving_to_target_forward_speed_;
            } else {
                v_cmd = moving_to_target_forward_speed_ * (distance_to_target / distance_to_slow_down_);
                if (v_cmd < 0.1) v_cmd = 0.1; // 保持最小蠕动速度
            }
            // 角度控制优化 (加入抗噪死区)
            double abs_err = std::abs(heading_error);
            double err_deg = abs_err * 180.0 / M_PI;
            // 假设噪声水平为 0.025m , 死区角度 (弧度) = 噪声 / 距离
            // 限制最小死区为 1 度，最大死区为 10 度 (防止极近距离失效或过大)
            double noise_level = 0.025; 
            double dynamic_deadband_rad = noise_level / std::max(distance_to_target, 0.1); 
            dynamic_deadband_rad *= 1.2;
            double dynamic_deadband_deg = dynamic_deadband_rad * 180.0 / M_PI;
            if (dynamic_deadband_deg > 10.0) dynamic_deadband_deg = 10.0;
            if (dynamic_deadband_deg < 1.5) dynamic_deadband_deg = 1.5;
            // --- 死区判断 ---
            if (err_deg < dynamic_deadband_deg) {
                // 误差在噪声范围内，视为 0，不转向
                desired_w = 0.0;
            } else {
            // 误差超出噪声范围，执行转向
                double current_kp = 1.2 * 2.5; 
                desired_w = heading_error * current_kp;
                // 动态限幅 (距离越近，最大角速度越小)
                double max_w_allowed = max_vyaw_;
                if (distance_to_target < 1.2) max_w_allowed = max_vyaw_ ;
                if (distance_to_target < 0.5) max_w_allowed = min_useful_vyaw_;
                yaw_cmd = std::clamp(desired_w, -max_w_allowed, max_w_allowed);
            }

            int32_t ret = sport_client_.Move(v_cmd, 0.0, yaw_cmd);
            if (ret != 0 && !moving_) {
                RCLCPP_ERROR(this->get_logger(), "Move command failed with code: %d", ret);
            }
            moving_ = (ret == 0);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "Moving: Target=(%.2f,%.2f) Dist=%.2f YawErr=%.2f vcmd=%.2f vyaw=%.2f", target_x_, target_y_, distance_to_target, heading_error, v_cmd, yaw_cmd);
            break;
        }

        case REPOSITIONING_BACKWARD:
        {
            const double dx = current_x_ - reposition_start_x_;
            const double dy = current_y_ - reposition_start_y_;
            const double moved = std::hypot(dx, dy);
            if (moved >= reposition_back_distance_) {
                sport_client_.Move(0, 0, 0);
                RCLCPP_INFO(this->get_logger(),
                    "Reposition back complete (%.3f m). Realigning toward waypoint.", moved);
                state_ = ALIGNING_YAW;
                break;
            }
            sport_client_.Move(-0.25, 0.0, 0.0);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Repositioning backward: %.3f / %.3f m", moved, reposition_back_distance_);
            break;
        }

        case WAITING_FOR_FRESH_RTK:
        {
            sport_client_.Move(0, 0, 0);
            if (rtk_update_seq_ <= arm_wait_rtk_seq_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "Waypoint reached. Waiting for one fresh RTK frame before arm task...");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Fresh RTK frame received. Start arm task.");
            state_ = EXECUTING_ARM_TASK;
            return;
        }

        case EXECUTING_ARM_TASK:
        {
            sport_client_.Move(0, 0, 0);
            // === 检查 RTK 数据是否足够新（例如 < 400ms）===
            rclcpp::Time now = this->now();
            double rtk_age = (now - last_rtk_update_time_).seconds();
            // 400ms，略小于 RTK 周期（500ms， 2hz）
            // 90ms, rtk周期(100ms, 10hz)
            const double MAX_RTK_AGE_FOR_ARM = (1000/rtk_hz_) - 10; 
            if (rtk_age > MAX_RTK_AGE_FOR_ARM) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "RTK data too old (%.3f s), waiting for fresh update before executing arm task.", rtk_age);
                return;
            }
            if (!z1_move_to_target_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(), "Service /z1_move_to_target not available.");
                return;
            }
            // 第一次进入：关闭避障 + 发起机械臂动作
            if (!arm_task_requested_) {
                // === 计算机械臂底座 z1_base 在世界坐标系的位置 ===
                double robot_yaw = current_yaw_;
                const double arm_offset_x = arm_offset_x_; 
                const double arm_offset_z = 0.05;   // unused for 2D, but noted
                double z1_world_x = current_x_ + arm_offset_x * std::cos(robot_yaw);
                double z1_world_y = current_y_ + arm_offset_x * std::sin(robot_yaw);
                // === 计算目标点相对于 z1_base 的局部坐标 ===
                double dx_world = target_x_ - z1_world_x;
                double dy_world = target_y_ - z1_world_y;
                // 旋转到 z1_base 坐标系（使用 target_heading）
                double cos_h = std::cos(robot_yaw);
                double sin_h = std::sin(robot_yaw);
                double dx_local =  dx_world * cos_h + dy_world * sin_h;
                double dy_local = -dx_world * sin_h + dy_world * cos_h;
                last_dx_local_ = dx_local;
                // 设定末端目标高度（可根据任务调整）
                const double target_z = z1_arm_end_height_;
                // 构造目标位姿（相对于 z1_base）
                auto request = std::make_shared<z1_arm_controller_cpp::srv::MoveArm::Request>();
                request->target_pose.position.x = dx_local;
                request->target_pose.position.y = dy_local;
                request->target_pose.position.z = target_z;
                // 可选：设置末端朝向（例如让夹爪垂直向下）
                // 使用四元数表示：绕 Y 轴旋转 90 度
                tf2::Quaternion q;
                //q.setRPY(0, 83.0 * M_PI / 180.0, 0); // 经测试，机械臂末端平面朝下的角度为83度
                q.setRPY(0, 90.0 * M_PI / 180.0, 0); 
                request->target_pose.orientation = tf2::toMsg(q);
                /*request->target_pose.orientation.w = 0.9004;
                request->target_pose.orientation.x = 0.0;
                request->target_pose.orientation.y = 0.435;
                request->target_pose.orientation.z = 0.0;*/
                // 发起请求并保存 future
                auto arm_task_future = z1_move_to_target_client_->async_send_request(request);
                arm_task_future_ = arm_task_future.future.share();
                arm_task_requested_ = true;
                RCLCPP_INFO(this->get_logger(), "Sent request to move arm.");
                return; // 等待下次循环检查结果
            }
            // 已发起请求，检查是否完成
            if (arm_task_future_.valid() &&
                arm_task_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
            {
                auto result = arm_task_future_.get();
                arm_task_requested_ = false; // 重置标记
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "/z1_move_to_target succeeded.");
                    state_ = TRIGGERING_RELAY;
                } else {
                    RCLCPP_INFO(this->get_logger(), "/z1_move_to_target failed: %s", result->message.c_str());
                    if (last_dx_local_ > 0.6) {
                       RCLCPP_INFO(this->get_logger(), "Arm target too far (dx_local=%.3f > 0.6). Forward and retrying.", last_dx_local_);
                        state_ = RETRYING_ARM_AFTER_FORWARD;
                    } else if (last_dx_local_ < 0.1){
                        RCLCPP_INFO(this->get_logger(), "Arm target too close (dx_local=%.3f > 0.6). Backing up and retrying.", last_dx_local_);
                        state_ = RETRYING_ARM_AFTER_BACKUP;
                    } 
                    else {
                        RCLCPP_INFO(this->get_logger(), "Arm target within range but failed. Going to WAITING_FOR_WAYPOINT.");
                        state_ = WAITING_FOR_WAYPOINT;
                    }
                }
                return;
            }
            // 还未完成，继续等待
            RCLCPP_DEBUG(this->get_logger(), "Waiting for arm task to complete...");
            break;
        }

        case RETRYING_ARM_AFTER_FORWARD: {
            RCLCPP_INFO(this->get_logger(), "Forward before retrying arm task...");
            sport_client_.Move(1, 0.0, 0.0);
            sport_client_.Move(0, 0, 0); 
            arm_task_requested_ = false;
            state_ = EXECUTING_ARM_TASK;
            break;
        }
        case RETRYING_ARM_AFTER_BACKUP: {
            RCLCPP_INFO(this->get_logger(), "Backup before retrying arm task...");
            sport_client_.Move(-1, 0.0, 0.0);
            sport_client_.Move(0, 0, 0); 
            arm_task_requested_ = false;
            state_ = EXECUTING_ARM_TASK;
            break;
        }

        case TRIGGERING_RELAY:
        {
            sport_client_.Move(0, 0, 0);
            if (!trigger_relay_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(), "Service /trigger_valve_ch1 not available.");
                return;
            }
            // 第一次进入：发起电磁阀动作
            if (!ch1_trigger_task_requested_) {
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto ch1_trigger_task_future = trigger_relay_client_->async_send_request(request);
                ch1_trigger_task_future_ = ch1_trigger_task_future.future.share();
                ch1_trigger_task_requested_ = true;
                RCLCPP_INFO(this->get_logger(), "Sent request to /trigger_valve_ch1.");
                return; // 等待下次循环检查结果
            }

            if (ch1_trigger_task_future_.valid() && 
                ch1_trigger_task_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) 
            {
                auto result = ch1_trigger_task_future_.get();
                ch1_trigger_task_requested_ = false;
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "/trigger_valve_ch1 succeeded. 电磁阀打开");
                    state_ = RESETTING_ARM;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "/trigger_valve_ch1 failed: %s", result->message.c_str());
                    state_ = WAITING_FOR_WAYPOINT;
                }
            }
            break;
        }

        case RESETTING_ARM:
        {
            sport_client_.Move(0, 0, 0);
            if (!z1_reset_arm_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(), "Service /z1_reset_arm not available.");
                return;
            }
            // 第一次进入：发起机械臂复位动作
            if (!arm_reset_task_requested_) {
                auto request = std::make_shared<z1_arm_controller_cpp::srv::MoveArm::Request>();
                auto z1_reset_task_future = z1_reset_arm_client_->async_send_request(request);
                arm_reset_task_future_ = z1_reset_task_future.future.share();
                arm_reset_task_requested_ = true;
                RCLCPP_INFO(this->get_logger(), "Sent request to reset arm.");
                return; // 等待下次循环检查结果
            }

            if (arm_reset_task_future_.valid() && 
                arm_reset_task_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) 
            {
                auto result = arm_reset_task_future_.get();
                arm_reset_task_requested_ = false;
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "/z1_reset_arm succeeded.");
                    state_ = GET_NEXT_WAYPOINT;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "/z1_reset_arm failed: %s", result->message.c_str());
                    state_ = WAITING_FOR_WAYPOINT;
                }
            }
            break;
        }

        case GET_NEXT_WAYPOINT:
        {
            sport_client_.Move(0, 0, 0);
            RCLCPP_INFO(this->get_logger(), "Ready to fetch next waypoint...");
            publishWaypointClouds();
            state_ = WAITING_FOR_WAYPOINT;   
            break;
        }

        case FINISH_ALL_POINTS:
        {
            sport_client_.Move(0, 0, 0);
            return; 
        }

        }
    }


private:
    bool ParseWaypointsFile(const std::string& file_path, std::vector<Waypoint>& output)
    {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            return false;
        }

        std::string line;
        int line_num = 0;
        while (std::getline(file, line)) {
            ++line_num;
            if (line.empty() || line[0] == '#') {
                continue;
            }

            std::stringstream ss(line);
            std::string id;
            std::string x_str;
            std::string y_str;
            std::string z_str;
            if (!(std::getline(ss, id, ',') &&
                  std::getline(ss, x_str, ',') &&
                  std::getline(ss, y_str, ',') &&
                  std::getline(ss, z_str, ','))) {
                RCLCPP_WARN(this->get_logger(), "Invalid waypoint line %d: %s", line_num, line.c_str());
                continue;
            }

            try {
                Waypoint p;
                p.id = id;
                p.x = std::stod(x_str);
                p.y = std::stod(y_str);
                p.z = std::stod(z_str);
                output.push_back(p);
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse waypoint line %d: %s", line_num, line.c_str());
            }
        }
        return !output.empty();
    }

    bool LoadWaypointsFromFile(const std::string& configured_path)
    {
        std::vector<std::string> candidates;
        candidates.push_back(configured_path);
        if (std::filesystem::path(configured_path).is_relative()) {
            candidates.push_back((std::filesystem::current_path() / configured_path).string());
        }

        std::vector<Waypoint> parsed;
        std::string used_path;
        for (const auto& candidate : candidates) {
            parsed.clear();
            if (ParseWaypointsFile(candidate, parsed)) {
                used_path = candidate;
                break;
            }
        }

        if (parsed.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Unable to load waypoints from path: %s", configured_path.c_str());
            return false;
        }

        waypoints_ = parsed;
        current_waypoint_index_ = 0;
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s in file order", waypoints_.size(), used_path.c_str());
        return true;
    }

private:
    State state_;

    // --- 成员变量 ---
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_rtk_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_temp_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<ChannelSubscriber<unitree_go::msg::dds_::LowState_>> lowstate_subscriber_;
    nav_msgs::msg::Path path_msg_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr acquired_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unacquired_pub_;

    std::string waypoint_file_path_;
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_ = 0;

    rclcpp::Client<z1_arm_controller_cpp::srv::MoveArm>::SharedPtr z1_move_to_target_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_relay_client_;
    rclcpp::Client<z1_arm_controller_cpp::srv::MoveArm>::SharedPtr z1_reset_arm_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr erase_emergency_stop_service_;
    // 用于保存机械臂动作的 future 和是否已发起请求
    rclcpp::Client<z1_arm_controller_cpp::srv::MoveArm>::SharedFuture arm_task_future_;
    bool arm_task_requested_ = false;
    rclcpp::Client<z1_arm_controller_cpp::srv::MoveArm>::SharedFuture arm_reset_task_future_;
    bool arm_reset_task_requested_ = false;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture ch1_trigger_task_future_;
    bool ch1_trigger_task_requested_ = false;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;

    SportClient sport_client_;

    // 位置信息
    double current_x_ = NAN, current_y_ = NAN, current_z_ = NAN, avoid_start_x_ = NAN, avoid_start_y_ = NAN;
    double reposition_start_x_ = NAN, reposition_start_y_ = NAN;
    double target_x_ = 0.0, target_y_ = 0.0;
    double current_vx_ = 0.0, current_vy_ = 0.0;
    double last_dx_local_ = 0.0;
    bool moving_;
    bool paused_ = false;
    bool pause_stop_latched_ = false;
    double last_path_x_ = 0.0, last_path_y_ = 0.0;
    double current_yaw_ = NAN; 

    // 声明可配置参数（作为类成员）
    double heading_alignment_threshold_, moving_to_target_forward_speed_;
    double z1_arm_end_height_;
    double arrive_distance_;
    double min_distance_for_arm_task_;
    double reposition_back_distance_;
    double avoid_dist_, obs_range_;
    double arm_offset_x_;
    int rtk_hz_;
    double distance_to_slow_down_;
    double min_useful_vyaw_, max_vyaw_;

    // 存储上一次回调的时间，用于计算时间间隔
    rclcpp::Time last_time_;
    rclcpp::Time last_rtk_update_time_;
    uint64_t rtk_update_seq_ = 0;
    uint64_t arm_wait_rtk_seq_ = 0;

    sensor_msgs::msg::PointCloud2 makeWaypointCloud(size_t from, size_t to)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = "map";
        cloud.header.stamp = this->now();
        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(to - from);
        cloud.is_dense = true;
        cloud.is_bigendian = false;
        sensor_msgs::PointCloud2Modifier mod(cloud);
        mod.setPointCloud2Fields(3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT64,
            "y", 1, sensor_msgs::msg::PointField::FLOAT64,
            "z", 1, sensor_msgs::msg::PointField::FLOAT64);
        mod.resize(to - from);
        sensor_msgs::PointCloud2Iterator<double> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
        for (size_t i = from; i < to; ++i, ++ix, ++iy, ++iz) {
            *ix = waypoints_[i].x;
            *iy = waypoints_[i].y;
            *iz = waypoints_[i].z;
        }
        return cloud;
    }

    void publishWaypointClouds()
    {
        acquired_pub_->publish(makeWaypointCloud(0, current_waypoint_index_));
        unacquired_pub_->publish(makeWaypointCloud(current_waypoint_index_, waypoints_.size()));
    }

    void PublishOdom()
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = "map";      
        odom.child_frame_id = "base_link";  
        odom.pose.pose.position.x = current_x_;
        odom.pose.pose.position.y = current_y_;
        odom.pose.pose.position.z = current_z_;

        // 速度（来自 IMU 积分 vx, vy）
        odom.twist.twist.linear.x = current_vx_;
        odom.twist.twist.linear.y = current_vy_;
        odom_pub_->publish(odom);

        float dx = last_path_x_ - current_x_;
        float dy = last_path_y_ - current_y_;
        float distance = std::sqrt(dx * dx + dy * dy);
        if (distance >= 0.05) { // 5 cm 发布一个
            last_path_x_ = current_x_;
            last_path_y_ = current_y_;
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = odom.header.stamp;
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose = odom.pose.pose;
            path_msg_.header.stamp = odom.header.stamp; 
            path_msg_.poses.push_back(pose_stamped);
            path_pub_->publish(path_msg_);
            //如果路径太长，你可以定期清理 path_msg_.poses，避免内存无限增长
            if (path_msg_.poses.size() > 1000) {
                path_msg_.poses.erase(path_msg_.poses.begin(), path_msg_.poses.begin() + 100);
            }
        }
    }

};

int main(int argc, char *argv[])
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        return -1;
    }

    rclcpp::init(argc, argv);
    ChannelFactory::Instance()->Init(0, argv[1]);

    auto node = std::make_shared<B2WNavigationController>();
    rclcpp::spin(node);
    // 显式释放节点，触发析构函数中的 sport_client_ 清理
    node.reset();
    rclcpp::shutdown();
    return 0;
}
