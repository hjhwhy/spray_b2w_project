//### 业务逻辑代码 -- 定位导航部分 ###
// 12月3日 --- ekf
#include <iostream>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_srvs/srv/trigger.hpp> 
#include <std_srvs/srv/set_bool.hpp> 
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "z1_arm_controller_cpp/srv/move_arm.hpp"  
#include "spray_path_planner/srv/get_next_waypoint.hpp"     
#include "spray_path_planner/srv/set_start_point.hpp"

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/b2/sport/sport_client.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Dense>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;
using namespace std::chrono_literals;

#define TOPIC_LOWSTATE "rt/lowstate"
#define DT_IMU 0.004f                   // IMU周期：4ms (250Hz)

struct EKF {
    Eigen::Vector3d x;   // yaw, bias, scale
    Eigen::Matrix3d P;   // covariance
    Eigen::Matrix3d Q;   // process noise
    Eigen::Matrix<double,1,1> R; // measurement noise
};

class B2WNavigationController : public rclcpp::Node
{
public:
    explicit B2WNavigationController()
        : Node("b2w_navigation_controller"),
          current_target_valid_(false), moving_(false), state_(WAITING_FOR_WAYPOINT),
          calibration_start_time_(0.0),
          is_calibrated_(false), bias_z_(0.0), yaw_(0.0),
          prev_rtk_x_(0.0), prev_rtk_y_(0.0), has_prev_rtk_(false),
          last_rtk_update_time_(this->now()),
          pid_integral_(0.0), pid_prev_error_(0.0),
          last_control_time_(this->now())
    {
        // 定义参数并设置默认值（可在 YAML 中覆盖）
        this->declare_parameter("min_move_distance_for_yaw", 1.5);
        this->declare_parameter("initial_move_speed", 0.5);
        this->declare_parameter("min_speed_for_cog", 0.3);
        this->declare_parameter("min_distance_for_cog", 0.3);
        this->declare_parameter("moving_to_target_forward_speed", 0.6);
        this->declare_parameter("heading_alignment_threshold", 0.1); // 弧度，约5.7度
        this->declare_parameter("z1_arm_end_height", 0.0);
        this->declare_parameter("arrive_distance", 0.8);
        this->declare_parameter("calibration_duration", 5.0);
        this->declare_parameter("arm_offset_x", 0.3487);
        this->declare_parameter("imu_k_correction_error", 0.01);
        this->declare_parameter("imu_gyro_z_threshold", 0.1); //  rad/s
        this->declare_parameter<bool>("enable_correctYawBias", true);
        this->declare_parameter("rtk_hz", 2); 
        this->declare_parameter("distance_to_slow_down", 2.5);
      
        this->get_parameter("min_move_distance_for_yaw", min_move_distance_for_yaw_);
        this->get_parameter("initial_move_speed", initial_move_speed_);
        this->get_parameter("min_speed_for_cog", min_speed_for_cog_);
        this->get_parameter("min_distance_for_cog", min_distance_for_cog_);
        this->get_parameter("heading_alignment_threshold", heading_alignment_threshold_);
        this->get_parameter("moving_to_target_forward_speed", moving_to_target_forward_speed_);
        this->get_parameter("z1_arm_end_height", z1_arm_end_height_);
        this->get_parameter("arrive_distance", arrive_distance_);
        this->get_parameter("calibration_duration", calibration_duration_);
        this->get_parameter("arm_offset_x", arm_offset_x_);
        this->get_parameter("imu_k_correction_error", imu_k_correction_error_);
        this->get_parameter("imu_gyro_z_threshold", imu_gyro_z_threshold_);
        this->get_parameter("rtk_hz", rtk_hz_);
        this->get_parameter("distance_to_slow_down", distance_to_slow_down_);
        enable_correctYawBias_ = this->get_parameter("enable_correctYawBias").as_bool();
        RCLCPP_INFO(this->get_logger(), "Loaded parameters:");
        RCLCPP_INFO(this->get_logger(), "  min_move_distance_for_yaw: %.3f m", min_move_distance_for_yaw_);
        RCLCPP_INFO(this->get_logger(), "  initial_move_speed: %.3f m/s", initial_move_speed_);
        RCLCPP_INFO(this->get_logger(), "  min_speed_for_cog: %.3f m/s", min_speed_for_cog_);
        RCLCPP_INFO(this->get_logger(), " min_distance_for_cog: %.3f m", min_distance_for_cog_);
        RCLCPP_INFO(this->get_logger(), "  heading_alignment_threshold: %.3f rad", heading_alignment_threshold_);
        RCLCPP_INFO(this->get_logger(), " moving_to_target_forward_speed: %.3f m/s", moving_to_target_forward_speed_);
        RCLCPP_INFO(this->get_logger(), " z1_arm_end_height: %.3f m", z1_arm_end_height_);
        RCLCPP_INFO(this->get_logger(), " arrive_distance: %.3f m", arrive_distance_);
        RCLCPP_INFO(this->get_logger(), " calibration_duration: %.3f s", calibration_duration_);
        RCLCPP_INFO(this->get_logger(), " arm_offset_x: %.4f m", arm_offset_x_);
        RCLCPP_INFO(this->get_logger(), " imu_k_correction_error: %.3f", imu_k_correction_error_);
        RCLCPP_INFO(this->get_logger(), "imu_gyro_z_threshold: %.3f", imu_gyro_z_threshold_);
        RCLCPP_INFO(this->get_logger(), "rtk_hz: %d", rtk_hz_);
        RCLCPP_INFO(this->get_logger(), " enable_correctYawBias is : %s", enable_correctYawBias_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "distance_to_slow_down: %.3f", distance_to_slow_down_);
        ekf_.x << 0.0, 0.0, 1.0; // yaw, bias_z, scale = 1
        ekf_.P = Eigen::Matrix3d::Identity() * 0.1;
        ekf_.Q = (Eigen::Matrix3d() << 
                    0.001, 0,      0,
                    0,     0.0001, 0,
                    0,     0,      0.0001).finished();
        ekf_.R << 0.05;   // GNSS heading noise

        // New: Create log directory 
        std::string log_dir = "/home/" + std::string(std::getenv("USER")) + "/b2w_nav_log";
        std::filesystem::create_directories(log_dir);
        auto t = std::time(nullptr);
        std::stringstream ts;
        ts << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
        b2w_pos_z1_pose_file_.open(log_dir + "/b2w_pos_z1_pos" + ts.str() + ".csv", std::ios::app);
        b2w_pos_z1_pose_file_ << std::fixed << std::setprecision(3);  
        b2w_pos_z1_pose_file_ << "timestamp,target_x,target_y,current_x,current_y,z1_local_x,z1_local_y\n";
        RCLCPP_INFO(this->get_logger(), "Data logs saved to: %s", log_dir.c_str());

        current_vx_ = 0.0;
        current_vy_ = 0.0;
        last_time_ = this->now(); // 记录第一次调用的时间
        // 发布融合后的IMU消息（可选）
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/b2w_odom", 50);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/b2w_path", 10);
        path_msg_.header.frame_id = "map";
        odom_rtk_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/gnss_odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->RtkOdomCallback(msg);
            });
        // Channel Subscriber for B2W IMU
        lowstate_subscriber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
        lowstate_subscriber_->InitChannel(std::bind(&B2WNavigationController::channel_callback, 
            this, std::placeholders::_1), 1);
        
        //对坐标点进行排序
        set_start_point_client_ = this->create_client<spray_path_planner::srv::SetStartPoint>("/set_start_point");

        get_next_waypoint_client_ = this->create_client<spray_path_planner::srv::GetNextWaypoint>("/get_next_waypoint");
        z1_move_to_target_client_ = this->create_client<z1_arm_controller_cpp::srv::MoveArm>("/z1_move_to_target");
        trigger_relay_client_ = this->create_client<std_srvs::srv::Trigger>("/trigger_valve_ch1");
        z1_reset_arm_client_ = this->create_client<z1_arm_controller_cpp::srv::MoveArm>("/z1_reset_arm");
        // 避障客户端，前进时开启避障功能，停止时关闭，
        open_or_close_lidar_client_ = this->create_client<std_srvs::srv::SetBool>("/enable_avoidance");
        // 急停服务
        emergency_stop_service_ = this->create_service<std_srvs::srv::Trigger>("/emergency_stop", [this](
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            this->EmergencyStopCallback(request, response);
        });
        erase_emergency_stop_service_ = this->create_service<std_srvs::srv::Trigger>("/erase_emergency_stop", 
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            this->ResumeNavigationCallback(request, response);
        });

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

private:
    void EkfPredict(double gyro_z_raw, double dt)
    {
        double yaw = ekf_.x(0);
        double bias = ekf_.x(1);
        double gyro_corrected = gyro_z_raw - bias;
        // ---- 状态预测 ----
        ekf_.x(0) = NormalizeAngle(yaw + gyro_corrected * dt);
        // bias & scale 不变
        // ---- 雅可比矩阵 ----
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0,1) = -dt;
        // ---- 协方差预测 ----
        ekf_.P = F * ekf_.P * F.transpose() + ekf_.Q;

        yaw_ = ekf_.x(0);  // 替代原 yaw_
        bias_z_ = ekf_.x(1);
    }
    void EkfUpdateYawByCOG(double yaw_cog)
    {
        double yaw_pred = ekf_.x(0);
        Eigen::Matrix<double,1,3> H;
        H << 1, 0, 0;

        double y = NormalizeAngle(yaw_cog - yaw_pred);

        Eigen::Matrix<double,1,1> S = H * ekf_.P * H.transpose() + ekf_.R;
        Eigen::Matrix<double,3,1> K = ekf_.P * H.transpose() * S.inverse();

        ekf_.x = ekf_.x + K * y;

        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        ekf_.P = (I - K * H) * ekf_.P;
        // ---- 同步回全局变量 ----
        yaw_ = ekf_.x(0);
        bias_z_ = ekf_.x(1);
    }

    void ImuUpdate()
    {
        if (!is_calibrated_) {
            this->CalibrateImu();
            return;
        }
        EkfPredict(latest_gyro_z_, DT_IMU);
        PublishOdom();
    }

    void channel_callback(const void* msg_raw)
    {
        const unitree_go::msg::dds_::LowState_* msg = static_cast<const unitree_go::msg::dds_::LowState_*>(msg_raw);
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds(); // 计算时间间隔，单位：秒
        // 检查时间间隔，防止 dt 过小或为负（例如节点刚启动或时钟异常）
        if (dt <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid time interval: %f seconds. Skipping integration.", dt);
            return;
        }
        latest_gyro_z_ = msg->imu_state().gyroscope()[2]; // Z轴角速度 rad/s

        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";
        imu_msg.angular_velocity.x = msg->imu_state().gyroscope()[0];
        imu_msg.angular_velocity.y = msg->imu_state().gyroscope()[1];
        imu_msg.angular_velocity.z = latest_gyro_z_;

        imu_msg.linear_acceleration.x = msg->imu_state().accelerometer()[0];
        imu_msg.linear_acceleration.y = msg->imu_state().accelerometer()[1];
        double ax = msg->imu_state().accelerometer()[0];
        double ay = msg->imu_state().accelerometer()[1];
        // 使用简单的欧拉积分方法更新速度   v_new = v_old + a * dt
        current_vx_ += ax * dt;
        current_vy_ += ay * dt;
        last_time_ = current_time;
        imu_publisher_->publish(imu_msg);
    }

    void RtkOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
        // 创建请求--传入起点来排序路径--spray_path_planner
        if (!set_start_point_) {
            auto request = std::make_shared<spray_path_planner::srv::SetStartPoint::Request>();
            RCLCPP_ERROR(this->get_logger(), "调用 /set_start_point 服务");
            request->start.x = current_x_;
            request->start.y = current_y_;
            request->start.z =  msg->pose.pose.position.z;
            // 发送异步请求
            auto result_future = set_start_point_client_->async_send_request(
                request,
                [this](rclcpp::Client<spray_path_planner::srv::SetStartPoint>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "成功设置起始点: %s", response->message.c_str());
                        set_start_point_ = true;
                    } else {
                        RCLCPP_WARN(this->get_logger(), "设置起始点失败: %s", response->message.c_str());
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "调用 /set_start_point 服务时发生异常: %s", e.what());
                }
            });
        }
    
        double now_sec = this->now().seconds();
        if (!has_prev_rtk_) {
            prev_rtk_x_ = current_x_;
            prev_rtk_y_ = current_y_;
            prev_rtk_used_x_ = current_x_;
            prev_rtk_used_y_ = current_y_;
            has_prev_rtk_ = true;
            return;
        }

        double delta_north = current_x_ - prev_rtk_x_;  // Δx: 北向
        double delta_east  = current_y_ - prev_rtk_y_;  // Δy: 东向
        double dist = std::sqrt(delta_north*delta_north + delta_east*delta_east);
        // ---- 只有直线运动才累计,避免 COG 方向包含曲线段 ----
        if (fabs(latest_gyro_z_) < imu_gyro_z_threshold_) {
            accumulated_dist_ += dist;
        } else {
            accumulated_dist_ = 0.0;
            prev_rtk_used_x_ = current_x_;
            prev_rtk_used_y_ = current_y_;
        }
        cog_valid_ = false;
        if (enable_correctYawBias_ && 
            accumulated_dist_ > min_distance_for_cog_ && fabs(latest_gyro_z_) < imu_gyro_z_threshold_) {
            // 只有稳定直线运动才计算 COG
            RCLCPP_INFO(this->get_logger(), "correction imu bias z, latest gyro z: %.4f", latest_gyro_z_);
            cog_yaw_ = atan2(current_y_ - prev_rtk_used_y_, current_x_ - prev_rtk_used_x_);
            cog_distance_ = accumulated_dist_;
            cog_valid_ = true;
             // 重新开始积累
            prev_rtk_used_x_ = current_x_;
            prev_rtk_used_y_ = current_y_;
            accumulated_dist_ = 0.0;
        }
        prev_rtk_x_ = current_x_;
        prev_rtk_y_ = current_y_;        
        last_rtk_update_time_ = this->now();

    }

    void CalibrateImu()
    {
        rclcpp::Time now = this->now();
        if (calibration_start_time_ == 0.0) {
            calibration_start_time_ = now.seconds();
            gyro_sum_ = 0.0;
            calibration_count_ = 0;
            RCLCPP_INFO(this->get_logger(), "Starting IMU bias calibration... Keep robot still.");
            return;
        }

        if ((now.seconds() - calibration_start_time_) < calibration_duration_) {
            gyro_sum_ += latest_gyro_z_;
            calibration_count_++;
            return;
        }
        // 完成标定
        if (calibration_count_ > 0) {
            bias_z_ = gyro_sum_ / calibration_count_;
        }
        is_calibrated_ = true;
        RCLCPP_INFO(this->get_logger(), "IMU bias calibration done. bias_z = %.6f rad/s", bias_z_);
        // 此时仍静止COG 不可信, 改为进入新状态：主动移动以获取有效COG
        state_ = WAITING_FOR_WAYPOINT;  // <<< 改为先请求 waypoint
        has_started_moving_for_yaw_ = false;
        has_prev_rtk_ = false; // 可选：重置 RTK 记录，确保使用移动中的数据
        RCLCPP_INFO(this->get_logger(), "IMU calibrated. Preparing to move for initial heading estimation...");
    }

    double NormalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    double ComputeTargetHeading()
    {
        double delta_north = target_x_ - current_x_;   // 注意：target_x_ 是“北”
        double delta_east  = target_y_ - current_y_;   // target_y_ 是“东”
        double target_yaw = std::atan2(delta_east, delta_north);

        return target_yaw;
    }

    double ComputePidOutput(double error, double dt)
    {
        const double kp = 0.8, ki = 0.05, kd = 0.3;
        // 积分抗饱和
        pid_integral_ += error * dt;
        if (pid_integral_ > 1.0) pid_integral_ = 1.0;
        if (pid_integral_ < -1.0) pid_integral_ = -1.0;

        double derivative = dt > 1e-6 ? (error - pid_prev_error_) / dt : 0.0;
        double output = kp * error + ki * pid_integral_ + kd * derivative;

        const double max_yaw_rate = 0.75;
        if (output > max_yaw_rate) output = max_yaw_rate;
        if (output < -max_yaw_rate) output = -max_yaw_rate;

        pid_prev_error_ = error;
        return output;
    }

    void ControlLoop()
    {
        // --- 优先级最高的逻辑：IMU 标定期间必须静止 ---
        if (!is_calibrated_) {
            sport_client_.Move(0.0, 0.0, 0.0);  // 保持静止
            return; // 阻止状态机向下执行
        }

        switch (state_)
        {
        case WAITING_FOR_WAYPOINT:
        {
        // 如果还没有发起请求，则发起
        if (!has_pending_request_)
        {
            if (!get_next_waypoint_client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "Service /get_next_waypoint not available, retrying...");
                return;
            }
            auto request = std::make_shared<spray_path_planner::srv::GetNextWaypoint::Request>();
            auto future_and_request_id = get_next_waypoint_client_->async_send_request(request);
            get_next_waypoint_future_ = future_and_request_id.future.share();
            has_pending_request_ = true;
            RCLCPP_INFO(this->get_logger(), "Sent request for next waypoint.");
            return; // 不再继续处理，等下次循环检查结果
        }
        // 已经有请求，检查是否完成
        if (get_next_waypoint_future_.valid() && 
            get_next_waypoint_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
        {
            auto result = get_next_waypoint_future_.get();
            has_pending_request_ = false; // 清除标记
            if (result->success)
            {
                target_x_ = result->waypoint.x;
                target_y_ = result->waypoint.y;
                //current_target_valid_ = true;
                RCLCPP_INFO(this->get_logger(), "Received next waypoint: (%.2f, %.2f)", target_x_, target_y_);
                // 如果刚完成 IMU 校准且尚未有有效 yaw（即第一次导航），先进入初始移动
                if (!has_valid_initial_yaw_) {
                    state_ = MOVING_FOR_INITIAL_YAW;
                    has_started_moving_for_yaw_ = false;
                    has_prev_rtk_ = false;
                    RCLCPP_INFO(this->get_logger(), "First waypoint received. Starting initial yaw estimation move.");
                } else {
                    state_ = ALIGNING_YAW_AFTER_INITIAL_MOVE;
                    RCLCPP_INFO(this->get_logger(), "Waypoint received. Aligning yaw before moving to target.");
                }
                moving_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "No more waypoints: %s", result->message.c_str());
                sport_client_.Move(0, 0, 0); // 停止
                state_ = FINISH_ALL_POINTS;
            }
        }
        break;
        }

        case MOVING_FOR_INITIAL_YAW:
        {
            if (!has_started_moving_for_yaw_) {
            // 第一次进入：记录起始位置并开始移动
                if (std::isnan(current_x_) || std::isnan(current_y_)) {
                    RCLCPP_WARN(this->get_logger(), "Waiting for valid RTK fix before moving for initial yaw...");
                    sport_client_.Move(0.0, 0.0, 0.0);
                    return;
                }
                initial_move_start_x_ = current_x_;
                initial_move_start_y_ = current_y_;
                has_started_moving_for_yaw_ = true;
                RCLCPP_INFO(this->get_logger(), "Started moving forward for initial heading estimation...");
            }
            // 只要还没完成，就要持续发送前进指令
            int32_t ret = sport_client_.Move(initial_move_speed_, 0, 0);
            if (ret != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to start moving for initial yaw: %d", ret);
                yaw_ = 0.0;
                RCLCPP_WARN(this->get_logger(), "Using default yaw=0 due to move failure.");
                state_ = WAITING_FOR_WAYPOINT;
                return;
            }
            // 计算位移
            double delta_north = current_x_ - initial_move_start_x_;  // Δx: 北向
            double delta_east  = current_y_ - initial_move_start_y_;  // Δy: 东向
            double distance_moved = std::sqrt(delta_north*delta_north + delta_east*delta_east);

            if (distance_moved >= min_move_distance_for_yaw_) {
                sport_client_.Move(0, 0, 0);
                //  计算航向角
                yaw_ = std::atan2(delta_east, delta_north);  // atan2(东, 北)
                yaw_ = NormalizeAngle(yaw_);
                RCLCPP_INFO(this->get_logger(), "Initial yaw set from movement: %.2f rad (%.1f deg)", yaw_, yaw_ * 180.0 / M_PI);
                // 保存当前位置作为下一次 COG 校正的起点
                prev_rtk_x_ = current_x_;
                prev_rtk_y_ = current_y_;
                has_prev_rtk_ = true;
                has_valid_initial_yaw_ = true;  // <<< 标记已设置初始 yaw
                
                ekf_.x(0) = yaw_;
                ekf_.P = Eigen::Matrix3d::Identity() * 0.01; 

                // 跳转到新状态：原地对准目标方向
                state_ = ALIGNING_YAW_AFTER_INITIAL_MOVE;  
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Moving for initial yaw... Distance: %.2f m", distance_moved);
            }
            break;
        }

        case ALIGNING_YAW_AFTER_INITIAL_MOVE:
        {
            double target_heading = ComputeTargetHeading();
            double current_yaw = yaw_;
            double heading_error = NormalizeAngle(target_heading - current_yaw);
            if (std::abs(heading_error) <= heading_alignment_threshold_) {
                RCLCPP_INFO(this->get_logger(), "Initial yaw alignment complete. Error: %.2f rad", heading_error);
                sport_client_.Move(0, 0, 0);
                state_ = MOVING_TO_TARGET;  
                return;
            }
            rclcpp::Time now = this->now();
            double dt = (now - last_control_time_).seconds();
            last_control_time_ = now;
            double vyaw = ComputePidOutput(heading_error, dt);
            int32_t ret = sport_client_.Move(0.0, 0.0, vyaw);

            if (ret != 0) {
                RCLCPP_ERROR(this->get_logger(), "Rotate command failed in ALIGNING_YAW_AFTER_INITIAL_MOVE: %d", ret);
            }
            RCLCPP_DEBUG(this->get_logger(), "Aligning initial yaw: error=%.3f, vyaw=%.3f", heading_error, vyaw);
            break;
        }

        case MOVING_TO_TARGET:
        {
            float dx = target_x_ - current_x_;
            float dy = target_y_ - current_y_;
            float distance = std::sqrt(dx * dx + dy * dy);

            if (distance <= arrive_distance_) {
                RCLCPP_INFO(this->get_logger(), "Arrived at target! Distance: %.2f m", distance);
                sport_client_.Move(0, 0, 0); 
                moving_ = false;
                // 计算当前朝向误差
                double target_heading = ComputeTargetHeading();
                double current_yaw = yaw_;
                double heading_error = NormalizeAngle(target_heading - current_yaw);
                // 检查是否还需要调整朝向
                if (std::abs(heading_error) > heading_alignment_threshold_) {
                    RCLCPP_INFO(this->get_logger(), "Position arrived, starting heading alignment. Error: %.2f rad", heading_error);
                    state_ = ALIGNING_TO_TARGET; // 切换到对准状态
                    pid_integral_ = 0.0; // 可选：重置积分项，避免累积误差影响新任务
                    pid_prev_error_ = heading_error;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Arrived and aligned! Heading error: %.2f rad", heading_error);
                    state_ = EXECUTING_ARM_TASK;
                }
                return;
            }
            double target_heading = ComputeTargetHeading();
            double current_yaw = yaw_;  // 融合后的航向
            double heading_error = NormalizeAngle(target_heading - current_yaw);
            // PID输出为角速度 vyaw
            rclcpp::Time now = this->now();
            double dt = (now - last_control_time_).seconds();
            last_control_time_ = now;
            double vyaw = ComputePidOutput(heading_error, dt);
            //接近目标时减速
            float vx = moving_to_target_forward_speed_;
            if (distance < distance_to_slow_down_) {
                vx *= (distance / distance_to_slow_down_); // 线性减速
            }
            float vy = 0.0f;
            int32_t ret = sport_client_.Move(vx, vy, vyaw);
            
            if (ret != 0 && !moving_) {
                RCLCPP_ERROR(this->get_logger(), "Move command failed with code: %d", ret);
            }
            moving_ = (ret == 0);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,  // 节流周期：1000ms = 1秒
                "Moving: Target=(%.2f,%.2f) Dist=%.2f YawErr=%.2f vyaw=%.2f",
                    target_x_, target_y_, distance, heading_error, vyaw);
            break;
        }

        // 新增状态：原地对准目标点方向
        case ALIGNING_TO_TARGET:
        {
            double target_heading = ComputeTargetHeading();
            double current_yaw = yaw_;
            double heading_error = NormalizeAngle(target_heading - current_yaw);
            // 检查是否对准
            if (std::abs(heading_error) <= heading_alignment_threshold_) {
                RCLCPP_INFO(this->get_logger(), "Heading alignment complete. Error: %.2f rad", heading_error);
                sport_client_.Move(0, 0, 0); 
                state_ = EXECUTING_ARM_TASK;
                return;
            }
            // 继续计算PID输出，但只发送角速度指令，vx=0
            rclcpp::Time now = this->now();
            double dt = (now - last_control_time_).seconds();
            last_control_time_ = now;
            double vyaw = ComputePidOutput(heading_error, dt);
            int32_t ret = sport_client_.Move(0.0, 0.0, vyaw); 

            if (ret != 0) {
                RCLCPP_ERROR(this->get_logger(), "Rotate command failed with code: %d", ret);
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Aligning: YawErr=%.6f vyaw=%.2f", heading_error, vyaw);
            break;
        }

        case EXECUTING_ARM_TASK:
        {
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
                // 关闭避障
                RCLCPP_INFO(this->get_logger(), "准备调用 enable_avoidance 服务，关闭");
                auto request_lidar = std::make_shared<std_srvs::srv::SetBool::Request>();
                request_lidar->data = false; 
                auto future_result = open_or_close_lidar_client_->async_send_request(request_lidar, 
                    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                    try {
                        auto response = future.get();
                        if (response->success) {
                            RCLCPP_INFO(this->get_logger(), "SetBool 成功: %s", response->message.c_str());
                        } else {
                            RCLCPP_WARN(this->get_logger(), "SetBool 失败: %s", response->message.c_str());
                        }
                    } catch (const std::exception &e) {
                        RCLCPP_ERROR(this->get_logger(), "调用 SetBool 服务时发生异常: %s", e.what());
                    }
                });
                // === 计算机械臂底座 z1_base 在世界坐标系的位置 ===
                // === 使用 target_heading（而非 yaw_）计算 z1_base 的世界坐标 ===
                double target_heading = ComputeTargetHeading(); // yaw_ + heading_error
                const double arm_offset_x = arm_offset_x_; // from URDF
                const double arm_offset_z = 0.05;   // unused for 2D, but noted
                double z1_world_x = current_x_ + arm_offset_x * std::cos(target_heading);
                double z1_world_y = current_y_ + arm_offset_x * std::sin(target_heading);
                // === 计算目标点相对于 z1_base 的局部坐标 ===
                double dx_world = target_x_ - z1_world_x;
                double dy_world = target_y_ - z1_world_y;
                // 旋转到 z1_base 坐标系（使用 target_heading）
                double cos_h = std::cos(target_heading);
                double sin_h = std::sin(target_heading);
                double dx_local =  dx_world * cos_h + dy_world * sin_h;
                double dy_local = -dx_world * sin_h + dy_world * cos_h;
                last_dx_local_ = dx_local;
                // 写入 log文件
                {
                    if (b2w_pos_z1_pose_file_.is_open()) {
                    // === 恢复世界坐标（逆变换） ===
                    double rx = z1_world_x + dx_local * std::cos(yaw_) - dy_local * std::sin(yaw_);
                    double ry = z1_world_y + dx_local * std::sin(yaw_) + dy_local * std::cos(yaw_);
                    double err = std::hypot(rx - target_x_, ry - target_y_);
                    b2w_pos_z1_pose_file_ << this->now().seconds() << ","
                            << target_x_ << "," << target_y_ << ","
                            << current_x_<< "," << current_y_ << ","
                            << dx_local << ","<< dy_local << "    " 
                            << rx << "," << ry << "," << err <<"\n";
                            b2w_pos_z1_pose_file_.flush();
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Failed to open /tmp/arm_target_log.csv for writing!");
                    }
                }

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
                q.setRPY(0, M_PI/2, 0); // 举例：让末端Z轴朝下（需根据实际URDF调整）
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
            // 还未完成，继续等待
            break;
        }

        case GET_NEXT_WAYPOINT:
        {
            RCLCPP_INFO(this->get_logger(), "Ready to fetch next waypoint...");
            state_ = WAITING_FOR_WAYPOINT;   //触发新一轮请求
            //打开雷达避障
            RCLCPP_INFO(this->get_logger(), "准备调用 enable_avoidance 服务，请求值  打开");
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = true; 
            auto future_result = open_or_close_lidar_client_->async_send_request(
                request,
                [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "SetBool 成功: %s", response->message.c_str());
                    } else {
                        RCLCPP_WARN(this->get_logger(), "SetBool 失败: %s", response->message.c_str());
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "调用 enable_avoidance 服务时发生异常: %s", e.what());
                }
            });
            //
            break;
        }
        case EMERGENCY_STOPPED:
        {
            // 保持静止，等待 erase_emergency_stop 服务调用，从而切换到move to target 状态
            sport_client_.Move(0, 0, 0);
            break;
        }

        }
    }

    void EmergencyStopCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        sport_client_.Move(0, 0, 0);
        state_ = EMERGENCY_STOPPED;
        //current_target_valid_ = false;
        response->success = true;
        response->message = "Emergency stop executed.";
    }

    void ResumeNavigationCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (state_ != EMERGENCY_STOPPED) {
            response->success = false;
            response->message = "Not in emergency stop state.";
            return;
        }
        // 恢复到正常流程：重新开始请求路径点
        state_ = MOVING_TO_TARGET;
        // 可选：重置 PID 或其他状态
        pid_integral_ = 0.0;
        pid_prev_error_ = 0.0;

        response->success = true;
        response->message = "Resumed navigation from emergency stop.";
    }

private:
    enum State {
        WAITING_FOR_WAYPOINT,
        ALIGNING_YAW_AFTER_INITIAL_MOVE,
        MOVING_TO_TARGET,
        ALIGNING_TO_TARGET,
        EXECUTING_ARM_TASK,
        RETRYING_ARM_AFTER_FORWARD,
        RETRYING_ARM_AFTER_BACKUP,
        TRIGGERING_RELAY,
        RESETTING_ARM,
        GET_NEXT_WAYPOINT,
        EMERGENCY_STOPPED,
        MOVING_FOR_INITIAL_YAW,
        FINISH_ALL_POINTS  
    };
    State state_;
    
    EKF ekf_;

     // log file paths 
    std::ofstream b2w_pos_z1_pose_file_;

    // --- 成员变量 ---
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_rtk_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<ChannelSubscriber<unitree_go::msg::dds_::LowState_>> lowstate_subscriber_;
    nav_msgs::msg::Path path_msg_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Client<spray_path_planner::srv::GetNextWaypoint>::SharedPtr get_next_waypoint_client_;
    rclcpp::Client<spray_path_planner::srv::SetStartPoint>::SharedPtr set_start_point_client_;
    bool set_start_point_ = false;
    // 当前获取航点的服务请求句柄
    std::shared_future<spray_path_planner::srv::GetNextWaypoint::Response::SharedPtr> get_next_waypoint_future_;
    bool has_pending_request_ = false; // 标记是否有未完成的请求

    rclcpp::Client<z1_arm_controller_cpp::srv::MoveArm>::SharedPtr z1_move_to_target_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_relay_client_;
    rclcpp::Client<z1_arm_controller_cpp::srv::MoveArm>::SharedPtr z1_reset_arm_client_;
    // 用于保存机械臂动作的 future 和是否已发起请求
    rclcpp::Client<z1_arm_controller_cpp::srv::MoveArm>::SharedFuture arm_task_future_;
    bool arm_task_requested_ = false;
    rclcpp::Client<z1_arm_controller_cpp::srv::MoveArm>::SharedFuture arm_reset_task_future_;
    bool arm_reset_task_requested_ = false;
    //
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture ch1_trigger_task_future_;
    bool ch1_trigger_task_requested_ = false;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr open_or_close_lidar_client_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr erase_emergency_stop_service_;

    //rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr lidar_obstacle_client_;

    rclcpp::Time last_control_time_{0}; // 初始化为 0
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;

    SportClient sport_client_;

    bool has_valid_initial_yaw_ = false; 

    // 位置信息
    float current_x_ = 0.0f, current_y_ = 0.0f, current_z_ = 0.0f;
    float target_x_ = 0.0f, target_y_ = 0.0f;
    float current_vx_ = 0.0f, current_vy_ = 0.0f;
    float last_dx_local_ = 0.0f;
    bool current_target_valid_;
    bool moving_;
    float prev_rtk_used_x_ , prev_rtk_used_y_ ;
    float accumulated_dist_ = 0.0f;

    // IMU 融合相关
    double latest_gyro_z_ = 0.0;
    double bias_z_;
    double yaw_;  // 融合后的航向角（弧度）
    bool is_calibrated_;
    double calibration_start_time_;
    double gyro_sum_;
    int calibration_count_;
    bool cog_valid_ = false;
    double cog_distance_ = 0.0;
    double cog_yaw_ = 0.0;

    // --- 新增用于初始航向移动的变量 ---
    bool has_started_moving_for_yaw_ = false;
    double initial_move_start_x_ = 0.0;
    double initial_move_start_y_ = 0.0;

    // 声明可配置参数（作为类成员）
    double min_move_distance_for_yaw_;
    double initial_move_speed_;
    double min_speed_for_cog_;
    double min_distance_for_cog_;
    double heading_alignment_threshold_; // 新增：航向对准阈值
    double moving_to_target_forward_speed_;
    double z1_arm_end_height_;
    double arrive_distance_;
    double calibration_duration_;
    double arm_offset_x_;
    bool enable_correctYawBias_;
    double imu_k_correction_error_; // 校正增益
    double imu_gyro_z_threshold_;
    int rtk_hz_;
    double distance_to_slow_down_;

    // 存储上一次回调的时间，用于计算时间间隔
    rclcpp::Time last_time_;

    // RTK COG 用于校正
    double prev_rtk_x_, prev_rtk_y_;
    bool has_prev_rtk_;
    rclcpp::Time last_rtk_update_time_;

    // PID参数可在ComputePidOutput中调整
    double pid_integral_;     // 积分项累计值
    double pid_prev_error_;   // 上一次的误差，用于微分项

    void PublishOdom()
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = "map";      
        odom.child_frame_id = "base_link";  

        odom.pose.pose.position.x = current_x_;
        odom.pose.pose.position.y = current_y_;
        odom.pose.pose.position.z = current_z_;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_);
        odom.pose.pose.orientation = tf2::toMsg(q);
        // 速度（来自 IMU 积分 vx, vy）
        odom.twist.twist.linear.x = current_vx_;
        odom.twist.twist.linear.y = current_vy_;
        odom.twist.twist.angular.z = latest_gyro_z_;
        odom_pub_->publish(odom);

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
    rclcpp::shutdown();
    return 0;
}