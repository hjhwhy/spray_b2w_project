#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iomanip>
#include <algorithm>

enum class NavState {
    ALIGNING,
    MOVING,
    STOPPED,
    AVOID_TURNING,   // 阶段1: 原地寻找空旷方向
    AVOID_MOVING     // 阶段2: 强制直线前进绕开
};

struct Waypoint {
    std::string name;
    double utm_x, utm_y, z;
    double local_x, local_y;
};

struct ArrivalRecord {
    std::string wp_name;
    double target_x, target_y;
    double actual_x, actual_y;
    double actual_yaw_deg;
    double dist_error;
    double yaw_error_deg;
};

struct Point2D {
    double x;
    double y;
};

class DogController : public rclcpp::Node {
public:
    DogController() : Node("dog_controller"), origin_set_(false), current_state_(NavState::STOPPED) {
        // 参数声明
        this->declare_parameter<std::string>("waypoint_file", "/home/test/rtk_nav_ws/src/ins_driver/waypoints.txt");
        this->declare_parameter<double>("max_linear_speed", 0.5);
        this->declare_parameter<double>("max_angular_speed", 0.5);
        this->declare_parameter<double>("arrive_distance", 0.1);        
        this->declare_parameter<double>("heading_alignment_threshold_deg", 10.0);
        this->declare_parameter<double>("distance_to_slow_down", 1.8);
        this->declare_parameter<double>("kp_angular", 1.2);
        this->declare_parameter<double>("antenna_offset", 0.0);
        this->declare_parameter<double>("obstacle_detection_range", 1.0); 
        this->declare_parameter<double>("avoid_forward_distance", 1.0); 

        // 获取参数
        std::string wp_file = this->get_parameter("waypoint_file").as_string();
        max_v_ = this->get_parameter("max_linear_speed").as_double();
        max_w_ = this->get_parameter("max_angular_speed").as_double();
        arrive_dist_ = this->get_parameter("arrive_distance").as_double();
        align_thresh_ = this->get_parameter("heading_alignment_threshold_deg").as_double() * M_PI / 180.0;
        slow_down_dist_ = this->get_parameter("distance_to_slow_down").as_double();
        kp_w_ = this->get_parameter("kp_angular").as_double();
        antenna_offset_ = this->get_parameter("antenna_offset").as_double();
        
        obs_range_ = this->get_parameter("obstacle_detection_range").as_double();
        avoid_dist_ = this->get_parameter("avoid_forward_distance").as_double();

        if (!load_waypoints(wp_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints.");
        }

        sub_utm_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "utm_fix", 10, std::bind(&DogController::utm_callback, this, std::placeholders::_1));
        
        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&DogController::scan_callback, this, std::placeholders::_1));
            
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DogController::control_loop, this));
        
        current_wp_index_ = 0;
        has_current_pos_ = false;
        has_scan_data_ = false;
        
        // 绕障辅助变量初始化
        avoid_start_pos_ = {0.0, 0.0};
        avoid_target_yaw_ = 0.0;
    }

private:
    bool load_waypoints(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) return false;
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string name, x_str, y_str, z_str;
            if (std::getline(ss, name, ',') && std::getline(ss, x_str, ',') && 
                std::getline(ss, y_str, ',') && std::getline(ss, z_str, ',')) {
                try {
                    Waypoint wp;
                    wp.name = name; wp.utm_x = std::stod(x_str); wp.utm_y = std::stod(y_str);
                    wp.z = std::stod(z_str); wp.local_x = 0.0; wp.local_y = 0.0;
                    waypoints_.push_back(wp);
                } catch (...) {}
            }
        }
        return !waypoints_.empty();
    }

    void utm_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double utm_x = msg->pose.position.x;
        double utm_y = msg->pose.position.y;
        double yaw_ros = get_yaw_from_quaternion(msg->pose.orientation);
        current_yaw_ = yaw_ros;

        if (!origin_set_) {
            double center_x = utm_x + antenna_offset_ * std::sin(current_yaw_);
            double center_y = utm_y + antenna_offset_ * std::cos(current_yaw_);
            origin_x_ = center_x; 
            origin_y_ = center_y; 
            origin_set_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Origin Set. Heading: %.1f° ", current_yaw_ * 180.0/M_PI);
            for (auto& wp : waypoints_) {
                wp.local_x = wp.utm_x - origin_x_;
                wp.local_y = wp.utm_y - origin_y_;
            }
        }

        double center_x = utm_x + antenna_offset_ * std::sin(current_yaw_);
        double center_y = utm_y + antenna_offset_ * std::cos(current_yaw_);
        current_pos_.x = center_x - origin_x_;
        current_pos_.y = center_y - origin_y_;
        
        has_current_pos_ = true;
        last_fix_time_ = this->now();
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_ = *msg;
        has_scan_data_ = true;
    }

    // 检查前方是否有障碍物
    bool is_front_obstacle() {
        if (!has_scan_data_) return false;
        
        int size = last_scan_.ranges.size();
        if (size == 0) return false;

        // 扫描前方 -45度 到 +45度
        double angle_step = last_scan_.angle_increment;
        int center_idx = size / 2;
        int span = (int)((45.0 * M_PI / 180.0) / angle_step);
        
        int start_idx = std::max(0, center_idx - span);
        int end_idx = std::min(size - 1, center_idx + span);

        for (int i = start_idx; i <= end_idx; ++i) {
            float dist = last_scan_.ranges[i];
            if (dist > last_scan_.range_min && dist < last_scan_.range_max) {
                if (0.4 <dist && dist< obs_range_) {
                    return true;
                }
            }
        }
        return false;
    }

    // 寻找空旷方向并返回建议的角速度 (正左，负右)
    double find_clear_direction() {
        if (!has_scan_data_) return -max_w_; // 默认向右

        int size = last_scan_.ranges.size();
        int center_idx = size / 2;
        
        // 采样左侧 (90度) 和 右侧 (-90度)
        int span_90 = (int)((90.0 * M_PI / 180.0) / last_scan_.angle_increment);
        int left_idx = (center_idx + span_90) % size;
        int right_idx = (center_idx - span_90 + size) % size;

        float left_dist = 100.0;
        float right_dist = 100.0;

        if (last_scan_.ranges[left_idx] > last_scan_.range_min && last_scan_.ranges[left_idx] < last_scan_.range_max)
            left_dist = last_scan_.ranges[left_idx];
        
        if (last_scan_.ranges[right_idx] > last_scan_.range_min && last_scan_.ranges[right_idx] < last_scan_.range_max)
            right_dist = last_scan_.ranges[right_idx];

        // 哪边远往哪边转
        if (left_dist > right_dist) {
            RCLCPP_DEBUG(this->get_logger(), "Avoid: Left (%.2f) > Right (%.2f), Turn Left", left_dist, right_dist);
            return max_w_; // 向左转 (正)
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Avoid: Right (%.2f) >= Left (%.2f), Turn Right", right_dist, left_dist);
            return -max_w_; // 向右转 (负)
        }
    }

    double calculate_heading_error(double target, double current) {
        double error = target - current;
        if (error > M_PI) error -= 2.0 * M_PI;
        else if (error < -M_PI) error += 2.0 * M_PI;
        return error;
    }

    // 新增：记录并打印到达信息
    void record_arrival(const Waypoint& target, double distance, double heading_error) {
        ArrivalRecord record;
        record.wp_name = target.name;
        record.target_x = target.local_x;
        record.target_y = target.local_y;
        record.actual_x = current_pos_.x;
        record.actual_y = current_pos_.y;
        record.actual_yaw_deg = current_yaw_ * 180.0 / M_PI;
        record.dist_error = distance;
        record.yaw_error_deg = heading_error * 180.0 / M_PI;

        RCLCPP_INFO(this->get_logger(), "=========================================");
        RCLCPP_INFO(this->get_logger(), "✅ WAYPOINT REACHED: [%s]", record.wp_name.c_str());
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "🎯 Target Pos (Local): (%.4f, %.4f)", record.target_x, record.target_y);
        RCLCPP_INFO(this->get_logger(), "📍 Actual Pos (Local): (%.4f, %.4f)", record.actual_x, record.actual_y);
        RCLCPP_INFO(this->get_logger(), "🧭 Actual Yaw:         %.2f°", record.actual_yaw_deg);
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "📏 Distance Error:     %.4f m", record.dist_error);
        RCLCPP_INFO(this->get_logger(), "🔄 Yaw Error:          %.2f°", record.yaw_error_deg);
        RCLCPP_INFO(this->get_logger(), "=========================================");
        
        std::ofstream outfile("/home/test/arrival_log.txt", std::ios::app);
        if(outfile.is_open()) {
            outfile << record.wp_name << "," << record.actual_x << "," << record.actual_y 
                    << "," << record.actual_yaw_deg << "," << record.dist_error 
                    << "," << record.yaw_error_deg << std::endl;
            outfile.close();
        }
    }


    void control_loop() {
        if (!has_current_pos_ || !origin_set_ || waypoints_.empty()) {
            publish_velocity(0.0, 0.0);
            return;
        }
        if ((this->now() - last_fix_time_).seconds() > 2.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "RTK Signal Lost!");
            publish_velocity(0.0, 0.0);
            return;
        }
        if (current_wp_index_ >= waypoints_.size()) {
            publish_velocity(0.0, 0.0);
            return;
        }

        const Waypoint& target = waypoints_[current_wp_index_];
        double dx = target.local_x - current_pos_.x;
        double dy = target.local_y - current_pos_.y;
        double distance_to_target = std::hypot(dx, dy);
        double target_heading_raw = std::atan2(dy, dx);
        double heading_error = calculate_heading_error(target_heading_raw, current_yaw_);

        double v_cmd = 0.0;
        double w_cmd = 0.0;
        const char* state_str = "UNKNOWN";

        switch (current_state_) {
            case NavState::STOPPED:
            case NavState::ALIGNING: {
                state_str = "ALIGNING";
                if (std::abs(heading_error) <= align_thresh_) {
                    RCLCPP_INFO(this->get_logger(), "Aligned! Moving.");
                    current_state_ = NavState::MOVING;
                } else {
                    v_cmd = 0.0;
                    w_cmd = std::clamp(heading_error * kp_w_, -max_w_, max_w_);
                }
                break;
            }

            case NavState::AVOID_TURNING: {
                state_str = "AVOID_TURN";
                v_cmd = 0.0; // 原地旋转
                
                // 持续旋转直到前方无障碍
                if (!is_front_obstacle()) {
                    RCLCPP_INFO(this->get_logger(), "Direction clear. Starting forward avoidance.");
                    // 记录起始点和当前朝向
                    avoid_start_pos_ = current_pos_;
                    avoid_target_yaw_ = current_yaw_; 
                    current_state_ = NavState::AVOID_MOVING;
                } else {
                    // 继续旋转
                    w_cmd = find_clear_direction();
                }
                break;
            }

            case NavState::AVOID_MOVING: {
                state_str = "AVOID_MOVE";
                
                // 计算相对于绕障起点的直线距离
                double avoid_dx = current_pos_.x - avoid_start_pos_.x;
                double avoid_dy = current_pos_.y - avoid_start_pos_.y;
                double traveled_dist = std::hypot(avoid_dx, avoid_dy);

                // 保持朝向不变 (简单的 P 控制器锁定航向)
                double yaw_err = calculate_heading_error(avoid_target_yaw_, current_yaw_);
                w_cmd = std::clamp(yaw_err * kp_w_, -max_w_ * 0.5, max_w_ * 0.5);

                if (traveled_dist >= avoid_dist_) {
                    RCLCPP_INFO(this->get_logger(), "Avoidance complete (Traveled %.2fm). Resuming navigation.", traveled_dist);
                    current_state_ = NavState::ALIGNING;
                    // 重置后，下一帧会进入 MOVING 逻辑重新计算指向目标的角度
                } else {
                    // 强制以低速直线前进
                    v_cmd = max_v_ * 0.6; // 绕障时速度稍慢
                    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                        "Avoiding: Traveled %.2f / %.2f m", traveled_dist, avoid_dist_);
                }
                break;
            }

            case NavState::MOVING: {
                state_str = "MOVING";
                // 1. 优先检查障碍物
                if (is_front_obstacle()) {
                    RCLCPP_WARN(this->get_logger(), "Obstacle detected! Switching to Avoidance.");
                    current_state_ = NavState::AVOID_TURNING;
                    v_cmd = 0.0;
                    w_cmd = 0.0;
                    break; 
                }
                // 2. 到达判断
                if (distance_to_target <= arrive_dist_) {
                    record_arrival(target, distance_to_target, heading_error);
                    RCLCPP_INFO(this->get_logger(), "Reached WP[%d] (%s). Next!", current_wp_index_, target.name.c_str());
                    current_wp_index_++;
                    current_state_ = NavState::STOPPED;
                    publish_velocity(0.0, 0.0);
                    return; 
                }
    
                if (distance_to_target > slow_down_dist_) {
                    v_cmd = max_v_;
                } else {
                    v_cmd = max_v_ * (distance_to_target / slow_down_dist_);
                    if (v_cmd < 0.1) v_cmd = 0.1; // 保持最小蠕动速度
                }
                // 4. 角度控制优化 (加入抗噪死区)
                double abs_err = std::abs(heading_error);
                double err_deg = abs_err * 180.0 / M_PI;
                // --- 计算动态死区 ---
                // 假设噪声水平为 0.025m (2.5cm，留点余量)
                // 死区角度 (弧度) = 噪声 / 距离
                // 限制最小死区为 1 度，最大死区为 10 度 (防止极近距离失效或过大)
                double noise_level = 0.025; 
                double dynamic_deadband_rad = noise_level / std::max(distance_to_target, 0.1); 
                // 增加一点安全系数 (1.2 倍)
                dynamic_deadband_rad *= 1.2;
                // 转换为角度
                double dynamic_deadband_deg = dynamic_deadband_rad * 180.0 / M_PI;
                if (dynamic_deadband_deg > 10.0) dynamic_deadband_deg = 10.0;
                if (dynamic_deadband_deg < 1.5) dynamic_deadband_deg = 1.5;

                double desired_w = 0.0;
                // --- 死区判断 ---
                if (err_deg < dynamic_deadband_deg) {
                    // 误差在噪声范围内，视为 0，不转向
                    desired_w = 0.0;
                } else {
                    // 误差超出噪声范围，执行转向
                    double current_kp = kp_w_ * 2.5; 
                    desired_w = heading_error * current_kp;
                    // 动态限幅 (距离越近，最大角速度越小)
                    double max_w_allowed = max_w_;
                    if (distance_to_target < 1.2) max_w_allowed = max_w_ ;
                    if (distance_to_target < 0.5) max_w_allowed = max_w_ * 0.2;
                    
                    w_cmd = std::clamp(desired_w, -max_w_allowed, max_w_allowed);
                }

                break;
            }
            
            default:
                v_cmd = 0.0; w_cmd = 0.0;
                break;
        }
        
        // 日志输出
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[%s] Dist(Tgt): %.2f, Dist(Avoid): %.2f, Err: %.1f°",
            state_str, distance_to_target, 
            (current_state_ == NavState::AVOID_MOVING) ? 
                std::hypot(current_pos_.x - avoid_start_pos_.x, current_pos_.y - avoid_start_pos_.y) : 0.0,
            heading_error * 180.0/M_PI);

        publish_velocity(v_cmd, w_cmd);
    }

    void publish_velocity(double linear, double angular) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linear;
        cmd.angular.z = angular;
        pub_cmd_->publish(cmd);
    }

    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw; 
    }

    // 成员变量
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_utm_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Waypoint> waypoints_;
    
    bool origin_set_;
    double origin_x_, origin_y_;
    int current_wp_index_;
    Point2D current_pos_; 
    double current_yaw_; 
    rclcpp::Time last_fix_time_;
    bool has_current_pos_;
    
    NavState current_state_;
    double max_v_, max_w_, arrive_dist_, align_thresh_, slow_down_dist_, kp_w_, antenna_offset_;
    
    // 绕障变量
    double obs_range_;
    double avoid_dist_; 
    sensor_msgs::msg::LaserScan last_scan_;
    bool has_scan_data_;
    Point2D avoid_start_pos_;
    double avoid_target_yaw_;
}; 

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DogController>());
    rclcpp::shutdown();
    return 0;
}
