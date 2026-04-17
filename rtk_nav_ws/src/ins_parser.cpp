#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> 
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>
#include <algorithm> // for trim

// Linux 串口相关头文件
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>

// GeographicLib 用于 UTM 转换
#include <GeographicLib/UTMUPS.hpp>

// PROJ 用于 EPSG 投影转换
#include <proj.h>

// TF2 用于 欧拉角转四元数
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 辅助函数：去除字符串两端空格
static inline std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, (last - first + 1));
}

class SerialPort {
public:
    int fd_;
    SerialPort() : fd_(-1) {}
    
    bool open(const std::string& port, int baudrate) {
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("serial"), "Cannot open port %s: %s", port.c_str(), strerror(errno));
            return false;
        }

        struct termios config;
        if (tcgetattr(fd_, &config) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("serial"), "Failed to get port attributes");
            return false;
        }

        speed_t speed;
        switch(baudrate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            case 460800: speed = B460800; break;
            case 921600: speed = B921600; break;
            default: 
                RCLCPP_ERROR(rclcpp::get_logger("serial"), "Unsupported baudrate %d", baudrate);
                return false;
        }
        cfsetispeed(&config, speed);
        cfsetospeed(&config, speed);

        config.c_cflag |= (CLOCAL | CREAD);
        config.c_cflag &= ~PARENB;
        config.c_cflag &= ~CSTOPB;
        config.c_cflag &= ~CSIZE;
        config.c_cflag |= CS8;
        config.c_cflag &= ~CRTSCTS;
        config.c_iflag &= ~(IXON | IXOFF | IXANY);
        config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        config.c_oflag &= ~OPOST;
        config.c_cc[VTIME] = 10; 
        config.c_cc[VMIN] = 0;

        tcflush(fd_, TCIFLUSH);
        if (tcsetattr(fd_, TCSANOW, &config) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("serial"), "Failed to set port attributes");
            return false;
        }

      
        return true;
    }

    ssize_t read(char* buffer, size_t size) {
        return ::read(fd_, buffer, size);
    }

    void close() {
        if (fd_ != -1) ::close(fd_);
    }
};

struct GnssData {
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    double yaw_deg = 0.0;
    int gps_qual = 0;     
    bool has_heading = false;
    bool is_heading_computed = false;
    rclcpp::Time stamp;
    bool has_valid_gps = false; // 标记是否收到过有效的 GPGGA
};

class InsNode : public rclcpp::Node {
public:
    InsNode() : Node("ins_parser_node"), utm_zone_(-1), utm_northp_(true), gpgga_count_(0), headinga_count_(0) {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<bool>("use_epsg_crs_datum", true);

        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        use_epsg_crs_datum_ = this->get_parameter("use_epsg_crs_datum").as_bool();

        pub_fix_ = this->create_publisher<geometry_msgs::msg::PointStamped>("fix", 10);
        utm_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("utm_fix", 10);
        epsg_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("epsg_position", 10);
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", 10);

        RCLCPP_INFO(this->get_logger(), "=== INS Parser Node Started ===");
        RCLCPP_INFO(this->get_logger(), "Port: %s, Baud: %d", port.c_str(), baudrate);
        RCLCPP_INFO(this->get_logger(), "Waiting for $GPGGA and #HEADINGA messages...");

        if (!serial_.open(port, baudrate)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port. Exiting.");
            exit(1);
        }

        thread_ = std::thread(&InsNode::read_loop, this);
    }

    ~InsNode() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        serial_.close();
    }

private:
    void read_loop() {
        char buffer[2048];
        std::string line_buffer;
        
        while (rclcpp::ok() && running_) {
            ssize_t n = serial_.read(buffer, sizeof(buffer));
            if (n > 0) {
                for (int i = 0; i < n; ++i) {
                    char c = buffer[i];
                    if (c == '\n') {
                        process_line(line_buffer);
                        line_buffer.clear();
                    } else if (c != '\r') {
                        line_buffer += c;
                    }
                }
            } else if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Read error: %s", strerror(errno));
                    break;
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    std::vector<std::string> split(const std::string& s, char delimiter) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }

    void process_line(const std::string& line) {
        if (line.empty()) return;

        if (line.find("$GPGGA") != std::string::npos) {
            parse_gpgga(line);
        }
        else if (line.find("#HEADINGA") != std::string::npos) {
            parse_headinga(line);
        }
    }

    void parse_gpgga(const std::string& line) {
        gpgga_count_++;
        if (gpgga_count_ <= 5) {
             RCLCPP_INFO(this->get_logger(), "[Raw GPGGA] %s", line.c_str());
        }

        std::string data_part = line;
        size_t star_pos = data_part.find('*');
        if (star_pos != std::string::npos) {
            data_part = data_part.substr(0, star_pos);
        }

        std::vector<std::string> fields = split(data_part, ',');
        
        // GPGGA 最少需要 10 个字段 (Index 0-9)
        if (fields.size() < 10) {
            if (gpgga_count_ <= 5) RCLCPP_WARN(this->get_logger(), "GPGGA: Too few fields (%zu)", fields.size());
            return;
        }

        try {
            GnssData current_data;
            current_data.stamp = this->now();

            // Index 2: Lat
            if (!fields[2].empty()) {
                double lat_raw = std::stod(fields[2]);
                double lat_deg = std::floor(lat_raw / 100.0);
                double lat_min = lat_raw - (lat_deg * 100.0);
                current_data.lat = lat_deg + (lat_min / 60.0);
                if (fields[3] == "S") current_data.lat = -current_data.lat;
            }

            // Index 4: Lon
            if (!fields[4].empty()) {
                double lon_raw = std::stod(fields[4]);
                double lon_deg = std::floor(lon_raw / 100.0);
                double lon_min = lon_raw - (lon_deg * 100.0);
                current_data.lon = lon_deg + (lon_min / 60.0);
                if (fields[5] == "W") current_data.lon = -current_data.lon;
            }

            // Index 6: Quality
            if (!fields[6].empty()) {
                current_data.gps_qual = std::stoi(fields[6]);
            }

            // Index 9: Altitude
            if (!fields[9].empty()) {
                current_data.alt = std::stod(fields[9]);
            }

            // 更新全局数据
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                gnss_data_.lat = current_data.lat;
                gnss_data_.lon = current_data.lon;
                gnss_data_.alt = current_data.alt;
                gnss_data_.gps_qual = current_data.gps_qual;
                gnss_data_.stamp = current_data.stamp;
                
                if (current_data.gps_qual > 0) {
                    gnss_data_.has_valid_gps = true;
                }
            }

            if (gpgga_count_ <= 5 || (gpgga_count_ % 50 == 0)) {
                RCLCPP_INFO(this->get_logger(), "GPGGA Parsed: Qual=%d, Lat=%.6f, Lon=%.6f, Alt=%.2f", 
                            current_data.gps_qual, current_data.lat, current_data.lon, current_data.alt);
            }

            // 如果只有 GPGGA 来了，也发布一下 fix 话题用于调试
            if (current_data.gps_qual > 0) {
                publish_debug_fix();
                publish_gps();
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "GPGGA Parse Exception: %s", e.what());
            if (fields.size() > 6) {
                RCLCPP_ERROR(this->get_logger(), "Fields around quality: [%s], [%s], [%s]", 
                             fields[5].c_str(), fields[6].c_str(), fields[7].c_str());
            }
        }
    }

    void parse_headinga(const std::string& line) {
        headinga_count_++;
        // 仅打印前几条原始报文用于确认
        if (headinga_count_ <= 3) {
            RCLCPP_INFO(this->get_logger(), "[Raw HEADINGA] %s", line.c_str());
        }

        // 1. 去除校验和 (*xx)
        std::string data_part = line;
        size_t star_pos = data_part.find('*');
        if (star_pos != std::string::npos) {
            data_part = data_part.substr(0, star_pos);
        }

        // 2. 【关键修复】将分号替换为逗号，解决 "1114;SOL_COMPUTED" 问题
        std::replace(data_part.begin(), data_part.end(), ';', ',');

        std::vector<std::string> fields = split(data_part, ',');

        if (fields.size() < 10) {
            if (headinga_count_ <= 3) RCLCPP_WARN(this->get_logger(), "HEADINGA: Too few fields (%zu)", fields.size());
            return;
        }

        try {
            // --- 步骤 A: 定位状态字段 (Status) ---
            // 寻找包含 "SOL_COMPUTED" 的字段索引
            int status_idx = -1;
            bool is_computed = false;
            
            // 从索引 1 开始搜索（跳过 #HEADINGA）
            for (size_t i = 1; i < fields.size(); ++i) {
                std::string token = trim(fields[i]);
                if (token.find("SOL_COMPUTED") != std::string::npos) {
                    status_idx = static_cast<int>(i);
                    is_computed = true;
                    break;
                }
                // 也可以检查其他非计算状态，如 "INS_HIGH_VARIANCE" 等，视需求而定
                if (token.find("WAITING") != std::string::npos || token.find("COASTING") != std::string::npos) {
                    status_idx = static_cast<int>(i);
                    is_computed = false; // 标记为未解算
                    break;
                }
            }

            if (status_idx == -1) {
                // 如果找不到状态字段，可能是报文格式严重错误或不是 HEADINGA
                if (headinga_count_ <= 3) {
                    RCLCPP_WARN(this->get_logger(), "HEADINGA: Status keyword not found. Fields dump:");
                    for(size_t k=0; k<std::min(fields.size(), (size_t)12); ++k) {
                        RCLCPP_WARN(this->get_logger(), "  [%zu]: '%s'", k, fields[k].c_str());
                    }
                }
                return;
            }

            // --- 步骤 B: 提取航向 (Heading) ---
            // 根据 NovAtel 定义: Status, SolType, AzimuthStdDev, Heading
            // 索引关系: Status(idx), SolType(idx+1), StdDev(idx+2), Heading(idx+3)
            int yaw_idx = status_idx + 3;

            double yaw = 0.0;
            bool yaw_valid = false;

            if (yaw_idx < static_cast<int>(fields.size())) {
                std::string yaw_str = trim(fields[yaw_idx]);
                // 简单验证：确保是数字开头
                if (!yaw_str.empty() && (isdigit(yaw_str[0]) || yaw_str[0] == '-')) {
                    yaw = std::stod(yaw_str);
                    yaw_valid = true;
                }
            }

            if (!yaw_valid) {
                if (headinga_count_ <= 3) {
                    RCLCPP_WARN(this->get_logger(), "HEADINGA: Failed to parse Yaw at index %d (Value: '%s').", 
                                yaw_idx, yaw_idx < (int)fields.size() ? fields[yaw_idx].c_str() : "OUT_OF_RANGE");
                }
                // 如果航向解析失败，可以选择返回或继续使用旧值，这里选择返回以避免发布错误数据
                return; 
            }

            // --- 步骤 C: 更新全局数据 ---
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                gnss_data_.yaw_deg = yaw;
                gnss_data_.has_heading = true;
                gnss_data_.is_heading_computed = is_computed;
                gnss_data_.stamp = this->now();
            }

            // 日志输出
            if (headinga_count_ <= 3 || (headinga_count_ % 50 == 0)) {
                RCLCPP_INFO(this->get_logger(), "HEADINGA OK: Status=[%s], Yaw=%.6f deg", 
                            fields[status_idx].c_str(), yaw);
            }

            // --- 步骤 D: 发布数据 ---
            if (is_computed && yaw_valid) {
                publish_utm_pose();
                publish_epsg_pose();
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "HEADINGA Exception: %s", e.what());
        }
    }
    void publish_debug_fix() {
        auto msg_fix = std::make_unique<geometry_msgs::msg::PointStamped>();
        msg_fix->header.stamp = gnss_data_.stamp;
        msg_fix->header.frame_id = "gps_link";
        msg_fix->point.x = gnss_data_.lon;
        msg_fix->point.y = gnss_data_.lat;
        msg_fix->point.z = gnss_data_.alt;
        pub_fix_->publish(std::move(msg_fix));
    }

    void publish_gps() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        auto msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
        msg->header.stamp = gnss_data_.stamp;
        msg->header.frame_id = "gps_link";
        msg->latitude = gnss_data_.lat;
        msg->longitude = gnss_data_.lon;
        msg->altitude = gnss_data_.alt;

        switch (gnss_data_.gps_qual) {
            case 4:  // RTK Fixed
                msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                break;
            case 5:  // RTK Float
            case 2:  // DGPS
                msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
                break;
            case 1:  // GPS Fix
                msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                break;
            default:
                msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                break;
        }
        msg->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

        msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        gps_pub_->publish(std::move(msg));
    }

    void publish_utm_pose() {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // --- 关键调试日志 ---
        static int debug_log_count = 0;
        if (++debug_log_count % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), ">>> Check Status: GPS_Qual=%d (Valid=%d), Heading_Computed=%d", 
                        gnss_data_.gps_qual, gnss_data_.has_valid_gps ? 1 : 0, gnss_data_.is_heading_computed ? 1 : 0);
        }

        // 放宽条件：只要曾经收到过有效 GPS (has_valid_gps)，且当前航向有效，就尝试发布
        // 防止 GPGGA 更新慢导致中间帧丢失
        if (!gnss_data_.has_valid_gps) {
            return; // 从未收到过有效 GPS
        }
        
        if (!gnss_data_.is_heading_computed) {
            return; // 航向未解算
        }

        // 如果当前 gps_qual 为 0，但之前有过有效数据，我们可以选择继续发布最近的有效位置（取决于应用需求）
        // 这里为了安全，如果当前 qual 为 0 且距离上次有效时间过长，可以选择不发布。
        // 简单起见，只要 has_valid_gps 为真，且当前坐标不全是 0，就发布。
        
        bool is_valid_pos = (std::abs(gnss_data_.lat) > 1e-6 || std::abs(gnss_data_.lon) > 1e-6);
        if (!is_valid_pos) return;

        double utm_x, utm_y;
        int zone;
        bool northp;

        try {
            GeographicLib::UTMUPS::Forward(gnss_data_.lat, gnss_data_.lon, zone, northp, utm_x, utm_y);

            if (utm_zone_ == -1) {
                utm_zone_ = zone;
                utm_northp_ = northp;
                RCLCPP_INFO(this->get_logger(), "UTM Zone initialized: %d%s", zone, northp ? "N" : "S");
            }

            auto msg_utm = std::make_unique<geometry_msgs::msg::PoseStamped>();
            msg_utm->header.stamp = gnss_data_.stamp;
            std::string frame_name = "utm_" + std::to_string(zone) + (northp ? "N" : "S");
            msg_utm->header.frame_id = frame_name;
            
            msg_utm->pose.position.x = utm_x;
            msg_utm->pose.position.y = utm_y;
            msg_utm->pose.position.z = gnss_data_.alt;

            double yaw_ros_deg = 90.0 - gnss_data_.yaw_deg;
            double yaw_ros_rad = yaw_ros_deg * M_PI / 180.0;

            yaw_ros_rad = std::fmod(yaw_ros_rad, 2.0 * M_PI);
            if (yaw_ros_rad > M_PI) yaw_ros_rad -= 2.0 * M_PI;
            if (yaw_ros_rad <= -M_PI) yaw_ros_rad += 2.0 * M_PI;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw_ros_rad); 
            q.normalize();
            
            msg_utm->pose.orientation.x = q.x();
            msg_utm->pose.orientation.y = q.y();
            msg_utm->pose.orientation.z = q.z();
            msg_utm->pose.orientation.w = q.w();

            utm_pub_->publish(std::move(msg_utm));

            if (debug_log_count % 20 == 0) {
                RCLCPP_INFO(this->get_logger(), ">>> SUCCESS: Published UTM Pose. Lat=%.6f, Lon=%.6f, Yaw=%.2f (ROS: %.2f)", 
                            gnss_data_.lat, gnss_data_.lon, gnss_data_.yaw_deg, yaw_ros_deg);
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "UTM Conversion Error: %s", e.what());
        }
    }

    void publish_epsg_pose() {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (!gnss_data_.has_valid_gps || !gnss_data_.is_heading_computed) return;

        bool is_valid_pos = (std::abs(gnss_data_.lat) > 1e-6 || std::abs(gnss_data_.lon) > 1e-6);
        if (!is_valid_pos) return;

        PJ_CONTEXT *C = proj_context_create();
        if (!C) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to create PROJ context");
            return;
        }

        PJ *P = nullptr;
        if (use_epsg_crs_datum_) {
            P = proj_create_crs_to_crs(C, "EPSG:4326", "EPSG:2100", nullptr);
        } else {
            P = proj_create(C,
                "+proj=tmerc "
                "+lat_0=0 "
                "+lon_0=24 "
                "+k=0.9996 "
                "+x_0=500000 "
                "+y_0=0 "
                "+ellps=GRS80 "
                "+units=m "
                "+no_defs"
            );
        }

        if (!P) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to create EPSG projection");
            proj_context_destroy(C);
            return;
        }

        P = proj_normalize_for_visualization(C, P);

        PJ_COORD coord = proj_coord(gnss_data_.lon, gnss_data_.lat, gnss_data_.alt, 0);
        coord = proj_trans(P, PJ_FWD, coord);

        double epsg_x = coord.xy.x;
        double epsg_y = coord.xy.y;

        auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        msg->header.stamp = gnss_data_.stamp;
        msg->header.frame_id = "epsg_2100";

        msg->pose.position.x = epsg_x;
        msg->pose.position.y = epsg_y;
        msg->pose.position.z = coord.xyz.z;

        double yaw_ros_deg = 90.0 - gnss_data_.yaw_deg;
        double yaw_ros_rad = yaw_ros_deg * M_PI / 180.0;
        yaw_ros_rad = std::fmod(yaw_ros_rad, 2.0 * M_PI);
        if (yaw_ros_rad > M_PI) yaw_ros_rad -= 2.0 * M_PI;
        if (yaw_ros_rad <= -M_PI) yaw_ros_rad += 2.0 * M_PI;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_ros_rad);
        q.normalize();

        msg->pose.orientation.x = q.x();
        msg->pose.orientation.y = q.y();
        msg->pose.orientation.z = q.z();
        msg->pose.orientation.w = q.w();

        epsg_pub_->publish(std::move(msg));

        static int epsg_log_count = 0;
        if (++epsg_log_count % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), "EPSG Pose: x=%.3f, y=%.3f, yaw=%.2f",
                        epsg_x, epsg_y, yaw_ros_deg);
        }

        proj_destroy(P);
        proj_context_destroy(C);
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_fix_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr utm_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr epsg_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_; 
    
    SerialPort serial_;
    std::thread thread_;
    bool running_ = true;

    int utm_zone_;
    bool utm_northp_;
    bool use_epsg_crs_datum_;
    std::mutex data_mutex_;
    GnssData gnss_data_;
    
    int gpgga_count_;
    int headinga_count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InsNode>());
    rclcpp::shutdown();
    return 0;
}
