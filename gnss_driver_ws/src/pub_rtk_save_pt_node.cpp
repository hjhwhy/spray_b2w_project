#include <rclcpp/rclcpp.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp> 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <memory>
#include <array>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <sstream>  
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/assign/list_of.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// GeographicLib for UTM conversion
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Constants.hpp>

#include <proj.h>
#include <std_srvs/srv/trigger.hpp>

using boost::asio::ip::tcp;

class GNSSDriver : public rclcpp::Node
{
public:
    GNSSDriver() : Node("gnss_driver"), point_counter_(1),
    latest_orientation_(tf2::toMsg(tf2::Quaternion::getIdentity()))
    {
        this->declare_parameter<std::string>("connect_mode", "serial");
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud", 9600);
        this->declare_parameter<std::string>("waypoint_file", "~/gnss_waypoints.txt");
        this->declare_parameter<std::string>("project_mode", "utm");
        this->declare_parameter<bool>("use_epsg_crs_datum", true);
        
        connect_mode_ = this->get_parameter("connect_mode").as_string();
        port_name_ = this->get_parameter("port").as_string();
        baud_rate_ = this->get_parameter("baud").as_int();
        waypoint_file_ = this->get_parameter("waypoint_file").as_string();
        project_mode_ = this->get_parameter("project_mode").as_string();
        use_epsg_crs_datum_ = this->get_parameter("use_epsg_crs_datum").as_bool();

        if (waypoint_file_.substr(0, 2) == "~/") {
            const char* home = std::getenv("HOME");
            if (home != nullptr) {
                waypoint_file_ = std::string(home) + waypoint_file_.substr(1);
            }
        }
        RCLCPP_INFO(this->get_logger(), "connect_mode = %s", connect_mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "Waypoint file will be saved to: %s", waypoint_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "project_mode = %s", project_mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "use_epsg_crs_datum = %s",use_epsg_crs_datum_ ? "true" : "false");
        // 创建发布者
        nmea_sentence_pub_ = this->create_publisher<nmea_msgs::msg::Sentence>("nmea_sentence", 1000);
        gnss_pos_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gnss_pos", 10);
        gnss_yaw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("gnss_yaw", 10);
        utm_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("utm_position", 10); 
        epsg_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("epsg_position", 10);

        save_point_service_ = this->create_service<std_srvs::srv::Trigger>(
            "save_utm_point",
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request_header;
                (void)request;
                saveCurrentUTMPoint(response);
            });
        
        // 初始化 TF Buffer 和 Listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 初始化上一次保存的点
        last_saved_point_.x = 0.0;
        last_saved_point_.y = 0.0;
        last_saved_point_.z = 0.0;
        has_valid_point_ = false;
        // 启动连接线程
        if (connect_mode_ == "serial") {
            serial_thread_ = std::thread(&GNSSDriver::runSerial, this);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown Connect Mode: %s", connect_mode_.c_str());
        }
    }

    ~GNSSDriver()
    {
        running_ = false;
        if (serial_thread_.joinable()) {
            serial_thread_.join();
        }
    }

private:
    void saveCurrentUTMPoint(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
    {
        if (!has_valid_point_) {
            response->success = false;
            response->message = "No valid UTM point available yet.";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        std::ofstream file;
        bool append = std::filesystem::exists(waypoint_file_);
        file.open(waypoint_file_, std::ios::app);

        if (!file.is_open()) {
            response->success = false;
            response->message = "Failed to open file: " + waypoint_file_;
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        std::string point_name = "pt" + std::to_string(point_counter_++);
        file << point_name << ","
             << std::fixed << std::setprecision(3)
             << last_saved_point_.x << ","
             << last_saved_point_.y << ","
             << last_saved_point_.z << ",\n";
        file.close();

        response->success = true;
        response->message = "Saved point: " + point_name +
                            " (" + std::to_string(last_saved_point_.x) + ", " +
                            std::to_string(last_saved_point_.y) + ", " +
                            std::to_string(last_saved_point_.z) + ")";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    // log file paths 
    //std::ofstream gnss_pos_file_;
    //std::ofstream gnss_odom_file_;
    //std::ofstream epsg_file_;

    std::string connect_mode_;
    std::string port_name_;
    int baud_rate_;
    std::string waypoint_file_;
    std::string project_mode_;
    bool use_epsg_crs_datum_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_sentence_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_pos_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gnss_yaw_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr utm_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr epsg_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_point_service_;
    // 用于保存最新航向（来自 TRA），供 Odometry 使用
    geometry_msgs::msg::Quaternion latest_orientation_;
    std::mutex yaw_mutex_;

    std::thread serial_thread_;
    std::atomic<bool> running_{true};
    // 串口相关
    boost::asio::io_context serial_io_context_;
    std::unique_ptr<boost::asio::serial_port> serial_port_;
    std::thread serial_read_thread_;
    // 用于缓存未完整接收的数据
    std::string nmea_serial_buffer_;

    typedef enum { GGA, TRA, RMC, UNKNOWN } nmea_sentence_type;

    // 检查NMEA句子的校验和
    bool checksum(const std::string &nmea)
    {
        size_t len = nmea.length();
        if (len < 6) return false;
        size_t asterisk_pos = nmea.find('*');
        if (asterisk_pos == std::string::npos || asterisk_pos < 1) return false;

        unsigned int given_checksum = 0;
        std::string hex_str = nmea.substr(asterisk_pos + 1, 2);
        if (hex_str.length() == 2) {
            given_checksum = static_cast<unsigned int>(strtol(hex_str.c_str(), nullptr, 16));
        } else {
            return false;
        }

        unsigned int computed_checksum = 0;
        for (size_t i = 1; i < asterisk_pos; ++i) {
            computed_checksum ^= static_cast<unsigned char>(nmea[i]);
        }
        return given_checksum == computed_checksum;
    }

    void gnssCallback(const std::string &nmea)
    {
        bool is_nmea = checksum(nmea);
        if (!is_nmea)
        {
            RCLCPP_ERROR(this->get_logger(), "NMEA Sentence Check Failed!");
            RCLCPP_INFO(this->get_logger(), "Sentence : %s", nmea.c_str());
            return;
        }

        std::vector<std::string> nmea_split;
        boost::split(nmea_split, nmea, boost::is_any_of(","));
        nmea_sentence_type nst = UNKNOWN;
        if (nmea_split[0].find("GGA") != std::string::npos)
            nst = GGA;
        if (nmea_split[0].find("TRA") != std::string::npos)
            nst = TRA;
        if (nmea_split[0].find("RMC") != std::string::npos)
            nst = RMC;

        sensor_msgs::msg::NavSatFix gnss_pos_msg;
        sensor_msgs::msg::Imu gnss_yaw_msg;

        std::string gnss_frame_id = "rtk_link";
        std::string imu_frame_id = "imu_link";

        switch (nst)
        {
        case GGA:
        {
            if (nmea_split.size() < 10) {
                RCLCPP_WARN(this->get_logger(), "GGA sentence has insufficient fields");
                return;
            }

            double degree, minute;
            double hdop;

            if (!nmea_split[2].empty()) {
                degree = (int)strtod(nmea_split[2].c_str(), nullptr) / 100;
                minute = strtod(nmea_split[2].c_str(), nullptr) - degree * 100;
                gnss_pos_msg.latitude = degree + minute / 60.0;
            }
            if (!nmea_split[4].empty()) {
                degree = (int)strtod(nmea_split[4].c_str(), nullptr) / 100;
                minute = strtod(nmea_split[4].c_str(), nullptr) - degree * 100;
                gnss_pos_msg.longitude = degree + minute / 60.0;
            }
            if (!nmea_split[9].empty()) {
                gnss_pos_msg.altitude = strtod(nmea_split[9].c_str(), nullptr);
            }
            if (!nmea_split[7].empty()) {
                int satnu = strtol(nmea_split[7].c_str(), nullptr, 10);
                gnss_pos_msg.position_covariance_type = static_cast<uint8_t>(satnu);
            }
            if (!nmea_split[8].empty()) {
                hdop = strtod(nmea_split[8].c_str(), nullptr);
                double pos_cov = hdop * hdop / 2.0;
                gnss_pos_msg.position_covariance = boost::assign::list_of(pos_cov)(0)(0)(0)(pos_cov)(0)(0)(0)(pos_cov);
            }
            if (!nmea_split[6].empty()) {
                int pos_status = strtol(nmea_split[6].c_str(), nullptr, 10);
                if (pos_status == 0)
                    gnss_pos_msg.status.status = -1;
                else if (pos_status == 1)
                    gnss_pos_msg.status.status = 0;
                else if (pos_status == 2 || pos_status == 5)
                    gnss_pos_msg.status.status = 1;
                else if (pos_status == 4)
                    gnss_pos_msg.status.status = 2;
            }

            gnss_pos_msg.header.stamp = this->now();
            gnss_pos_msg.header.frame_id = gnss_frame_id;
            gnss_pos_pub_->publish(gnss_pos_msg);
            /*  Save GNSS POS 
            if (gnss_pos_file_.is_open()) {
                gnss_pos_file_ << this->now().seconds() << ","
                    << gnss_pos_msg.latitude << ","<< gnss_pos_msg.longitude << ","
                    << gnss_pos_msg.altitude << "\n";
                gnss_pos_file_.flush();
            }*/
            // --- START: UTM Conversion and Publish ---
            double utm_x, utm_y, epsg_x, epsg_y;
            try {
                if (project_mode_ == "utm") {
                    bool northp;
                    int zone;
                    GeographicLib::UTMUPS::Forward(
                        gnss_pos_msg.latitude,
                        gnss_pos_msg.longitude,
                        zone, northp, utm_x, utm_y
                    );
                    // Create PointStamped message
                    geometry_msgs::msg::PointStamped utm_point;
                    utm_point.header.stamp = this->now();
                    std::stringstream ss;
                    ss << "utm_zone_" << zone << (northp ? "N" : "S");
                    utm_point.header.frame_id = ss.str();
                    utm_point.point.x = utm_x;
                    utm_point.point.y = utm_y;
                    utm_point.point.z = gnss_pos_msg.altitude;
                    utm_pub_->publish(utm_point);

                    last_saved_point_.x = utm_x;
                    last_saved_point_.y = utm_y;
                    last_saved_point_.z = gnss_pos_msg.altitude;
                    has_valid_point_ = true;
                    
                } else {
                    PJ_CONTEXT *C = proj_context_create();
                    if (!C) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to create PROJ context");
                        return;
                    }
                    PJ *P = nullptr;
                    if (use_epsg_crs_datum_) {
                        // WGS84 -> EPSG:2100
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
                        RCLCPP_ERROR(this->get_logger(), "Failed to create projection object");
                        proj_context_destroy(C);
                        return;
                    }
                    P = proj_normalize_for_visualization(C, P);
                    // 经纬度转换
                    PJ_COORD coord = proj_coord(gnss_pos_msg.longitude, gnss_pos_msg.latitude, gnss_pos_msg.altitude, 0);
                    coord = proj_trans(P, PJ_FWD, coord);

                    epsg_x = coord.xy.x;
                    epsg_y = coord.xy.y;
                    // 发布 PointStamped 消息
                    geometry_msgs::msg::PointStamped greek_point;
                    greek_point.header.stamp = this->now();
                    greek_point.header.frame_id = "epsg_2100";
                    greek_point.point.x = epsg_x;
                    greek_point.point.y = epsg_y;
                    greek_point.point.z = coord.xyz.z;
                    epsg_pub_->publish(greek_point);

                    proj_destroy(P);
                    proj_context_destroy(C);

                    last_saved_point_.x = epsg_x;
                    last_saved_point_.y = epsg_y;
                    last_saved_point_.z = gnss_pos_msg.altitude;
                    has_valid_point_ = true;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "GNSS (lat, lon): [%.8f, %.8f] | EPSG:2100 (x, y): [%.3f, %.3f]",
                        gnss_pos_msg.latitude, gnss_pos_msg.longitude, epsg_x, epsg_y);
                }

            }
            catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "UTM conversion failed: %s", e.what());
            }
            // --- END: UTM Conversion and Publish ---
        }
        break;

        case TRA:
        {
            if (nmea_split.size() < 6) {
                RCLCPP_WARN(this->get_logger(), "TRA sentence has insufficient fields");
                return;
            }

            if (!nmea_split[5].empty()) {
                int status = strtol(nmea_split[5].c_str(), nullptr, 10);
                if (status == 4) // 定向固定解
                {
                    if (!nmea_split[2].empty()) {
                        double yaw = strtod(nmea_split[2].c_str(), nullptr);
                        RCLCPP_INFO(this->get_logger(), "YAW = %lf", yaw);
                        yaw = 90 - yaw;
                        if (yaw < 0) yaw += 360;
                        yaw = yaw * M_PI / 180.0;

                        tf2::Quaternion q;
                        q.setRPY(0, 0, yaw);

                        {
                            std::lock_guard<std::mutex> lock(yaw_mutex_);
                            latest_orientation_.x = q.x();
                            latest_orientation_.y = q.y();
                            latest_orientation_.z = q.z();
                            latest_orientation_.w = q.w();
                        }

                        gnss_yaw_msg.orientation = latest_orientation_;
                        gnss_yaw_msg.header.stamp = this->now();
                        gnss_yaw_msg.header.frame_id = imu_frame_id;
                        gnss_yaw_pub_->publish(gnss_yaw_msg);
                    }
                }
            }
        }
        break;

        case RMC:
            break;

        case UNKNOWN:
            break;

        default:
            break;
        }
    }

    void processNmeaBuffer(std::string& buffer)
    {
        while (running_) {
            size_t end_pos = buffer.find("\r\n");
            if (end_pos == std::string::npos) {
                if (buffer.length() > 1024) {
                    RCLCPP_WARN(this->get_logger(), "No \\r\\n found for a long time, clearing buffer.");
                    buffer.clear();
                }
                break;
            }

            size_t start_pos = std::string::npos;
            size_t dollar_pos = buffer.find('$');
            size_t hash_pos = buffer.find('#');
            if (dollar_pos != std::string::npos && dollar_pos < end_pos)
                start_pos = dollar_pos;
            else if (hash_pos != std::string::npos && hash_pos < end_pos)
                start_pos = hash_pos;

            if (start_pos == std::string::npos || start_pos > end_pos) {
                buffer.erase(0, end_pos + 2);
                continue;
            }

            std::string sentence = buffer.substr(start_pos, end_pos + 2 - start_pos);
            buffer.erase(0, end_pos + 2);

            if (sentence.length() >= 2 && sentence.substr(sentence.length()-2) == "\r\n") {
                sentence = sentence.substr(0, sentence.length()-2);
            }

            if (!sentence.empty()) {
                RCLCPP_DEBUG(this->get_logger(), "Parsed NMEA: %s", sentence.c_str());
                nmea_msgs::msg::Sentence nmea_msg;
                nmea_msg.header.stamp = this->now();
                nmea_msg.header.frame_id = "rtk_link";
                nmea_msg.sentence = sentence;
                nmea_sentence_pub_->publish(nmea_msg);
                if (sentence.find("#") == std::string::npos) {
                    gnssCallback(sentence);
                }
            }
        }
    }

    void runSerial()
    {
        try {
            serial_port_ = std::make_unique<boost::asio::serial_port>(serial_io_context_);
            serial_port_->open(port_name_);
            serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
            serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
            serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

            RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully, baudrate: %d", 
                        port_name_.c_str(), baud_rate_);

            serial_read_thread_ = std::thread([this]() {
                serialReadTask();
            });

            serial_io_context_.run();
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial error: %s", e.what());
        }
    }

    void serialReadTask()
    {
        std::array<uint8_t, 1024> buffer;
        boost::system::error_code ec;
        while (running_ && rclcpp::ok()) {
            size_t bytes_transferred = serial_port_->read_some(boost::asio::buffer(buffer), ec);
            if (ec == boost::asio::error::operation_aborted ||
                ec == boost::asio::error::bad_descriptor) {
                break;
            }
            if (!ec) {
                nmea_serial_buffer_ += std::string(reinterpret_cast<char*>(buffer.data()), bytes_transferred);
                processNmeaBuffer(nmea_serial_buffer_);
            } else {
                RCLCPP_WARN(this->get_logger(), "Serial read error: %s", ec.message().c_str());
            }
        }
        RCLCPP_INFO(this->get_logger(), "Serial read thread exiting");
    }

    // 成员变量
    std::atomic<int> point_counter_;
    geometry_msgs::msg::Point last_saved_point_; 
    bool has_valid_point_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GNSSDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}