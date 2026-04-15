#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/byte.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joy.hpp> 
#include <nav_msgs/msg/odometry.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <mutex>
#include <thread>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/path.hpp> 
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <fstream>
#include <string>
#include <std_srvs/srv/trigger.hpp>

struct Point {
    std::string id;
    double x, y, z;
    Point(const std::string& i, double x_val, double y_val, double z_val)
        : id(i), x(x_val), y(y_val), z(z_val) {}
};

class RemoteControlNode : public rclcpp::Node
{
public:
    RemoteControlNode()
    : Node("remote_control_node") {
        this->declare_parameter<int>("listen_port", 8080);
        int port = this->get_parameter("listen_port").as_int();
        std::string home = std::getenv("HOME");
        std::string file_path = home + "/gnss_waypoints.txt";
        all_points_ = parsePoints(file_path);
        if (all_points_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "未能读取到任何有效点位！");
            return;
        }

        command_pub_ = this->create_publisher<std_msgs::msg::String>("remote_command", 10);
        joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
        progress_sub_ = this->create_subscription<std_msgs::msg::Byte>("/progress", 10,
            [this](const std_msgs::msg::Byte::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received /progress: %u", msg->data);
                sendProgress(msg->data);
            });
        position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/b2w_odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            sendPosition(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);  // 直接传 double
        });
        motors_temp_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/motors_temperatures", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            sendMaxTemperature(msg->data);
        });
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/b2w_path", 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                sendPath(msg);
        });
        acquired_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/acquired_points", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received /acquired_points");
            sendPointCloud(msg, 0x02);
        });
        unacquired_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/unacquired_points", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received /unacquired_points");
            sendPointCloud(msg, 0x03);
        });
        emergency_stop_client_ = this->create_client<std_srvs::srv::Trigger>("/emergency_stop");
        erase_emergency_stop_client_ = this->create_client<std_srvs::srv::Trigger>("/erase_emergency_stop");

        tcp_thread_ = std::thread(&RemoteControlNode::runTcpServer, this, port);
    }

    ~RemoteControlNode() {
        {
            std::lock_guard<std::mutex> lock(client_mutex_);
            if (client_sock_ >= 0) {
                close(client_sock_);
                client_sock_ = -1;
            }
        }
        if (tcp_thread_.joinable()) {
            tcp_thread_.join();
        }
    }

private:
    void runTcpServer(int port)
    {
        int server_fd;
        struct sockaddr_in address;
        int opt = 1;
        int addrlen = sizeof(address);
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
            return;
        }
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
            RCLCPP_ERROR(this->get_logger(), "Setsockopt failed");
            close(server_fd);
            return;
        }
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Bind failed");
            close(server_fd);
            return;
        }
        if (listen(server_fd, 1) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Listen failed");
            close(server_fd);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "TCP server listening on port %d", port);
        while (rclcpp::ok()) {
            int new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
            if (new_socket < 0) {
                RCLCPP_ERROR(this->get_logger(), "Accept failed");
                continue;
            }
            // 保存新的客户端连接（覆盖旧的）
            {
                std::lock_guard<std::mutex> lock(client_mutex_);
                if (client_sock_ >= 0) {
                    close(client_sock_);
                }
                client_sock_ = new_socket;
            }

            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &address.sin_addr, client_ip, INET_ADDRSTRLEN);
            RCLCPP_INFO(this->get_logger(), "New client connected from %s", client_ip);
            sendAllPoints();  // 新连接后立即发 0x01 全部点
            // 启动接收线程（每个连接一个线程，这里简化为直接处理）
            std::thread(&RemoteControlNode::handleClient, this, new_socket).detach();
        }
        close(server_fd);
    }

void handleClient(int sock)
{
    std::vector<uint8_t> buffer(1024);
    while (rclcpp::ok()) {
        ssize_t bytes = recv(sock, buffer.data(), buffer.size(), 0);
        if (bytes <= 0) {
            RCLCPP_WARN(this->get_logger(), "Client disconnected");
            // 清除客户端 socket（如果是当前连接）
            std::lock_guard<std::mutex> lock(client_mutex_);
            if (client_sock_ == sock) {
                client_sock_ = -1;
            }
            close(sock);
            break;
        }
        size_t index = 0;
        while (index + 7 <= static_cast<size_t>(bytes)) {
            if (buffer[index] != 0xF5) {
                index++;
                continue;
            }
            uint8_t func_code = buffer[index + 1];
            // 小端读取数据段长度
            uint16_t data_len = buffer[index + 2] | (static_cast<uint16_t>(buffer[index + 3]) << 8);
            size_t total_len = 1 + 1 + 2 + data_len + 2 + 1; // header+cmd+len+data+crc(2)+tail
            if (index + total_len > static_cast<size_t>(bytes)) {
                break; // 包不完整
            }
            if (buffer[index + total_len - 1] != 0x5F) {
                index++;
                continue;
            }

            // 解析数据段中的 1 字节指令类型
            if (func_code == 0x08) {
                if (data_len == 1 && index + 5 < static_cast<size_t>(bytes)) {
                    // 读取 1 字节指令类型
                    uint8_t instruction_type = buffer[index + 4];
                    if (instruction_type >= 0x01 && instruction_type <= 0x03) {
                        std::string cmd_str;
                        switch (instruction_type) {
                            case 0x01: cmd_str = "start"; 
                                // === 调用 start_all.sh ===
                                system("setsid /home/test/start_all.sh &");
                                break;
                            case 0x02: cmd_str = "pause"; 
                                // 调用 /emergency_stop 服务
                                if (emergency_stop_client_->wait_for_service(std::chrono::seconds(1))) {
                                    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                                    auto future = emergency_stop_client_->async_send_request(request);
                                    // 等待服务响应
                                    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
                                    auto response = future.get();
                                    if (response->success) {
                                        RCLCPP_INFO(this->get_logger(), "Emergency stop triggered successfully.");
                                    } else {
                                        RCLCPP_WARN(this->get_logger(), "Failed to trigger emergency stop.");
                                    }
                                } else {
                                    RCLCPP_WARN(this->get_logger(), "Emergency stop service is not available.");
                                }
                                break;
                            case 0x03: cmd_str = "stop";  
                                system("pkill -f start_all.sh");
                                break;
                            default: break;
                        }
                        auto msg = std_msgs::msg::String();
                        msg.data = cmd_str;
                        command_pub_->publish(msg);
                        RCLCPP_INFO(this->get_logger(), "Received command: %s (0x%02X)", cmd_str.c_str(), instruction_type);
                    } else if (instruction_type >= 0x04 && instruction_type <= 0x09) {
                        auto joy_msg = std::make_shared<sensor_msgs::msg::Joy>();
                        joy_msg->axes.resize(3, 0.0); // [lateral, forward, yaw]
                        joy_msg->buttons.clear();
                        const float speed = 0.5f;
                        switch (instruction_type) {
                            case 0x04: joy_msg->axes[1] =  speed; break; // 前进
                            case 0x05: joy_msg->axes[1] = -speed; break; // 后退
                            case 0x06: joy_msg->axes[0] =  speed; break; // 左移
                            case 0x07: joy_msg->axes[0] = -speed; break; // 右移
                            case 0x08: joy_msg->axes[2] = -speed; break; // 左转
                            case 0x09: joy_msg->axes[2] =  speed; break; // 右转
                            default: break;
                        }
                        joy_pub_->publish(*joy_msg);
                        RCLCPP_INFO(this->get_logger(), "Published Joy for instruction 0x%02X", instruction_type);
                    } else if (instruction_type == 0x10) {
                        // 调用 /erase_emergency_stop 服务
                        if (erase_emergency_stop_client_->wait_for_service(std::chrono::seconds(1))) {
                            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                            auto future = emergency_stop_client_->async_send_request(request);
                            rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
                            auto response = future.get();
                            if (response->success) {
                                RCLCPP_INFO(this->get_logger(), "Emergency stop triggered successfully.");
                            } else {
                                RCLCPP_WARN(this->get_logger(), "Failed to trigger emergency stop.");
                            }
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Emergency stop service is not available.");
                        }
                    } 
                    else {
                        RCLCPP_WARN(this->get_logger(), "Unknown instruction type: 0x%02X", instruction_type);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid data length for func_code=0x01: %u", data_len);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Unsupported function code: 0x%02X", func_code);
            }

            index += total_len;
        }
    }
}



    void sendPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud, uint8_t func_code)
    {
        RCLCPP_INFO(this->get_logger(), "Cloud step=%u fields=%ld", cloud->point_step, cloud->fields.size());
        std::lock_guard<std::mutex> lock(client_mutex_);
        if (client_sock_ < 0) return;

        size_t num_points = cloud->width * cloud->height;
        if (num_points == 0) return;
        try {
            // 使用 const iterator 读取 x/y/z
            sensor_msgs::PointCloud2ConstIterator<double> iter_x(*cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<double> iter_y(*cloud, "y");
            sensor_msgs::PointCloud2ConstIterator<double> iter_z(*cloud, "z");
            size_t data_len = num_points * 24;  // 3 * double
            size_t packet_len = 1 + 1 + 2 + data_len + 2 + 1;
            std::vector<uint8_t> packet(packet_len);
            size_t idx = 0;

            packet[idx++] = 0xF5;
            packet[idx++] = func_code;
            packet[idx++] = num_points & 0xFF;
            packet[idx++] = (num_points >> 8) & 0xFF;
            for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
                double x = *iter_x;
                double y = *iter_y;
                double z = *iter_z;
                std::memcpy(&packet[idx], &x, sizeof(double)); idx += sizeof(double);
                std::memcpy(&packet[idx], &y, sizeof(double)); idx += sizeof(double);
                std::memcpy(&packet[idx], &z, sizeof(double)); idx += sizeof(double);
            }
            packet[idx++] = 0;   // CRC low
            packet[idx++] = 0;   // CRC high
            packet[idx++] = 0x5F; // 包尾
            send(client_sock_, packet.data(), packet.size(), 0);
            RCLCPP_INFO(this->get_logger(), "Sent PointCloud: %zu points func=0x%02X", num_points, func_code);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send PointCloud: %s", e.what());
        }
    }

    void sendAllPoints()
    {
        std::lock_guard<std::mutex> lock(client_mutex_);
        if (client_sock_ < 0) return;
        if (all_points_.empty()) return;
        size_t num_points = all_points_.size();
        size_t data_len = num_points * 24; // 3 * double
        size_t packet_len = 1 + 1 + 2 + data_len + 2 + 1;

        std::vector<uint8_t> packet(packet_len);
        size_t idx = 0;
        packet[idx++] = 0xF5;      // header
        packet[idx++] = 0x01;      // 功能码 = 0x01 (ALL POINTS)
        packet[idx++] = num_points & 0xFF;         
        packet[idx++] = (num_points >> 8) & 0xFF;

        for (const auto& p : all_points_) {
            std::memcpy(&packet[idx], &p.x, sizeof(double)); idx += sizeof(double);
            std::memcpy(&packet[idx], &p.y, sizeof(double)); idx += sizeof(double);
            std::memcpy(&packet[idx], &p.z, sizeof(double)); idx += sizeof(double);
        }
        packet[idx++] = 0x00; // CRC low
        packet[idx++] = 0x00; // CRC high
        packet[idx++] = 0x5F; // tail
        send(client_sock_, packet.data(), packet.size(), 0);
         logPacket("TX ALL_POINTS", packet.data(), packet.size(), this->get_logger());
        RCLCPP_INFO(this->get_logger(), "Sent ALL points (%zu points) with func=0x01", num_points);
    }

    void sendProgress(uint8_t progress)
    {
        std::lock_guard<std::mutex> lock(client_mutex_);
        if (client_sock_ < 0) return;
        uint8_t packet[] = {
            0xF5,
            0x06,            // 功能码
            progress,
            0x00, 0x00,      // CRC (ignored)
            0x5F
        };
        send(client_sock_, packet, sizeof(packet), 0);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "Sent progress: %u", progress);
         logPacket("TX PROGRESS", packet, sizeof(packet), this->get_logger());
    }

    void sendPosition(double x, double y, double z) {
        std::lock_guard<std::mutex> lock(client_mutex_);
        if (client_sock_ < 0) return;
        // 构造包
        uint8_t packet[1 + 1 + 3*8 + 2 + 1]; // 头+功能码+3*double+CRC+尾
        size_t idx = 0;
        packet[idx++] = 0xF5;               // 包头
        packet[idx++] = 0x07;               // 功能码

        // 写入 X, Y, Z（小端，直接 memcpy）
        std::memcpy(&packet[idx], &x, sizeof(double)); idx += sizeof(double);
        std::memcpy(&packet[idx], &y, sizeof(double)); idx += sizeof(double);
        std::memcpy(&packet[idx], &z, sizeof(double)); idx += sizeof(double);

        packet[idx++] = 0x00;               // CRC low
        packet[idx++] = 0x00;               // CRC high
        packet[idx++] = 0x5F;               // 包尾

        send(client_sock_, packet, sizeof(packet), 0);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 4000,
         "Sent position : (%.3f, %.3f, %.3f)", x, y, z);
        // --- 打印包内容 ---
        std::ostringstream oss;
        oss << "Sending packet bytes: ";
        for (size_t i = 0; i < sizeof(packet); ++i) {
            oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') 
                << static_cast<int>(packet[i]) << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }
    void sendMaxTemperature(const std::vector<float>& temps)
    {
        if (temps.empty()) return;
        float max_temp_f = *std::max_element(temps.begin(), temps.end());
        // 限制为 0~255
        uint8_t temp_byte = static_cast<uint8_t>(std::min(std::max(max_temp_f, 0.0f), 255.0f));

        std::lock_guard<std::mutex> lock(client_mutex_);
        if (client_sock_ < 0) return;

        uint8_t packet[6];
        packet[0] = 0xF5;      // 包头
        packet[1] = 0x05;      // 功能码
        packet[2] = temp_byte; // 温度值
        packet[3] = 0x00;      // CRC低字节（这里暂用0）
        packet[4] = 0x00;      // CRC高字节（这里暂用0）
        packet[5] = 0x5F;      // 包尾

        send(client_sock_, packet, sizeof(packet), 0);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "Sent max temperature: %u", temp_byte);
    }

    void sendPath(const nav_msgs::msg::Path::SharedPtr path_msg)
    {
        std::lock_guard<std::mutex> lock(client_mutex_);
        if (client_sock_ < 0 || path_msg->poses.empty()) return;

        // 限制点数在 0-255 之间
        uint8_t N = static_cast<uint8_t>(path_msg->poses.size());
        if (path_msg->poses.size() > 255) {
             RCLCPP_WARN(this->get_logger(), "Path has more than 255 points (%zu), truncating to 255.", path_msg->poses.size());
             N = 255;
        }
        // 修正长度计算
        // 结构: Header(1) + Func(1) + Count(1) + Points(N*24) + CRC(2) + Tail(1)
        // data_len 在这里仅指 "Count + Points" 或者我们直接算总长，避免混淆
        size_t points_data_len = N * 24; 
        size_t packet_len = 1 + 1 + 1 + points_data_len + 2 + 1; 
        
        std::vector<uint8_t> packet(packet_len, 0);

        size_t idx = 0;
        packet[idx++] = 0xF5; // 包头
        packet[idx++] = 0x04; // 功能码
        packet[idx++] = N;    // 点数量 (这是数据部分的第一个字节)
        // 写入轨迹点
        for (int i = 0; i < N; ++i) {
            const auto &pose_stamped = path_msg->poses[i];
            double x = pose_stamped.pose.position.x;
            double y = pose_stamped.pose.position.y;
            double z = pose_stamped.pose.position.z;
            
            std::memcpy(&packet[idx], &x, sizeof(double)); idx += sizeof(double);
            std::memcpy(&packet[idx], &y, sizeof(double)); idx += sizeof(double);
            std::memcpy(&packet[idx], &z, sizeof(double)); idx += sizeof(double);
        }

        packet[idx++] = 0x00; // CRC low
        packet[idx++] = 0x00; // CRC high
        packet[idx++] = 0x5F; // 包尾
        // 调试用：确认 idx 是否等于 packet_len
        if (idx != packet_len) {
            RCLCPP_ERROR(this->get_logger(), "Packet length mismatch! Calculated: %zu, Written: %zu", packet_len, idx);
        }

        send(client_sock_, packet.data(), packet.size(), 0);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "Sent Path with %u points", N);
        
        // 打印包内容
        logPacket("TX PATH", packet.data(), packet.size(), this->get_logger());
    }

    std::vector<Point> parsePoints(const std::string& filename) {
        std::vector<Point> points;
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开文件: %s", filename.c_str());
            return points;
        }

        std::string line; int line_num = 0;
        while (std::getline(file, line)) {
            line_num++;
            if (line.empty() || line[0] == '#') continue;

            std::stringstream ss(line);
            std::string id, x_str, y_str, z_str;
            if (std::getline(ss, id, ',') &&
                std::getline(ss, x_str, ',') &&
                std::getline(ss, y_str, ',') &&
                std::getline(ss, z_str, ',')) {
                try {
                    points.emplace_back(id,
                        std::stod(x_str),
                        std::stod(y_str),
                        std::stod(z_str));
                    RCLCPP_INFO(this->get_logger(), 
                    "Parsed point: id='%s', x=%.9f, y=%.9f, z=%.9f", 
                    points.back().id.c_str(),
                    points.back().x,
                    points.back().y,
                    points.back().z);
                } catch (...) {
                    RCLCPP_WARN(this->get_logger(), "解析错误，第 %d 行", line_num);
                }
            }
        }
        file.close();
        return points;
    }
    void logPacket(const std::string &name,
               const uint8_t *data,
               size_t len,
               rclcpp::Logger logger)
{
    std::ostringstream oss;
    oss << name << " [len=" << len << "] : ";
    for (size_t i = 0; i < len; ++i) {
        oss << std::hex << std::uppercase
            << std::setw(2) << std::setfill('0')
            << static_cast<int>(data[i]) << " ";
    }
    RCLCPP_INFO(logger, "%s", oss.str().c_str());
}


    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr progress_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motors_temp_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr acquired_points_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr unacquired_points_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_stop_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr erase_emergency_stop_client_;

    std::vector<Point> all_points_; // 存储所有原始点
    // TCP
    std::thread tcp_thread_;
    int client_sock_ = -1;          // 当前客户端 socket
    std::mutex client_mutex_;       // 保护 client_sock_
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}