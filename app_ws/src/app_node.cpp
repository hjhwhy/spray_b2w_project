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
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/path.hpp> 
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <fstream>
#include <string>
#include <unordered_map>
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
        this->declare_parameter<int>("listen_port", 9002);
        int port = this->get_parameter("listen_port").as_int();
        const char *home_env = std::getenv("HOME");
        std::string home = home_env ? home_env : "";
        std::string file_path = home + "/gnss_waypoints.txt";
        all_points_ = parsePoints(file_path);
        if (all_points_.empty()) {
            RCLCPP_WARN(this->get_logger(), "未能读取到任何有效点位，将继续启动 TCP 服务。");
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
        stop_requested_.store(true);
        {
            std::lock_guard<std::mutex> lock(client_mutex_);
            closeClientSocketLocked();
        }
        if (server_fd_ >= 0) {
            close(server_fd_);
            server_fd_ = -1;
        }
        if (client_thread_.joinable()) {
            client_thread_.join();
        }
        if (tcp_thread_.joinable()) {
            tcp_thread_.join();
        }
    }

private:
    void runTcpServer(int port)
    {
        struct sockaddr_in address;
        int opt = 1;
        socklen_t addrlen = sizeof(address);
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
            return;
        }
        if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set SO_REUSEADDR");
            close(server_fd_);
            server_fd_ = -1;
            return;
        }
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(server_fd_, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Bind failed");
            close(server_fd_);
            server_fd_ = -1;
            return;
        }
        if (listen(server_fd_, 1) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Listen failed");
            close(server_fd_);
            server_fd_ = -1;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "TCP server listening on port %d", port);
        while (rclcpp::ok() && !stop_requested_.load()) {
            int new_socket = accept(server_fd_, reinterpret_cast<struct sockaddr *>(&address), &addrlen);
            if (new_socket < 0) {
                if (stop_requested_.load()) {
                    break;
                }
                if (errno == EINTR) {
                    continue;
                }
                RCLCPP_ERROR(this->get_logger(), "Accept failed: %s", std::strerror(errno));
                continue;
            }

            if (client_thread_.joinable()) {
                {
                    std::lock_guard<std::mutex> lock(client_mutex_);
                    closeClientSocketLocked();
                }
                client_thread_.join();
            }

            {
                std::lock_guard<std::mutex> lock(client_mutex_);
                client_sock_ = new_socket;
            }

            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &address.sin_addr, client_ip, INET_ADDRSTRLEN);
            RCLCPP_INFO(this->get_logger(), "New client connected from %s", client_ip);
            sendAllPoints();
            client_thread_ = std::thread(&RemoteControlNode::handleClient, this, new_socket);
        }
        if (server_fd_ >= 0) {
            close(server_fd_);
            server_fd_ = -1;
        }
    }

    void handleClient(int sock)
    {
        std::array<uint8_t, 1024> recv_buffer{};
        std::vector<uint8_t> stream_buffer;

        while (rclcpp::ok() && !stop_requested_.load()) {
            ssize_t bytes = recv(sock, recv_buffer.data(), recv_buffer.size(), 0);
            if (bytes == 0) {
                RCLCPP_INFO(this->get_logger(), "Client disconnected");
                break;
            }
            if (bytes < 0) {
                if (stop_requested_.load()) {
                    break;
                }
                if (errno == EINTR) {
                    continue;
                }
                RCLCPP_WARN(this->get_logger(), "recv() failed: %s", std::strerror(errno));
                break;
            }

            stream_buffer.insert(
                stream_buffer.end(),
                recv_buffer.begin(),
                recv_buffer.begin() + bytes);

            while (parseNextPacket(stream_buffer)) {
            }
        }
        {
            std::lock_guard<std::mutex> lock(client_mutex_);
            if (client_sock_ == sock) {
                closeClientSocketLocked();
            } else if (sock >= 0) {
                close(sock);
            }
        }
    }

    bool parseNextPacket(std::vector<uint8_t> &buffer)
    {
        auto header_it = std::find(buffer.begin(), buffer.end(), static_cast<uint8_t>(0xF5));
        if (header_it == buffer.end()) {
            buffer.clear();
            return false;
        }
        if (header_it != buffer.begin()) {
            buffer.erase(buffer.begin(), header_it);
        }

        if (buffer.size() < 2) {
            return false;
        }

        const uint8_t func_code = buffer[1];
        size_t total_len = 0;

        if (func_code == 0x08) {
            if (buffer.size() < 7) {
                return false;
            }
            const uint16_t data_len = buffer[2] | (static_cast<uint16_t>(buffer[3]) << 8);
            if (data_len > 256) {
                RCLCPP_WARN(this->get_logger(), "Discarding invalid 0x08 packet with data_len=%u", data_len);
                buffer.erase(buffer.begin());
                return true;
            }
            total_len = 1 + 1 + 2 + data_len + 2 + 1;
            if (buffer.size() < total_len) {
                return false;
            }
            if (buffer[total_len - 1] != 0x5F) {
                RCLCPP_WARN(this->get_logger(), "Invalid tail for 0x08 packet");
                buffer.erase(buffer.begin());
                return true;
            }

            logPacket("RX COMMAND", buffer.data(), total_len, this->get_logger());
            handleCommandPacket(buffer.data() + 4, data_len);
            buffer.erase(buffer.begin(), buffer.begin() + total_len);
            return true;
        }

        if (func_code == 0x09) {
            if (buffer.size() < 7) {
                return false;
            }
            const uint8_t region_count = buffer[2];
            total_len = 1 + 1 + 1 + static_cast<size_t>(region_count) * 64 + 1 + 2 + 1;
            if (buffer.size() < total_len) {
                return false;
            }
            if (buffer[total_len - 1] != 0x5F) {
                RCLCPP_WARN(this->get_logger(), "Invalid tail for 0x09 packet");
                buffer.erase(buffer.begin());
                return true;
            }

            logPacket("RX REGION", buffer.data(), total_len, this->get_logger());
            handleRegionPacket(buffer.data(), total_len);
            buffer.erase(buffer.begin(), buffer.begin() + total_len);
            return true;
        }

        RCLCPP_WARN(this->get_logger(), "Unsupported function code: 0x%02X", func_code);
        buffer.erase(buffer.begin());
        return true;
    }

    void handleCommandPacket(const uint8_t *payload, uint16_t data_len)
    {
        if (data_len < 1) {
            RCLCPP_WARN(this->get_logger(), "Invalid data length for func_code=0x08: %u", data_len);
            return;
        }

        const uint8_t instruction_type = payload[0];
        RCLCPP_INFO(this->get_logger(), "Decoded instruction type: 0x%02X", instruction_type);
        if (data_len > 1) {
            RCLCPP_INFO(this->get_logger(), "Command 0x%02X carries %u extra payload bytes; they will be ignored",
                        instruction_type, static_cast<unsigned>(data_len - 1));
        }

        if (instruction_type >= 0x01 && instruction_type <= 0x03) {
            std::string cmd_str;
            switch (instruction_type) {
                case 0x01:
                    cmd_str = "start";
                {
                    const int rc = system(
                        "bash -c '"
                        "if [ -f /tmp/start_all.pid ]; then "
                        "pid=$(cat /tmp/start_all.pid); "
                        "if [ -n \"$pid\" ] && kill -0 \"$pid\" 2>/dev/null; then "
                        "echo start_all already running with pid \"$pid\"; "
                        "exit 0; "
                        "else "
                        "rm -f /tmp/start_all.pid; "
                        "fi; "
                        "fi; "
                        "setsid /home/test/start_all.sh &'");
                    RCLCPP_INFO(this->get_logger(), "Start command system() returned %d", rc);
                    break;
                }
                case 0x02:
                    cmd_str = "pause";
                    RCLCPP_INFO(this->get_logger(), "Received command: pause (0x02)");
                    if (requestTrigger(emergency_stop_client_, "Emergency stop")) {
                        RCLCPP_INFO(this->get_logger(), "Pause request accepted.");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Pause rejected.");
                    }
                    break;
                case 0x03:
                    cmd_str = "stop";
                {
                    const int rc = system(
                        "bash -c '"
                        "if [ -f /tmp/start_all.pid ]; then "
                        "pid=$(cat /tmp/start_all.pid); "
                        "if [ -n \"$pid\" ] && kill -0 \"$pid\" 2>/dev/null; then "
                        "kill -TERM -\"$pid\"; "
                        "else "
                        "echo start_all not running; "
                        "fi; "
                        "else "
                        "echo /tmp/start_all.pid not found; "
                        "fi'");
                    RCLCPP_INFO(this->get_logger(), "Stop command system() returned %d", rc);
                    break;
                }
                default:
                    break;
            }
            auto msg = std_msgs::msg::String();
            msg.data = cmd_str;
            command_pub_->publish(msg);
            if (instruction_type != 0x02) {
                RCLCPP_INFO(this->get_logger(), "Received command: %s (0x%02X)", cmd_str.c_str(), instruction_type);
            }
            return;
        }

        if (instruction_type >= 0x04 && instruction_type <= 0x09) {
            sensor_msgs::msg::Joy joy_msg;
            joy_msg.axes.resize(3, 0.0F);
            const float direction = 1.0F;
            switch (instruction_type) {
                case 0x04: joy_msg.axes[1] =  direction; break;
                case 0x05: joy_msg.axes[1] = -direction; break;
                case 0x06: joy_msg.axes[0] =  direction; break;
                case 0x07: joy_msg.axes[0] = -direction; break;
                case 0x08: joy_msg.axes[2] =  direction; break;
                case 0x09: joy_msg.axes[2] = -direction; break;
                default: break;
            }
            joy_pub_->publish(joy_msg);
            RCLCPP_INFO(this->get_logger(), "Published Joy for instruction 0x%02X", instruction_type);
            return;
        }

        if (instruction_type == 0x10) {
            RCLCPP_INFO(this->get_logger(), "Received command: restart/resume (0x10)");
            if (requestTrigger(erase_emergency_stop_client_, "Erase emergency stop")) {
                RCLCPP_INFO(this->get_logger(), "Restart/resume request accepted.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Restart/resume rejected.");
            }
            return;
        }

        RCLCPP_WARN(this->get_logger(), "Unknown instruction type: 0x%02X", instruction_type);
    }

    void handleRegionPacket(const uint8_t *packet, size_t packet_len)
    {
        const uint8_t region_count = packet[2];
        const uint8_t instruction_type = packet[3 + static_cast<size_t>(region_count) * 64];
        if (instruction_type == 0x11) {
            RCLCPP_INFO(this->get_logger(), "Received priority region packet with %u regions", region_count);
        } else if (instruction_type == 0x12) {
            RCLCPP_INFO(this->get_logger(), "Received avoidance region packet with %u regions", region_count);
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "Received 0x09 packet with unknown instruction 0x%02X (len=%zu)",
                        instruction_type, packet_len);
        }
    }

    bool requestTrigger(
        const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
        const std::string &action_name)
    {
        if ((action_name == "Emergency stop" || action_name == "Erase emergency stop") &&
            !canAttemptStartAllControl()) {
            RCLCPP_WARN(
                this->get_logger(),
                "%s rejected: start_all 主流程尚未进入可控状态，请等待 start_all.ready 至少进入 partial 状态。",
                action_name.c_str());
            return false;
        }

        constexpr int kMaxAttempts = 5;
        bool service_ready = false;
        for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
            if (client->wait_for_service(std::chrono::seconds(1))) {
                service_ready = true;
                break;
            }
            RCLCPP_WARN(this->get_logger(),
                        "%s service is not available yet (attempt %d/%d).",
                        action_name.c_str(), attempt, kMaxAttempts);
        }

        if (!service_ready) {
            RCLCPP_WARN(this->get_logger(), "%s service is not available.", action_name.c_str());
            return false;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        client->async_send_request(
            request,
            [this, action_name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                const auto &response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "%s succeeded.", action_name.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "%s failed: %s",
                                action_name.c_str(), response->message.c_str());
                }
            });
        return true;
    }

    bool canAttemptStartAllControl() const
    {
        std::ifstream ready_file("/tmp/start_all.ready");
        if (!ready_file.is_open()) {
            RCLCPP_WARN(this->get_logger(), "start_all readiness file /tmp/start_all.ready does not exist.");
            return false;
        }

        std::unordered_map<std::string, std::string> kv;
        std::string line;
        while (std::getline(ready_file, line)) {
            const auto pos = line.find('=');
            if (pos == std::string::npos) {
                continue;
            }
            kv[line.substr(0, pos)] = line.substr(pos + 1);
        }

        const auto state_it = kv.find("state");
        const auto service_it = kv.find("service_ready");
        const auto probe_it = kv.find("probe_ok");
        if (state_it == kv.end() || service_it == kv.end() || probe_it == kv.end()) {
            RCLCPP_WARN(this->get_logger(), "start_all readiness file is incomplete.");
            return false;
        }

        if (state_it->second == "ready" && service_it->second == "1" && probe_it->second == "1") {
            return true;
        }

        if (state_it->second == "partial" && service_it->second == "1") {
            RCLCPP_WARN(
                this->get_logger(),
                "start_all readiness is partial (probe_ok=%s); attempting service call anyway.",
                probe_it->second.c_str());
            return true;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "start_all readiness state is '%s' (service_ready=%s, probe_ok=%s).",
            state_it->second.c_str(),
            service_it->second.c_str(),
            probe_it->second.c_str());
        return false;
    }

    void closeClientSocketLocked()
    {
        if (client_sock_ < 0) {
            return;
        }
        shutdown(client_sock_, SHUT_RDWR);
        close(client_sock_);
        client_sock_ = -1;
    }

    bool sendAllBytesLocked(int sock, const uint8_t *data, size_t len)
    {
        size_t sent_total = 0;
        while (sent_total < len) {
#ifdef MSG_NOSIGNAL
            constexpr int flags = MSG_NOSIGNAL;
#else
            constexpr int flags = 0;
#endif
            const ssize_t sent = send(sock, data + sent_total, len - sent_total, flags);
            if (sent < 0) {
                if (errno == EINTR) {
                    continue;
                }
                RCLCPP_WARN(this->get_logger(), "send() failed: %s", std::strerror(errno));
                return false;
            }
            if (sent == 0) {
                RCLCPP_WARN(this->get_logger(), "send() returned 0");
                return false;
            }
            sent_total += static_cast<size_t>(sent);
        }
        return true;
    }

    bool sendPacket(const uint8_t *data, size_t len)
    {
        std::lock_guard<std::mutex> lock(client_mutex_);
        if (client_sock_ < 0) {
            return false;
        }
        if (!sendAllBytesLocked(client_sock_, data, len)) {
            closeClientSocketLocked();
            return false;
        }
        return true;
    }

    bool sendPacket(const std::vector<uint8_t> &packet)
    {
        return sendPacket(packet.data(), packet.size());
    }

    void sendPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud, uint8_t func_code)
    {
        RCLCPP_INFO(this->get_logger(), "Cloud step=%u fields=%ld", cloud->point_step, cloud->fields.size());

        size_t num_points = cloud->width * cloud->height;
        if (num_points == 0) return;
        if (num_points > std::numeric_limits<uint16_t>::max()) {
            RCLCPP_WARN(this->get_logger(), "PointCloud has %zu points, truncating to %u",
                        num_points, std::numeric_limits<uint16_t>::max());
            num_points = std::numeric_limits<uint16_t>::max();
        }
        try {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
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
            sendPacket(packet);
            RCLCPP_INFO(this->get_logger(), "Sent PointCloud: %zu points func=0x%02X", num_points, func_code);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send PointCloud: %s", e.what());
        }
    }

    void sendAllPoints()
    {
        if (all_points_.empty()) return;
        size_t num_points = all_points_.size();
        if (num_points > std::numeric_limits<uint16_t>::max()) {
            RCLCPP_WARN(this->get_logger(), "All points count %zu exceeds protocol limit, truncating to %u",
                        num_points, std::numeric_limits<uint16_t>::max());
            num_points = std::numeric_limits<uint16_t>::max();
        }
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
        sendPacket(packet);
        logPacket("TX ALL_POINTS", packet.data(), packet.size(), this->get_logger());
        RCLCPP_INFO(this->get_logger(), "Sent ALL points (%zu points) with func=0x01", num_points);
    }

    void sendProgress(uint8_t progress)
    {
        uint8_t packet[] = {
            0xF5,
            0x06,            // 功能码
            0x01, 0x00,      // 数据段长度 = 1
            progress,
            0x00, 0x00,      // CRC (ignored)
            0x5F
        };
        sendPacket(packet, sizeof(packet));
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "Sent progress: %u", progress);
        logPacket("TX PROGRESS", packet, sizeof(packet), this->get_logger());
    }

    void sendPosition(double x, double y, double z) {
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

        sendPacket(packet, sizeof(packet));
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
         "Sent position : (%.3f, %.3f, %.3f)", x, y, z);
        logPacket("TX POSITION", packet, sizeof(packet), this->get_logger(), 5000);
    }
    void sendMaxTemperature(const std::vector<float>& temps)
    {
        if (temps.empty()) return;
        float max_temp_f = *std::max_element(temps.begin(), temps.end());
        // 限制为 0~255
        uint8_t temp_byte = static_cast<uint8_t>(std::min(std::max(max_temp_f, 0.0f), 255.0f));

        uint8_t packet[6];
        packet[0] = 0xF5;      // 包头
        packet[1] = 0x05;      // 功能码
        packet[2] = temp_byte; // 温度值
        packet[3] = 0x00;      // CRC低字节（这里暂用0）
        packet[4] = 0x00;      // CRC高字节（这里暂用0）
        packet[5] = 0x5F;      // 包尾

        sendPacket(packet, sizeof(packet));
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "Sent max temperature: %u", temp_byte);
    }

    void sendPath(const nav_msgs::msg::Path::SharedPtr path_msg)
    {
        if (path_msg->poses.empty()) return;

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

        sendPacket(packet);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "Sent Path with %u points", N);
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
               rclcpp::Logger logger,
               uint64_t throttle_ms = 0)
{
    std::ostringstream oss;
    oss << name << " [len=" << len << "] : ";
    for (size_t i = 0; i < len; ++i) {
        oss << std::hex << std::uppercase
            << std::setw(2) << std::setfill('0')
            << static_cast<int>(data[i]) << " ";
    }
    if (throttle_ms > 0) {
        RCLCPP_INFO_THROTTLE(logger, *this->get_clock(), throttle_ms, "%s", oss.str().c_str());
    } else {
        RCLCPP_INFO(logger, "%s", oss.str().c_str());
    }
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
    std::thread client_thread_;
    std::atomic<bool> stop_requested_{false};
    int server_fd_ = -1;
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
