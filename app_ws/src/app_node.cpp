#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/byte.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joy.hpp> 
#include <nav_msgs/msg/odometry.hpp>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <csignal>
#include <cstdio>
#include <iomanip>
#include <iterator>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>
#include <sys/stat.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/path.hpp> 
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <std_srvs/srv/trigger.hpp>
#include "z1_arm_controller_cpp/srv/move_arm.hpp"

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
        this->declare_parameter<double>("client_disconnect_auto_pause_seconds", 8.0);
        this->declare_parameter<double>("heartbeat_timeout_seconds", 3.5);
        this->declare_parameter<bool>("heartbeat_required_after_control", true);
        this->declare_parameter<double>("arm_reset_timeout_seconds", 15.0);
        this->declare_parameter<double>("position_upload_hz", 5.0);
        this->declare_parameter<double>("position_upload_min_distance", 0.02);
        int port = this->get_parameter("listen_port").as_int();
        client_disconnect_auto_pause_seconds_ =
            this->get_parameter("client_disconnect_auto_pause_seconds").as_double();
        heartbeat_timeout_seconds_ = this->get_parameter("heartbeat_timeout_seconds").as_double();
        heartbeat_required_after_control_ = this->get_parameter("heartbeat_required_after_control").as_bool();
        arm_reset_timeout_seconds_ = this->get_parameter("arm_reset_timeout_seconds").as_double();
        position_upload_hz_ = this->get_parameter("position_upload_hz").as_double();
        position_upload_min_distance_ = this->get_parameter("position_upload_min_distance").as_double();
        if (position_upload_hz_ < 0.0) {
            RCLCPP_WARN(this->get_logger(), "position_upload_hz=%.3f is negative; disabling time-based position upload.", position_upload_hz_);
            position_upload_hz_ = 0.0;
        }
        if (position_upload_min_distance_ < 0.0) {
            RCLCPP_WARN(this->get_logger(), "position_upload_min_distance=%.3f is negative; clamping to 0.0.", position_upload_min_distance_);
            position_upload_min_distance_ = 0.0;
        }
        if (arm_reset_timeout_seconds_ < 1.0) {
            RCLCPP_WARN(
                this->get_logger(),
                "arm_reset_timeout_seconds=%.3f is too small; clamping to 1.0s.",
                arm_reset_timeout_seconds_);
            arm_reset_timeout_seconds_ = 1.0;
        }
        if (client_disconnect_auto_pause_seconds_ > 0.0 && client_disconnect_auto_pause_seconds_ < 3.0) {
            RCLCPP_WARN(
                this->get_logger(),
                "client_disconnect_auto_pause_seconds=%.3f is below the 3s WiFi jitter grace period; clamping to 3.0s.",
                client_disconnect_auto_pause_seconds_);
            client_disconnect_auto_pause_seconds_ = 3.0;
        }
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
                {
                    std::lock_guard<std::mutex> lock(upload_state_mutex_);
                    latest_progress_ = msg->data;
                }
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
                {
                    std::lock_guard<std::mutex> lock(upload_state_mutex_);
                    latest_path_ = msg;
                }
                sendPath(msg);
        });
        acquired_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/acquired_points", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received /acquired_points");
            const size_t num_points = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
            if (num_points == 0 || hasFloat64XYZFields(*msg)) {
                std::lock_guard<std::mutex> lock(upload_state_mutex_);
                latest_acquired_points_ = msg;
            }
            sendPointCloud(msg, 0x02);
        });
        unacquired_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/unacquired_points", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received /unacquired_points");
            const size_t num_points = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
            if (num_points == 0 || hasFloat64XYZFields(*msg)) {
                std::lock_guard<std::mutex> lock(upload_state_mutex_);
                latest_unacquired_points_ = msg;
            }
            sendPointCloud(msg, 0x03);
        });
        emergency_stop_client_ = this->create_client<std_srvs::srv::Trigger>("/emergency_stop");
        erase_emergency_stop_client_ = this->create_client<std_srvs::srv::Trigger>("/erase_emergency_stop");
        z1_reset_arm_client_ = this->create_client<z1_arm_controller_cpp::srv::MoveArm>("/z1_reset_arm");
        heartbeat_watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&RemoteControlNode::checkHeartbeatWatchdog, this));

        tcp_thread_ = std::thread(&RemoteControlNode::runTcpServer, this, port);
    }

    ~RemoteControlNode() {
        stop_requested_.store(true);
        clearSafetyTimers();
        cancelDisconnectAutoPause("remote_control_node shutting down; cancel disconnect auto-pause");
        {
            std::lock_guard<std::mutex> lock(client_mutex_);
            shutdownClientSocketLocked();
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
            configureClientSocketTimeouts(new_socket);

            if (client_thread_.joinable()) {
                {
                    std::lock_guard<std::mutex> lock(client_mutex_);
                    shutdownClientSocketLocked();
                }
                client_thread_.join();
            }

            {
                std::lock_guard<std::mutex> lock(client_mutex_);
                client_sock_ = new_socket;
            }
            cancelDisconnectAutoPause("APP client reconnect/cancel disconnect auto-pause");
            resetHeartbeatForNewClient("APP client connected");

            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &address.sin_addr, client_ip, INET_ADDRSTRLEN);
            RCLCPP_INFO(this->get_logger(), "New client connected from %s", client_ip);
            client_thread_ = std::thread(&RemoteControlNode::handleClient, this, new_socket);
            sendAllPoints();
            replayLatestUploadStateToClient();
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
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
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
        bool active_client_disconnected = false;
        bool should_close_sock = sock >= 0;
        {
            std::lock_guard<std::mutex> lock(client_mutex_);
            if (client_sock_ == sock) {
                shutdownClientSocketLocked();
                active_client_disconnected = true;
            }
        }
        if (should_close_sock) {
            close(sock);
        }
        if (active_client_disconnected && !stop_requested_.load()) {
            scheduleDisconnectAutoPause();
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

        if (func_code == 0xFF) {
            if (buffer.size() < 7) {
                return false;
            }
            const uint16_t data_len = buffer[2] | (static_cast<uint16_t>(buffer[3]) << 8);
            if (data_len != 1) {
                RCLCPP_WARN(this->get_logger(), "Discarding invalid heartbeat packet with data_len=%u", data_len);
                buffer.erase(buffer.begin());
                return true;
            }
            total_len = 1 + 1 + 2 + data_len + 2 + 1;
            if (buffer.size() < total_len) {
                return false;
            }
            if (buffer[total_len - 1] != 0x5F) {
                RCLCPP_WARN(this->get_logger(), "Invalid tail for heartbeat packet");
                buffer.erase(buffer.begin());
                return true;
            }
            if (buffer[4] != 0xFF) {
                RCLCPP_WARN(this->get_logger(), "Invalid heartbeat value: 0x%02X", buffer[4]);
                buffer.erase(buffer.begin(), buffer.begin() + total_len);
                return true;
            }

            handleHeartbeatPacket();
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
        markClientActivity();
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
                    StartAllTarget target;
                    if (findStartAllTarget(target)) {
                        const auto mode = readStartAllMode().value_or("unknown");
                        RCLCPP_WARN(this->get_logger(),
                            "Start rejected: start_all already running, mode=%s, pid=%ld, pgid=%ld.",
                            mode.c_str(), static_cast<long>(target.pid), static_cast<long>(target.pgid));
                        return;
                    }
                    const int rc = system("bash -c 'setsid /home/test/start_all.sh --mode spray &'");
                    RCLCPP_INFO(this->get_logger(), "Start command system() returned %d", rc);
                    markAppControlSessionActive("start command");
                    break;
                }
                case 0x02:
                    cmd_str = "pause";
                    if (readStartAllMode().value_or("") == "dock_return") {
                        RCLCPP_WARN(this->get_logger(), "Pause rejected: dock_return mode only supports stop.");
                        return;
                    }
                    handlePauseCommand();
                    break;
                case 0x03:
                    cmd_str = "stop";
                {
                    handleStopCommand();
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
            markAppControlSessionActive("motion command");
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

        if (instruction_type == 0x0A || instruction_type == 0x0B) {
            markAppControlSessionActive("posture command");
            sensor_msgs::msg::Joy joy_msg;
            joy_msg.axes.resize(3, 0.0F);
            joy_msg.buttons.resize(2, 0);
            switch (instruction_type) {
                case 0x0A: joy_msg.buttons[0] = 1; break;
                case 0x0B: joy_msg.buttons[1] = 1; break;
                default: break;
            }
            joy_pub_->publish(joy_msg);
            RCLCPP_INFO(this->get_logger(), "Published Joy action for instruction 0x%02X", instruction_type);
            return;
        }

        if (instruction_type == 0x0C) {
            RCLCPP_INFO(this->get_logger(), "Received command: return to charging dock (0x0C)");

            StartAllTarget target;
            if (findStartAllTarget(target)) {
                const auto mode = readStartAllMode().value_or("unknown");
                RCLCPP_WARN(this->get_logger(),
                    "Dock return rejected: start_all already running, mode=%s, pid=%ld, pgid=%ld. Stop current task first.",
                    mode.c_str(), static_cast<long>(target.pid), static_cast<long>(target.pgid));
                return;
            }

            constexpr const char *charging_file = "/home/test/gnss_charging.txt";
            struct stat charging_stat {};
            if (stat(charging_file, &charging_stat) != 0 || !S_ISREG(charging_stat.st_mode) || charging_stat.st_size <= 0 ||
                access(charging_file, R_OK) != 0) {
                RCLCPP_ERROR(this->get_logger(),
                    "Dock return rejected: %s is not a readable non-empty regular file.", charging_file);
                return;
            }

            const int rc = system("bash -c 'setsid /home/test/start_all.sh --mode dock_return &'");
            RCLCPP_INFO(this->get_logger(), "Dock return start_all system() returned %d", rc);
            markAppControlSessionActive("dock return command");
            return;
        }

        if (instruction_type == 0x10) {
            RCLCPP_INFO(this->get_logger(), "Received command: restart/resume (0x10)");
            if (readStartAllMode().value_or("") == "dock_return") {
                RCLCPP_WARN(this->get_logger(), "Restart rejected: dock_return mode only supports stop.");
                return;
            }
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
        markClientActivity();
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

    void markClientActivity()
    {
        const auto now = this->now();
        std::lock_guard<std::mutex> lock(heartbeat_mutex_);
        last_client_activity_time_ = now;
    }

    void handleHeartbeatPacket()
    {
        const auto now = this->now();
        bool log_state_change = false;
        const char *state_change_reason = nullptr;
        bool active_after_update = false;
        {
            std::lock_guard<std::mutex> lock(heartbeat_mutex_);
            const bool had_heartbeat = has_heartbeat_;
            const bool had_timeout_pending = heartbeat_timeout_pending_ && heartbeat_timeout_pending_->load();
            last_client_activity_time_ = now;
            last_heartbeat_time_ = now;
            has_heartbeat_ = true;
            active_after_update = app_control_session_active_;
            if (heartbeat_timeout_pending_) {
                heartbeat_timeout_pending_->store(false);
                heartbeat_timeout_pending_.reset();
            }
            if (!had_heartbeat) {
                log_state_change = true;
                state_change_reason = "online";
            } else if (had_timeout_pending) {
                log_state_change = true;
                state_change_reason = "recovered after timeout";
            }
        }

        if (log_state_change) {
            RCLCPP_INFO(
                this->get_logger(),
                "APP heartbeat state changed: %s (control_session_active=%s)",
                state_change_reason,
                active_after_update ? "true" : "false");
        }
    }

    void markAppControlSessionActive(const std::string &reason)
    {
        if (!heartbeat_required_after_control_) {
            return;
        }
        const auto now = this->now();
        std::lock_guard<std::mutex> lock(heartbeat_mutex_);
        const bool was_active = app_control_session_active_;
        app_control_session_active_ = true;
        last_client_activity_time_ = now;
        if (!was_active || !has_heartbeat_) {
            last_heartbeat_time_ = now;
            has_heartbeat_ = true;
        }
        RCLCPP_INFO(this->get_logger(), "APP heartbeat session active: %s", reason.c_str());
    }

    void markAppControlSessionInactive(const std::string &reason)
    {
        std::lock_guard<std::mutex> lock(heartbeat_mutex_);
        app_control_session_active_ = false;
        has_heartbeat_ = false;
        if (heartbeat_timeout_pending_) {
            heartbeat_timeout_pending_->store(false);
            heartbeat_timeout_pending_.reset();
        }
        RCLCPP_INFO(this->get_logger(), "APP heartbeat session inactive: %s", reason.c_str());
    }

    void resetHeartbeatForNewClient(const std::string &reason)
    {
        const auto now = this->now();
        const bool start_all_controllable = canAttemptStartAllControl();
        std::lock_guard<std::mutex> lock(heartbeat_mutex_);
        last_client_activity_time_ = now;
        last_heartbeat_time_ = now;
        has_heartbeat_ = start_all_controllable;
        app_control_session_active_ = heartbeat_required_after_control_ && start_all_controllable;
        if (heartbeat_timeout_pending_) {
            heartbeat_timeout_pending_->store(false);
            heartbeat_timeout_pending_.reset();
        }
        RCLCPP_INFO(this->get_logger(), "APP heartbeat reset for new client: %s", reason.c_str());
    }

    void checkHeartbeatWatchdog()
    {
        if (heartbeat_timeout_seconds_ <= 0.0 || !heartbeat_required_after_control_) {
            return;
        }

        double age = 0.0;
        {
            std::lock_guard<std::mutex> lock(heartbeat_mutex_);
            if (!app_control_session_active_ || !has_heartbeat_) {
                return;
            }
            age = (this->now() - last_heartbeat_time_).seconds();
            if (age <= heartbeat_timeout_seconds_) {
                return;
            }
            if (heartbeat_timeout_pending_) {
                return;
            }
            heartbeat_timeout_pending_ = std::make_shared<std::atomic<bool>>(true);
        }

        RCLCPP_ERROR(
            this->get_logger(),
            "APP heartbeat timeout %.2fs > %.2fs; entering disconnect protection.",
            age, heartbeat_timeout_seconds_);
        handleHeartbeatTimeoutProtection("APP heartbeat timeout");
    }

    struct StartAllTarget {
        pid_t pid = -1;
        pid_t pgid = -1;
    };

    bool parsePositivePid(const std::string &text, pid_t &value) const
    {
        if (text.empty()) {
            return false;
        }
        for (char ch : text) {
            if (!std::isdigit(static_cast<unsigned char>(ch))) {
                return false;
            }
        }
        try {
            const long parsed = std::stol(text);
            if (parsed <= 1 || parsed > std::numeric_limits<pid_t>::max()) {
                return false;
            }
            value = static_cast<pid_t>(parsed);
            return true;
        } catch (const std::exception &) {
            return false;
        }
    }

    bool readTrustedRunFile(const std::string &path, std::string &content) const
    {
        struct stat st;
        if (lstat(path.c_str(), &st) != 0) {
            return false;
        }
        if (!S_ISREG(st.st_mode)) {
            RCLCPP_WARN(this->get_logger(), "Ignoring %s: not a regular file.", path.c_str());
            return false;
        }
        if ((st.st_mode & (S_IWGRP | S_IWOTH)) != 0) {
            RCLCPP_WARN(this->get_logger(), "Ignoring %s: group/world writable mode %o.", path.c_str(), st.st_mode & 0777);
            return false;
        }
        const uid_t uid = geteuid();
        if (st.st_uid != uid && st.st_uid != 0) {
            RCLCPP_WARN(this->get_logger(), "Ignoring %s: owner uid %ld is not trusted uid %ld/root.",
                        path.c_str(), static_cast<long>(st.st_uid), static_cast<long>(uid));
            return false;
        }

        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Ignoring %s: cannot open for reading.", path.c_str());
            return false;
        }
        std::ostringstream buffer;
        buffer << file.rdbuf();
        content = buffer.str();
        return !content.empty();
    }

    bool processGroupExists(pid_t pgid) const
    {
        if (pgid <= 1 || pgid == getpgrp()) {
            return false;
        }
        if (kill(-pgid, 0) == 0) {
            return true;
        }
        return errno == EPERM;
    }

    bool validateStartAllTarget(const StartAllTarget &target) const
    {
        if (target.pid <= 1 || target.pgid <= 1) {
            RCLCPP_WARN(this->get_logger(), "Rejecting invalid start_all target pid=%ld pgid=%ld.",
                        static_cast<long>(target.pid), static_cast<long>(target.pgid));
            return false;
        }
        if (target.pgid == getpgrp()) {
            RCLCPP_WARN(this->get_logger(), "Rejecting start_all pgid=%ld because it is the APP node process group.",
                        static_cast<long>(target.pgid));
            return false;
        }
        if (kill(target.pid, 0) != 0) {
            RCLCPP_WARN(this->get_logger(), "start_all pid=%ld is not running: %s.",
                        static_cast<long>(target.pid), std::strerror(errno));
            return false;
        }
        const pid_t actual_pgid = getpgid(target.pid);
        if (actual_pgid != target.pgid) {
            RCLCPP_WARN(this->get_logger(), "Rejecting start_all target: pid=%ld has pgid=%ld, expected pgid=%ld.",
                        static_cast<long>(target.pid), static_cast<long>(actual_pgid), static_cast<long>(target.pgid));
            return false;
        }

        struct stat proc_st;
        const std::string proc_dir = "/proc/" + std::to_string(target.pid);
        if (stat(proc_dir.c_str(), &proc_st) != 0) {
            RCLCPP_WARN(this->get_logger(), "Rejecting start_all target: cannot stat %s.", proc_dir.c_str());
            return false;
        }
        const uid_t uid = geteuid();
        if (proc_st.st_uid != uid && proc_st.st_uid != 0) {
            RCLCPP_WARN(this->get_logger(), "Rejecting start_all target pid=%ld: owner uid %ld is not trusted uid %ld/root.",
                        static_cast<long>(target.pid), static_cast<long>(proc_st.st_uid), static_cast<long>(uid));
            return false;
        }

        const std::string cmdline_path = proc_dir + "/cmdline";
        std::ifstream cmdline_file(cmdline_path, std::ios::binary);
        if (!cmdline_file.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Rejecting start_all target: cannot read %s.", cmdline_path.c_str());
            return false;
        }
        std::string cmdline((std::istreambuf_iterator<char>(cmdline_file)), std::istreambuf_iterator<char>());
        if (cmdline.find("start_all.sh") == std::string::npos) {
            std::replace(cmdline.begin(), cmdline.end(), '\0', ' ');
            RCLCPP_WARN(this->get_logger(), "Rejecting non-start_all pid=%ld cmdline='%s'.",
                        static_cast<long>(target.pid), cmdline.c_str());
            return false;
        }
        return true;
    }

    bool readStartAllTargetFromPgidFile(StartAllTarget &target) const
    {
        std::string content;
        if (!readTrustedRunFile("/tmp/start_all.pgid", content)) {
            return false;
        }
        std::istringstream input(content);
        std::string pid_text;
        std::string pgid_text;
        std::string tag;
        input >> pid_text >> pgid_text >> tag;
        if (tag != "start_all.sh") {
            RCLCPP_WARN(this->get_logger(), "Ignoring /tmp/start_all.pgid: tag '%s' is not start_all.sh.", tag.c_str());
            return false;
        }
        StartAllTarget candidate;
        if (!parsePositivePid(pid_text, candidate.pid) || !parsePositivePid(pgid_text, candidate.pgid)) {
            RCLCPP_WARN(this->get_logger(), "Ignoring /tmp/start_all.pgid: invalid pid/pgid content '%s'.", content.c_str());
            return false;
        }
        if (!validateStartAllTarget(candidate)) {
            return false;
        }
        target = candidate;
        return true;
    }

    bool readStartAllTargetFromPidFile(StartAllTarget &target) const
    {
        std::string content;
        if (!readTrustedRunFile("/tmp/start_all.pid", content)) {
            return false;
        }
        std::istringstream input(content);
        std::string pid_text;
        input >> pid_text;
        StartAllTarget candidate;
        if (!parsePositivePid(pid_text, candidate.pid)) {
            RCLCPP_WARN(this->get_logger(), "Ignoring /tmp/start_all.pid: invalid pid content '%s'.", content.c_str());
            return false;
        }
        candidate.pgid = getpgid(candidate.pid);
        if (!validateStartAllTarget(candidate)) {
            return false;
        }
        target = candidate;
        return true;
    }

    bool findStartAllTarget(StartAllTarget &target) const
    {
        if (readStartAllTargetFromPgidFile(target)) {
            return true;
        }
        RCLCPP_WARN(this->get_logger(), "Falling back from /tmp/start_all.pgid to /tmp/start_all.pid for fail-safe stop.");
        return readStartAllTargetFromPidFile(target);
    }

    void clearStartAllRunFiles() const
    {
        std::remove("/tmp/start_all.pid");
        std::remove("/tmp/start_all.pgid");
        std::remove("/tmp/start_all.ready");
    }

    std::optional<std::string> readStartAllMode() const
    {
        std::ifstream ready_file("/tmp/start_all.ready");
        if (!ready_file.is_open()) {
            return std::nullopt;
        }
        std::string line;
        while (std::getline(ready_file, line)) {
            constexpr const char *prefix = "mode=";
            if (line.rfind(prefix, 0) == 0) {
                return line.substr(std::strlen(prefix));
            }
        }
        return std::nullopt;
    }

    void publishSafetyStopMove(const std::string &reason)
    {
        sensor_msgs::msg::Joy joy_msg;
        joy_msg.axes.resize(3, 0.0F);
        joy_msg.buttons.resize(0);

        for (int i = 0; i < 3; ++i) {
            joy_pub_->publish(joy_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        RCLCPP_ERROR(this->get_logger(), "Published fail-safe StopMove Joy command: %s", reason.c_str());
    }

    void requestArmResetBestEffort(const std::string &reason)
    {
        if (!z1_reset_arm_client_) {
            RCLCPP_WARN(
                this->get_logger(),
                "best-effort arm reset skipped: /z1_reset_arm client not initialized (reason=%s).",
                reason.c_str());
            return;
        }
        if (!z1_reset_arm_client_->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_WARN(
                this->get_logger(),
                "best-effort arm reset skipped: /z1_reset_arm not available within 500ms (reason=%s).",
                reason.c_str());
            return;
        }

        auto request = std::make_shared<z1_arm_controller_cpp::srv::MoveArm::Request>();
        auto response_done = std::make_shared<std::atomic<bool>>(false);
        const std::string reason_copy = reason;

        auto timeout_id = std::make_shared<uint64_t>(0);
        auto timeout_timer = this->create_wall_timer(
            std::chrono::duration<double>(arm_reset_timeout_seconds_),
            [this, reason_copy, response_done, timeout_id]() {
                if (!response_done->exchange(true)) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "best-effort arm reset timed out after %.1fs (reason=%s).",
                        arm_reset_timeout_seconds_,
                        reason_copy.c_str());
                }
                cancelSafetyTimer(*timeout_id);
            });
        const uint64_t arm_reset_timeout_handle = registerSafetyTimer(timeout_timer);
        *timeout_id = arm_reset_timeout_handle;

        z1_reset_arm_client_->async_send_request(
            request,
            [this, reason_copy, response_done, arm_reset_timeout_handle](
                rclcpp::Client<z1_arm_controller_cpp::srv::MoveArm>::SharedFuture future) {
                if (response_done->exchange(true)) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "best-effort arm reset response arrived after timeout (reason=%s).",
                        reason_copy.c_str());
                    cancelSafetyTimer(arm_reset_timeout_handle);
                    return;
                }
                cancelSafetyTimer(arm_reset_timeout_handle);
                try {
                    const auto &response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "best-effort arm reset succeeded: %s (reason=%s).",
                            response->message.c_str(),
                            reason_copy.c_str());
                    } else {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "best-effort arm reset failed: %s (reason=%s).",
                            response->message.c_str(),
                            reason_copy.c_str());
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "best-effort arm reset exception: %s (reason=%s).",
                        e.what(),
                        reason_copy.c_str());
                }
            });

        RCLCPP_WARN(
            this->get_logger(),
            "best-effort arm reset dispatched (reason=%s, timeout=%.1fs).",
            reason.c_str(),
            arm_reset_timeout_seconds_);
    }

    int triggerStartAllFailSafeStop(const std::string &reason)
    {
        publishSafetyStopMove(reason);
        requestArmResetBestEffort(reason);
        RCLCPP_ERROR(
            this->get_logger(),
            "Triggering start_all fail-safe stop because %s. This intentionally stops the autonomous task instead of letting the robot continue uncontrolled.",
            reason.c_str());

        StartAllTarget target;
        if (!findStartAllTarget(target)) {
            RCLCPP_ERROR(this->get_logger(), "start_all fail-safe stop failed: no verified start_all target found.");
            return 1;
        }

        RCLCPP_ERROR(this->get_logger(), "Fail-safe stopping verified start_all pid=%ld pgid=%ld.",
                     static_cast<long>(target.pid), static_cast<long>(target.pgid));
        if (kill(-target.pgid, SIGTERM) != 0 && errno != ESRCH) {
            RCLCPP_WARN(this->get_logger(), "SIGTERM to start_all pgid=%ld failed: %s.",
                        static_cast<long>(target.pgid), std::strerror(errno));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (processGroupExists(target.pgid)) {
            RCLCPP_ERROR(this->get_logger(), "start_all pgid=%ld still alive after SIGTERM; escalating to SIGKILL.",
                         static_cast<long>(target.pgid));
            if (kill(-target.pgid, SIGKILL) != 0 && errno != ESRCH) {
                RCLCPP_ERROR(this->get_logger(), "SIGKILL to start_all pgid=%ld failed: %s.",
                             static_cast<long>(target.pgid), std::strerror(errno));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        if (processGroupExists(target.pgid)) {
            RCLCPP_ERROR(this->get_logger(), "start_all fail-safe stop failed: process group %ld still alive.",
                         static_cast<long>(target.pgid));
            return 2;
        }
        clearStartAllRunFiles();
        RCLCPP_ERROR(this->get_logger(), "start_all fail-safe stop completed for pgid=%ld.",
                     static_cast<long>(target.pgid));
        return 0;
    }

    void handlePauseCommand(const std::string &reason = "APP pause command")
    {
        RCLCPP_INFO(this->get_logger(), "Received command: pause (0x02), reason: %s", reason.c_str());
        if (requestTrigger(emergency_stop_client_, "Emergency stop")) {
            RCLCPP_INFO(this->get_logger(), "Pause request accepted; waiting for emergency_stop response confirmation.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Pause rejected or service unavailable; executing fail-safe stop.");
            triggerStartAllFailSafeStop("pause fallback: emergency_stop unavailable or readiness rejected");
        }
    }

    void handleStopCommand()
    {
        RCLCPP_INFO(this->get_logger(), "Received command: stop (0x03)");
        const int rc = triggerStartAllFailSafeStop("stop command");
        markAppControlSessionInactive("stop command");
        RCLCPP_INFO(this->get_logger(), "Stop command fail-safe returned %d", rc);
    }

    void clearSafetyTimers()
    {
        std::lock_guard<std::mutex> lock(safety_timer_mutex_);
        for (auto &entry : safety_timers_) {
            if (entry.timer) {
                entry.timer->cancel();
            }
        }
        safety_timers_.clear();
    }

    uint64_t registerSafetyTimer(const rclcpp::TimerBase::SharedPtr &timer)
    {
        std::lock_guard<std::mutex> lock(safety_timer_mutex_);
        const uint64_t id = ++next_safety_timer_id_;
        safety_timers_.push_back({id, timer});
        return id;
    }

    void cancelSafetyTimer(uint64_t id)
    {
        std::lock_guard<std::mutex> lock(safety_timer_mutex_);
        auto it = std::find_if(
            safety_timers_.begin(),
            safety_timers_.end(),
            [id](const SafetyTimerEntry &entry) { return entry.id == id; });
        if (it != safety_timers_.end()) {
            if (it->timer) {
                it->timer->cancel();
            }
            safety_timers_.erase(it);
        }
    }

    void cancelDisconnectAutoPause(const std::string &reason)
    {
        std::lock_guard<std::mutex> lock(disconnect_timer_mutex_);
        if (disconnect_auto_pause_pending_) {
            disconnect_auto_pause_pending_->store(false);
            disconnect_auto_pause_pending_.reset();
        }
        if (disconnect_auto_pause_timer_) {
            disconnect_auto_pause_timer_->cancel();
            disconnect_auto_pause_timer_.reset();
            RCLCPP_INFO(this->get_logger(), "%s", reason.c_str());
        }
    }

    void scheduleDisconnectAutoPause()
    {
        if (client_disconnect_auto_pause_seconds_ <= 0.0) {
            RCLCPP_INFO(this->get_logger(), "APP client disconnected; disconnect auto-pause is disabled.");
            return;
        }

        auto pending = std::make_shared<std::atomic<bool>>(true);
        {
            std::lock_guard<std::mutex> lock(disconnect_timer_mutex_);
            if (disconnect_auto_pause_pending_) {
                disconnect_auto_pause_pending_->store(false);
                disconnect_auto_pause_pending_.reset();
            }
            if (disconnect_auto_pause_timer_) {
                disconnect_auto_pause_timer_->cancel();
            }
            disconnect_auto_pause_pending_ = pending;
            disconnect_auto_pause_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(client_disconnect_auto_pause_seconds_),
                [this, pending]() {
                    if (!pending->exchange(false)) {
                        return;
                    }
                    {
                        std::lock_guard<std::mutex> lock(disconnect_timer_mutex_);
                        if (disconnect_auto_pause_pending_ != pending) {
                            return;
                        }
                        disconnect_auto_pause_timer_.reset();
                        disconnect_auto_pause_pending_.reset();
                    }
                    {
                        std::lock_guard<std::mutex> client_lock(client_mutex_);
                        if (client_sock_ >= 0) {
                            RCLCPP_INFO(
                                this->get_logger(),
                                "APP client reconnected before disconnect auto-pause fired; skipping stale auto-pause timer.");
                            return;
                        }
                    }
                    if (!canAttemptStartAllControl()) {
                        RCLCPP_WARN(
                            this->get_logger(),
                            "APP client disconnected timeout %.1fs reached, but start_all is not controllable; publishing StopMove only.",
                            client_disconnect_auto_pause_seconds_);
                        publishSafetyStopMove("APP client disconnected timeout: start_all not controllable");
                        return;
                    }
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "APP client disconnected timeout %.1fs reached; requesting automatic pause.",
                        client_disconnect_auto_pause_seconds_);
                    handlePauseCommand("APP client disconnected timeout");
                });
        }

        RCLCPP_WARN(
            this->get_logger(),
            "APP client disconnected; will auto-pause after %.1fs if it does not reconnect.",
            client_disconnect_auto_pause_seconds_);
    }

    void handleHeartbeatTimeoutProtection(const std::string &reason)
    {
        {
            std::lock_guard<std::mutex> client_lock(client_mutex_);
            if (client_sock_ >= 0) {
                shutdownClientSocketLocked();
            }
        }

        if (!canAttemptStartAllControl()) {
            RCLCPP_WARN(
                this->get_logger(),
                "%s, but start_all is not controllable; publishing StopMove only.",
                reason.c_str());
            publishSafetyStopMove(reason + ": start_all not controllable");
            return;
        }

        handlePauseCommand(reason);
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
        auto response_done = std::make_shared<std::atomic<bool>>(false);
        uint64_t emergency_timeout_id = 0;
        if (action_name == "Emergency stop") {
            auto timeout_id = std::make_shared<uint64_t>(0);
            auto timeout_timer = this->create_wall_timer(
                std::chrono::seconds(2),
                [this, response_done, timeout_id]() {
                    if (!response_done->exchange(true)) {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "Emergency stop response timed out; executing fail-safe stop.");
                        triggerStartAllFailSafeStop("emergency stop response timeout");
                    }
                    cancelSafetyTimer(*timeout_id);
                });
            emergency_timeout_id = registerSafetyTimer(timeout_timer);
            *timeout_id = emergency_timeout_id;
        }

        client->async_send_request(
            request,
            [this, action_name, response_done, emergency_timeout_id](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                try {
                    if (response_done->exchange(true)) {
                        RCLCPP_WARN(this->get_logger(), "%s response arrived after timeout/fail-safe.", action_name.c_str());
                        return;
                    }
                    if (emergency_timeout_id != 0) {
                        cancelSafetyTimer(emergency_timeout_id);
                    }
                    const auto &response = future.get();
                    RCLCPP_INFO(
                        this->get_logger(),
                        "%s response: success=%s message='%s'",
                        action_name.c_str(),
                        response->success ? "true" : "false",
                        response->message.c_str());

                    if (action_name == "Emergency stop") {
                        const bool pause_confirmed = response->success &&
                            (response->message.find("Task paused") != std::string::npos ||
                             response->message.find("Task already paused") != std::string::npos);
                        if (!pause_confirmed) {
                            RCLCPP_ERROR(
                                this->get_logger(),
                                "Emergency stop response did not confirm paused state; executing fail-safe stop.");
                            triggerStartAllFailSafeStop("emergency stop bad response");
                        }
                        return;
                    }

                    if (!response->success) {
                        RCLCPP_WARN(this->get_logger(), "%s failed: %s",
                                    action_name.c_str(), response->message.c_str());
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "%s request failed with exception: %s",
                                 action_name.c_str(), e.what());
                    if (action_name == "Emergency stop") {
                        triggerStartAllFailSafeStop("emergency stop exception");
                    }
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

    void configureClientSocketTimeouts(int sock)
    {
        timeval timeout{};
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        if (setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to set SO_SNDTIMEO: %s", std::strerror(errno));
        }
        if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to set SO_RCVTIMEO: %s", std::strerror(errno));
        }
    }

    void shutdownClientSocketLocked()
    {
        if (client_sock_ < 0) {
            return;
        }
        shutdown(client_sock_, SHUT_RDWR);
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
        bool closed_active_client = false;
        {
            std::lock_guard<std::mutex> lock(client_mutex_);
            if (client_sock_ < 0) {
                return false;
            }
            if (!sendAllBytesLocked(client_sock_, data, len)) {
                shutdown(client_sock_, SHUT_RDWR);
                client_sock_ = -1;
                closed_active_client = true;
            } else {
                return true;
            }
        }
        if (closed_active_client && !stop_requested_.load()) {
            scheduleDisconnectAutoPause();
        }
        return false;
    }

    bool sendPacket(const std::vector<uint8_t> &packet)
    {
        return sendPacket(packet.data(), packet.size());
    }

    bool hasFloat64XYZFields(const sensor_msgs::msg::PointCloud2 &cloud)
    {
        const sensor_msgs::msg::PointField *x_field = nullptr;
        const sensor_msgs::msg::PointField *y_field = nullptr;
        const sensor_msgs::msg::PointField *z_field = nullptr;
        for (const auto &field : cloud.fields) {
            if (field.name == "x") {
                x_field = &field;
            } else if (field.name == "y") {
                y_field = &field;
            } else if (field.name == "z") {
                z_field = &field;
            }
        }
        auto is_valid_float64 = [](const sensor_msgs::msg::PointField *field) {
            return field != nullptr &&
                   field->datatype == sensor_msgs::msg::PointField::FLOAT64 &&
                   field->count >= 1;
        };
        if (!is_valid_float64(x_field) || !is_valid_float64(y_field) || !is_valid_float64(z_field)) {
            RCLCPP_ERROR(this->get_logger(),
                         "PointCloud2 x/y/z fields must exist and be FLOAT64; refusing APP upload.");
            return false;
        }
        const uint32_t required_point_step = std::max({x_field->offset, y_field->offset, z_field->offset}) + sizeof(double);
        if (cloud.point_step < required_point_step) {
            RCLCPP_ERROR(this->get_logger(),
                         "PointCloud2 point_step=%u is too small for FLOAT64 x/y/z offsets (required >= %u); refusing APP upload.",
                         cloud.point_step, required_point_step);
            return false;
        }
        const size_t min_row_step = static_cast<size_t>(cloud.width) * cloud.point_step;
        const size_t required_data_size = cloud.height > 0
            ? static_cast<size_t>(cloud.row_step) * cloud.height
            : 0;
        if (cloud.is_bigendian) {
            RCLCPP_ERROR(this->get_logger(),
                         "PointCloud2 is big-endian but APP protocol requires little-endian FLOAT64; refusing APP upload.");
            return false;
        }
        if (cloud.height > 1 && cloud.row_step != min_row_step) {
            RCLCPP_ERROR(this->get_logger(),
                         "PointCloud2 row_step=%u does not match tightly-packed width*point_step=%zu; refusing APP upload.",
                         cloud.row_step, min_row_step);
            return false;
        }
        if (cloud.row_step < min_row_step) {
            RCLCPP_ERROR(this->get_logger(),
                         "PointCloud2 row_step=%u is smaller than width*point_step=%zu; refusing APP upload.",
                         cloud.row_step, min_row_step);
            return false;
        }
        if (cloud.data.size() < required_data_size) {
            RCLCPP_ERROR(this->get_logger(),
                         "PointCloud2 data size=%zu is smaller than row_step*height=%zu; refusing APP upload.",
                         cloud.data.size(), required_data_size);
            return false;
        }
        return true;
    }

    void sendEmptyPointCloud(uint8_t func_code)
    {
        uint8_t packet[] = {0xF5, func_code, 0x00, 0x00, 0x00, 0x00, 0x5F};
        if (sendPacket(packet, sizeof(packet))) {
            logPacket("TX POINTCLOUD_EMPTY", packet, sizeof(packet), this->get_logger());
            RCLCPP_INFO(this->get_logger(), "Sent empty PointCloud func=0x%02X count=0", func_code);
        }
    }

    void sendPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud, uint8_t func_code)
    {
        RCLCPP_INFO(this->get_logger(), "Cloud step=%u fields=%zu", cloud->point_step, cloud->fields.size());

        size_t num_points = static_cast<size_t>(cloud->width) * static_cast<size_t>(cloud->height);
        if (num_points == 0) {
            sendEmptyPointCloud(func_code);
            return;
        }
        if (!hasFloat64XYZFields(*cloud)) {
            return;
        }
        if (num_points > std::numeric_limits<uint16_t>::max()) {
            RCLCPP_WARN(this->get_logger(), "PointCloud has %zu points, truncating to %u",
                        num_points, std::numeric_limits<uint16_t>::max());
            num_points = std::numeric_limits<uint16_t>::max();
        }
        try {
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
            if (sendPacket(packet)) {
                RCLCPP_INFO(this->get_logger(), "Sent PointCloud: %zu points func=0x%02X", num_points, func_code);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send PointCloud: %s", e.what());
        }
    }

    void sendAllPoints()
    {
        if (all_points_.empty()) {
            uint8_t packet[] = {0xF5, 0x01, 0x00, 0x00, 0x00, 0x00, 0x5F};
            if (sendPacket(packet, sizeof(packet))) {
                logPacket("TX ALL_POINTS_EMPTY", packet, sizeof(packet), this->get_logger());
                RCLCPP_INFO(this->get_logger(), "Sent ALL points count=0 with func=0x01");
            }
            return;
        }
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

        for (size_t i = 0; i < num_points; ++i) {
            const auto& p = all_points_[i];
            std::memcpy(&packet[idx], &p.x, sizeof(double)); idx += sizeof(double);
            std::memcpy(&packet[idx], &p.y, sizeof(double)); idx += sizeof(double);
            std::memcpy(&packet[idx], &p.z, sizeof(double)); idx += sizeof(double);
        }
        packet[idx++] = 0x00; // CRC low
        packet[idx++] = 0x00; // CRC high
        packet[idx++] = 0x5F; // tail
        if (sendPacket(packet)) {
            logPacket("TX ALL_POINTS", packet.data(), packet.size(), this->get_logger());
            RCLCPP_INFO(this->get_logger(), "Sent ALL points (%zu points) with func=0x01", num_points);
        }
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
        if (sendPacket(packet, sizeof(packet))) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "Sent progress: %u", progress);
            logPacket("TX PROGRESS", packet, sizeof(packet), this->get_logger());
        }
    }

    bool shouldSendPosition(double x, double y, double z)
    {
        if (!has_last_sent_position_) {
            return true;
        }
        const auto now = this->now();
        const double dx = x - last_sent_x_;
        const double dy = y - last_sent_y_;
        const double dz = z - last_sent_z_;
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (distance >= position_upload_min_distance_) {
            return true;
        }
        if (position_upload_hz_ > 0.0) {
            const double min_interval = 1.0 / position_upload_hz_;
            if ((now - last_position_sent_time_).seconds() >= min_interval) {
                return true;
            }
        }
        return false;
    }

    void sendPosition(double x, double y, double z, bool force = false, bool update_throttle_state = true) {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "Skip invalid position: x=%.3f y=%.3f z=%.3f", x, y, z);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(upload_state_mutex_);
            latest_position_ = std::array<double, 3>{x, y, z};
        }
        if (!force) {
            std::lock_guard<std::mutex> lock(position_upload_mutex_);
            if (!shouldSendPosition(x, y, z)) {
                return;
            }
        }

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

        if (!sendPacket(packet, sizeof(packet))) {
            return;
        }
        if (update_throttle_state) {
            std::lock_guard<std::mutex> lock(position_upload_mutex_);
            last_position_sent_time_ = this->now();
            has_last_sent_position_ = true;
            last_sent_x_ = x;
            last_sent_y_ = y;
            last_sent_z_ = z;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
         "Sent position : (%.3f, %.3f, %.3f)", x, y, z);
        logPacket("TX POSITION", packet, sizeof(packet), this->get_logger(), 5000);
    }

    void sendMaxTemperatureByte(uint8_t temp_byte)
    {
        uint8_t packet[6];
        packet[0] = 0xF5;      // 包头
        packet[1] = 0x05;      // 功能码
        packet[2] = temp_byte; // 温度值
        packet[3] = 0x00;      // CRC低字节（这里暂用0）
        packet[4] = 0x00;      // CRC高字节（这里暂用0）
        packet[5] = 0x5F;      // 包尾

        if (sendPacket(packet, sizeof(packet))) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "Sent max temperature: %u", temp_byte);
        }
    }

    void sendMaxTemperature(const std::vector<float>& temps)
    {
        if (temps.empty()) return;
        std::optional<float> max_temp_f;
        for (float temp : temps) {
            if (!std::isfinite(temp)) {
                continue;
            }
            if (!max_temp_f || temp > *max_temp_f) {
                max_temp_f = temp;
            }
        }
        if (!max_temp_f) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 4000,
                                 "Skip max temperature upload: all motor temperatures are non-finite.");
            return;
        }
        // 限制为 0~255
        uint8_t temp_byte = static_cast<uint8_t>(std::min(std::max(*max_temp_f, 0.0f), 255.0f));
        {
            std::lock_guard<std::mutex> lock(upload_state_mutex_);
            latest_max_temperature_ = temp_byte;
        }
        sendMaxTemperatureByte(temp_byte);
    }

    void sendPath(const nav_msgs::msg::Path::SharedPtr path_msg)
    {
        if (path_msg->poses.empty()) {
            uint8_t packet[] = {0xF5, 0x04, 0x00, 0x00, 0x00, 0x5F};
            if (sendPacket(packet, sizeof(packet))) {
                logPacket("TX PATH_EMPTY", packet, sizeof(packet), this->get_logger());
                RCLCPP_INFO(this->get_logger(), "Sent empty Path N=0");
            }
            return;
        }

        const size_t total_points = path_msg->poses.size();
        const size_t num_points = std::min<size_t>(total_points, 255);
        if (total_points > num_points) {
             RCLCPP_WARN(this->get_logger(), "Path has more than 255 points (%zu), sending latest 255.", total_points);
        }
        const uint8_t N = static_cast<uint8_t>(num_points);
        const size_t start_idx = total_points > num_points ? total_points - num_points : 0;
        // 修正长度计算
        // 结构: Header(1) + Func(1) + Count(1) + Points(N*24) + CRC(2) + Tail(1)
        // data_len 在这里仅指 "Count + Points" 或者我们直接算总长，避免混淆
        size_t points_data_len = num_points * 24; 
        size_t packet_len = 1 + 1 + 1 + points_data_len + 2 + 1; 
        
        std::vector<uint8_t> packet(packet_len, 0);

        size_t idx = 0;
        packet[idx++] = 0xF5; // 包头
        packet[idx++] = 0x04; // 功能码
        packet[idx++] = N;    // 点数量 (这是数据部分的第一个字节)
        // 写入轨迹点
        for (size_t i = 0; i < num_points; ++i) {
            const auto &pose_stamped = path_msg->poses[start_idx + i];
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

        if (sendPacket(packet)) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "Sent Path with %u points", N);
        }
    }

    void replayLatestUploadStateToClient()
    {
        sensor_msgs::msg::PointCloud2::SharedPtr acquired;
        sensor_msgs::msg::PointCloud2::SharedPtr unacquired;
        std::optional<uint8_t> progress;
        nav_msgs::msg::Path::SharedPtr path;
        std::optional<std::array<double, 3>> position;
        std::optional<uint8_t> max_temperature;
        {
            std::lock_guard<std::mutex> lock(upload_state_mutex_);
            acquired = latest_acquired_points_;
            unacquired = latest_unacquired_points_;
            progress = latest_progress_;
            path = latest_path_;
            position = latest_position_;
            max_temperature = latest_max_temperature_;
        }

        RCLCPP_INFO(this->get_logger(), "Replaying latest APP upload state to new client");
        if (acquired) {
            sendPointCloud(acquired, 0x02);
        }
        if (unacquired) {
            sendPointCloud(unacquired, 0x03);
        }
        if (progress) {
            sendProgress(*progress);
        }
        if (path) {
            sendPath(path);
        }
        if (position) {
            sendPosition((*position)[0], (*position)[1], (*position)[2], true, false);
        }
        if (max_temperature) {
            sendMaxTemperatureByte(*max_temperature);
        }
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
    rclcpp::Client<z1_arm_controller_cpp::srv::MoveArm>::SharedPtr z1_reset_arm_client_;
    double arm_reset_timeout_seconds_ = 15.0;

    std::vector<Point> all_points_; // 存储所有原始点
    // TCP
    std::thread tcp_thread_;
    std::thread client_thread_;
    std::atomic<bool> stop_requested_{false};
    int server_fd_ = -1;
    int client_sock_ = -1;          // 当前客户端 socket
    std::mutex client_mutex_;       // 保护 client_sock_
    struct SafetyTimerEntry {
        uint64_t id;
        rclcpp::TimerBase::SharedPtr timer;
    };

    std::mutex safety_timer_mutex_; // 保护 safety_timers_
    uint64_t next_safety_timer_id_ = 0;
    std::vector<SafetyTimerEntry> safety_timers_; // keep emergency timeout timers alive
    std::mutex disconnect_timer_mutex_; // 保护断联自动暂停计时器
    rclcpp::TimerBase::SharedPtr disconnect_auto_pause_timer_;
    std::shared_ptr<std::atomic<bool>> disconnect_auto_pause_pending_;
    double client_disconnect_auto_pause_seconds_ = 8.0;
    double heartbeat_timeout_seconds_ = 3.5;
    bool heartbeat_required_after_control_ = true;
    rclcpp::TimerBase::SharedPtr heartbeat_watchdog_timer_;
    std::mutex heartbeat_mutex_;
    rclcpp::Time last_heartbeat_time_;
    rclcpp::Time last_client_activity_time_;
    bool has_heartbeat_ = false;
    bool app_control_session_active_ = false;
    std::shared_ptr<std::atomic<bool>> heartbeat_timeout_pending_;

    std::mutex upload_state_mutex_;
    std::optional<uint8_t> latest_progress_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_acquired_points_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_unacquired_points_;
    nav_msgs::msg::Path::SharedPtr latest_path_;
    std::optional<std::array<double, 3>> latest_position_;
    std::optional<uint8_t> latest_max_temperature_;

    std::mutex position_upload_mutex_;
    double position_upload_hz_{5.0};
    double position_upload_min_distance_{0.02};
    rclcpp::Time last_position_sent_time_;
    bool has_last_sent_position_{false};
    double last_sent_x_{0.0};
    double last_sent_y_{0.0};
    double last_sent_z_{0.0};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
