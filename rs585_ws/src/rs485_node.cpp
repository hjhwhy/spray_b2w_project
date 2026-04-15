#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <mutex>

class RelayControlNode : public rclcpp::Node
{
public:
    RelayControlNode() : Node("relay_control_node")
    {
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 9600);
        this->declare_parameter("trigger_time_ms", 600);  //喷涂时间参数
        std::string serial_port;
        int baud_rate;
        int trigger_time_ms;
        this->get_parameter("serial_port", serial_port);
        this->get_parameter("baud_rate", baud_rate);
        this->get_parameter("trigger_time_ms", trigger_time_ms); 
        // 打开串口
        serial_fd_ = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) {
            RCLCPP_FATAL(this->get_logger(), "无法打开串口: %s", serial_port.c_str());
            rclcpp::shutdown();
            return;
        }
        configure_serial(baud_rate, 8, 'N', 1);

        // --- CH1 电磁阀服务：触发后开 1 秒再关 ---
        ch1_service_ = this->create_service<std_srvs::srv::Trigger>(
            "trigger_valve_ch1",
            [this, trigger_time_ms](
                const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
                const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
            {
                response->success = trigger_ch1_valve(trigger_time_ms);
                response->message = response->success ? "CH1 电磁阀触发成功" : "CH1 触发失败";
            });

        // --- CH2 水泵服务：开关控制 ---
        ch2_service_ = this->create_service<std_srvs::srv::Trigger>(
            "trigger_pump_ch2",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
                const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
            {
                response->success = trigger_ch2_pump();
                response->message = response->success ? "CH2 水泵触发成功" : "CH2 触发失败";
            });

        RCLCPP_INFO(this->get_logger(), "继电器控制服务已启动");
        RCLCPP_INFO(this->get_logger(), "CH1 电磁阀服务: /trigger_valve_ch1");
        RCLCPP_INFO(this->get_logger(), "CH2 水泵服务: /trigger_pump_ch2");

        // --- 节点启动时默认打开 CH2（水泵） ---
        RCLCPP_INFO(this->get_logger(), "节点启动：自动打开 CH2（水泵）");
        handle_ch2(true);
    }

    ~RelayControlNode()
    {
        if (serial_fd_ != -1) {
            RCLCPP_INFO(this->get_logger(), "节点退出：关闭 CH2（水泵）");
            handle_ch2(false);
            close(serial_fd_);
        }
    }

private:
    int serial_fd_;
    std::mutex cmd_mutex_;

    // ===== 基础串口配置 =====
    void configure_serial(int baud_rate, int data_bits, char parity, int stop_bits)
    {
        struct termios options;
        tcgetattr(serial_fd_, &options);

        speed_t baud;
        switch (baud_rate) {
            case 9600:   baud = B9600; break;
            case 19200:  baud = B19200; break;
            case 38400:  baud = B38400; break;
            case 115200: baud = B115200; break;
            default:     baud = B9600; break;
        }
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);

        options.c_cflag &= ~CSIZE;
        if (data_bits == 8) options.c_cflag |= CS8;

        if (parity == 'E') { options.c_cflag |= PARENB; options.c_cflag &= ~PARODD; }
        else if (parity == 'O') { options.c_cflag |= PARENB; options.c_cflag |= PARODD; }
        else options.c_cflag &= ~PARENB;

        if (stop_bits == 2) options.c_cflag |= CSTOPB;
        else options.c_cflag &= ~CSTOPB;

        options.c_cflag |= (CLOCAL | CREAD);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 10;

        tcflush(serial_fd_, TCIFLUSH);
        tcsetattr(serial_fd_, TCSANOW, &options);
    }

    // ===== 命令发送 =====
    void send_command(const uint8_t* cmd, size_t len)
    {
        int n = write(serial_fd_, cmd, len);
        if (n != (int)len)
            RCLCPP_ERROR(this->get_logger(), "串口发送失败");
        usleep(100000);
    }
    // ============================================================
    // CH1 电磁阀 — 触发时开 1 秒再关
    // ============================================================
    bool trigger_ch1_valve(int trigger_time_ms)
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);

        uint8_t ch1_on[]  = {0x01,0x05,0x00,0x00,0xFF,0x00,0x8C,0x3A};
        uint8_t ch1_off[] = {0x01,0x05,0x00,0x00,0x00,0x00,0xCD,0xCA};

        RCLCPP_INFO(this->get_logger(), "CH1 电磁阀：打开 1 秒");
        send_command(ch1_on, sizeof(ch1_on));

        std::this_thread::sleep_for(std::chrono::milliseconds(trigger_time_ms));

        RCLCPP_INFO(this->get_logger(), "CH1 电磁阀：关闭");
        send_command(ch1_off, sizeof(ch1_off));

        return true;
    }
    // ============================================================
    // CH2 水泵 — 用于开关
    // ============================================================
    bool handle_ch2(bool on)
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);

        if (on) {
            uint8_t cmd[] = {0x01,0x05,0x00,0x01,0xFF,0x00,0xDD,0xFA};
            send_command(cmd, sizeof(cmd));
        } else {
            uint8_t cmd[] = {0x01,0x05,0x00,0x01,0x00,0x00,0x9C,0x0A};
            send_command(cmd, sizeof(cmd));
        }
        return true;
    }

    // 保留你原来的触发动作为 CH2 的 Trigger 服务
    bool trigger_ch2_pump()
    {
        handle_ch2(true);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        handle_ch2(false);
        return true;
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ch1_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ch2_service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelayControlNode>());
    rclcpp::shutdown();
    return 0;
}
