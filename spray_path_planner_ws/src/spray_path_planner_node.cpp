
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/byte.hpp> 
#include "spray_path_planner/srv/get_next_waypoint.hpp"
#include "spray_path_planner/srv/set_start_point.hpp" // 新增头文件
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

struct Point {
    std::string id;
    double x, y, z;
    Point(const std::string& i, double x_val, double y_val, double z_val)
        : id(i), x(x_val), y(y_val), z(z_val) {}
};

class SprayPathPlannerNode : public rclcpp::Node {
public:
    SprayPathPlannerNode() : Node("spray_path_planner_node"), current_index_(0) {
        this->declare_parameter("file_path", std::string(""));
        std::string file_path;
        this->get_parameter("file_path", file_path);

        if (file_path.empty()) {
            RCLCPP_FATAL(this->get_logger(), "未设置参数 'file_path'！");
            rclcpp::shutdown();
            return;
        }

        all_points_ = parsePoints(file_path);
        if (all_points_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "未能读取到任何有效点位！");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "成功加载 %zu 个点位", all_points_.size());
        //  创建进度发布器
        progress_pub_ = this->create_publisher<std_msgs::msg::Byte>("/progress", 10);
        // 已获取和未获取点云发布
        acquired_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/acquired_points", 10);
        unacquired_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/unacquired_points", 10);
        // 默认以第一个点为起点初始化路径
        path_ = shortestPathFromStart(all_points_, 0);
        RCLCPP_INFO(this->get_logger(), "初始最短路径已生成，总长度: %.2f", computeTotalDistance(path_));

        // 创建获取下一个航点的服务
        get_next_waypoint_service_ = this->create_service<spray_path_planner::srv::GetNextWaypoint>(
            "get_next_waypoint",
            [this](const std::shared_ptr<spray_path_planner::srv::GetNextWaypoint::Request>&,
                   const std::shared_ptr<spray_path_planner::srv::GetNextWaypoint::Response>& response) {
                if (current_index_ >= path_.size()) {
                    response->success = false;
                    response->message = "All waypoints completed.";
                    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                    return;
                }

                const auto& p = path_[current_index_];
                response->waypoint.x = p.x;
                response->waypoint.y = p.y;
                response->waypoint.z = p.z;
                response->success = true;
                response->message = "OK";
                RCLCPP_INFO(this->get_logger(), "返回点位 #%zu: (%.2f, %.2f, %.2f)",
                            current_index_ + 1, p.x, p.y, p.z);
                current_index_++;
                //  每次取点后发布当前进度
                publishProgress();
            });
        // 设置起点
        set_start_point_service_ = this->create_service<spray_path_planner::srv::SetStartPoint>(
            "set_start_point",
            [this](const std::shared_ptr<spray_path_planner::srv::SetStartPoint::Request>& req,
                   const std::shared_ptr<spray_path_planner::srv::SetStartPoint::Response>& res) {
                // 找到距离请求点最近的实际点作为新的起点索引
                size_t start_idx = findClosestPointIndex(all_points_, { "", req->start.x, req->start.y, req->start.z });
                if (start_idx == std::string::npos) {
                    res->success = false;
                    res->message = "No valid point found near the provided start.";
                    RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
                    return;
                }

                // 重新生成路径
                path_ = shortestPathFromStart(all_points_, start_idx);
                current_index_ = 0; // 重置当前索引
                
                std::string log_dir = "/home/" + std::string(std::getenv("USER")) + "/sprayer_path_planner_ws";
                std::string save_file = "/path_saved.csv";
                std::ofstream ofs(save_file, std::ios::trunc);
                if (ofs.is_open()) {
                    ofs << "index,id,x,y,z\n";  // CSV 表头
                    for (size_t i = 0; i < path_.size(); ++i) {
                        ofs << i << "," << path_[i].id << "," 
                        << path_[i].x << "," << path_[i].y << ","  << path_[i].z << "\n";
                    }
                    ofs.close();
                    RCLCPP_INFO(this->get_logger(), "路径已保存到: %s", save_file.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "无法写入路径文件: %s", save_file.c_str());
                }
                
                res->success = true;
                res->message = "Path replanned from closest point to (" + std::to_string(req->start.x) +
                             ", " + std::to_string(req->start.y) +
                             ", " + std::to_string(req->start.z) + ")";
                RCLCPP_INFO(this->get_logger(), "已重新规划路径，起始点匹配 '%s' (索引 %zu)", all_points_[start_idx].id.c_str(), start_idx);
            });
    }

private:
    std::vector<Point> all_points_; // 存储所有原始点
    std::vector<Point> path_;       // 当前路径
    size_t current_index_;

    // 服务成员变量
    rclcpp::Service<spray_path_planner::srv::GetNextWaypoint>::SharedPtr get_next_waypoint_service_;
    rclcpp::Service<spray_path_planner::srv::SetStartPoint>::SharedPtr set_start_point_service_; // 新增
    rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr progress_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr acquired_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unacquired_pub_;

    // 进度发布函数
    void publishProgress() {
        std_msgs::msg::Byte msg;
        if (path_.empty()) {
            msg.data = 0;
        } else {
            float ratio = (float)current_index_ / (float)path_.size();
            msg.data = static_cast<uint8_t>(ratio * 100.0f);  // 0~100 的整数
        }
        progress_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Progress: %d%%", msg.data);
        // 分割已获取和未获取点
        std::vector<Point> acquired(path_.begin(), path_.begin() + current_index_);
        std::vector<Point> unacquired(path_.begin() + current_index_, path_.end());
        if (acquired_pub_->get_subscription_count() > 0) {
            acquired_pub_->publish(createPointCloud2(acquired));
        }
        if (unacquired_pub_->get_subscription_count() > 0) {
            unacquired_pub_->publish(createPointCloud2(unacquired));
        }
    }

    sensor_msgs::msg::PointCloud2 createPointCloud2(const std::vector<Point>& points) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = "map";
        cloud.header.stamp = this->now();
        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(points.size());
        cloud.is_dense = true;
        cloud.is_bigendian = false;
        // 使用 modifier 设置字段
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT64,
            "y", 1, sensor_msgs::msg::PointField::FLOAT64,
            "z", 1, sensor_msgs::msg::PointField::FLOAT64
        );
        // 只用 modifier.resize 自动分配数据
        modifier.resize(points.size());
        // 使用 iterator 填充数据
        sensor_msgs::PointCloud2Iterator<double> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<double> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<double> iter_z(cloud, "z");

        for (size_t i = 0; i < points.size(); ++i) {
            iter_x[i] = points[i].x;
            iter_y[i] = points[i].y;
            iter_z[i] = points[i].z;
        }
        return cloud;
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
                } catch (...) {
                    RCLCPP_WARN(this->get_logger(), "解析错误，第 %d 行", line_num);
                }
            }
        }
        file.close();
        return points;
    }

    double distance2D(const Point& a, const Point& b) {
        return std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
    }

    // 辅助函数：找离目标点最近的点的索引
    size_t findClosestPointIndex(const std::vector<Point>& points, const Point& target) {
        if (points.empty()) return std::string::npos;

        double min_dist = std::numeric_limits<double>::max();
        size_t idx = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            double d = distance2D(points[i], target);
            if (d < min_dist) {
                min_dist = d;
                idx = i;
            }
        }
        return idx;
    }

    std::vector<Point> shortestPathFromStart(const std::vector<Point>& points, size_t startIdx) {
        if (points.empty() || startIdx >= points.size()) return {};
        std::vector<Point> result;
        std::vector<bool> visited(points.size(), false);
        size_t current = startIdx;

        while (result.size() < points.size()) {
            result.push_back(points[current]);
            visited[current] = true;

            double minDist = std::numeric_limits<double>::max();
            size_t nextIdx = current;
            for (size_t i = 0; i < points.size(); ++i) {
                if (!visited[i]) {
                    double dist = distance2D(points[current], points[i]);
                    if (dist < minDist) {
                        minDist = dist;
                        nextIdx = i;
                    }
                }
            }
            current = nextIdx;
        }
        return result;
    }

    double computeTotalDistance(const std::vector<Point>& path) {
        double total = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            total += distance2D(path[i-1], path[i]);
        }
        return total;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SprayPathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}