// spray_path_planner_node.cpp
// 旧版本，使用文件中的点作为起点
//
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>

// 替换为你自己的包名
#include "spray_path_planner/srv/get_next_waypoint.hpp"
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <limits>

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

        auto points = parsePoints(file_path);
        if (points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "未能读取到任何有效点位！");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "成功加载 %zu 个点位", points.size());

        // 使用文件中第一个点作为起点
        path_ = shortestPathFromStart(points, 0);
        RCLCPP_INFO(this->get_logger(), "生成最短路径，总长度: %.2f", computeTotalDistance(path_));

        // 创建服务
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
            });
    }

private:
    std::vector<Point> path_;
    size_t current_index_;

    rclcpp::Service<spray_path_planner::srv::GetNextWaypoint>::SharedPtr get_next_waypoint_service_;

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

    std::vector<Point> shortestPathFromStart(const std::vector<Point>& points, size_t startIdx) {
        if (points.empty()) return {};
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