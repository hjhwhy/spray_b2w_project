import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
APP_NODE = ROOT / "app_ws" / "src" / "app_node.cpp"
B2W_NAV = ROOT / "b2w_navigation_ws" / "src" / "main.cpp"
APP_CMAKE = ROOT / "app_ws" / "CMakeLists.txt"


def app_source() -> str:
    return APP_NODE.read_text(encoding="utf-8")


def nav_source() -> str:
    return B2W_NAV.read_text(encoding="utf-8")


def app_cmake() -> str:
    return APP_CMAKE.read_text(encoding="utf-8")


def function_body(text: str, signature: str, next_signature: str | None = None) -> str:
    start = text.find(signature)
    assert start != -1, f"missing {signature}"
    if next_signature is None:
        return text[start:]
    end = text.find(next_signature, start)
    assert end != -1, f"missing {next_signature} after {signature}"
    return text[start:end]


def test_pointcloud_sender_reads_float64_cloud_as_double():
    text = app_source()
    body = function_body(text, "void sendPointCloud", "void sendAllPoints")
    assert "PointCloud2ConstIterator<double> iter_x" in body
    assert "PointCloud2ConstIterator<double> iter_y" in body
    assert "PointCloud2ConstIterator<double> iter_z" in body
    assert "PointCloud2ConstIterator<float>" not in body
    assert "double x = *iter_x" in body
    assert "num_points * 24" in body
    assert "static_cast<size_t>(cloud->width) * static_cast<size_t>(cloud->height)" in body


def test_pointcloud_sender_requires_float64_xyz_fields():
    text = app_source()
    helper = function_body(text, "bool hasFloat64XYZFields", "void sendEmptyPointCloud")
    assert "sensor_msgs::msg::PointField::FLOAT64" in helper
    assert 'field.name == "x"' in helper
    assert 'field.name == "y"' in helper
    assert 'field.name == "z"' in helper
    assert "cloud.point_step" in helper
    assert "cloud.data.size()" in helper
    assert "cloud.row_step" in helper
    assert "cloud.is_bigendian" in helper
    assert "row_step*height" in helper
    assert "RCLCPP_ERROR" in helper
    pointcloud = function_body(text, "void sendPointCloud", "void sendAllPoints")
    assert "hasFloat64XYZFields(*cloud)" in pointcloud
    assert pointcloud.find("hasFloat64XYZFields(*cloud)") < pointcloud.find("PointCloud2ConstIterator<double>")


def test_empty_pointcloud_sends_zero_count_frame():
    text = app_source()
    assert "void sendEmptyPointCloud" in text
    assert "{0xF5, func_code, 0x00, 0x00, 0x00, 0x00, 0x5F}" in text
    body = function_body(text, "void sendPointCloud", "void sendAllPoints")
    assert "if (num_points == 0)" in body
    assert "sendEmptyPointCloud(func_code)" in body
    assert body.find("sendEmptyPointCloud(func_code)") < body.find("hasFloat64XYZFields(*cloud)")


def test_position_sender_filters_non_finite_values():
    text = app_source()
    body = function_body(text, "void sendPosition", "void sendMaxTemperatureByte")
    assert "std::isfinite(x)" in body
    assert "std::isfinite(y)" in body
    assert "std::isfinite(z)" in body
    assert "RCLCPP_WARN_THROTTLE" in body
    assert "Skip invalid position" in body
    invalid_guard = re.search(r"if \(!std::isfinite\(x\)[\s\S]+?return;[\s\S]+?\}", body)
    assert invalid_guard is not None
    assert invalid_guard.group(0).find("return;") < body.find("packet[idx++] = 0xF5")
    assert "latest_position_ = std::array<double, 3>{x, y, z}" in body


def test_all_points_empty_sends_zero_count_frame():
    text = app_source()
    body = function_body(text, "void sendAllPoints", "void sendProgress")
    assert "if (all_points_.empty())" in body
    assert "{0xF5, 0x01, 0x00, 0x00, 0x00, 0x00, 0x5F}" in body
    assert "TX ALL_POINTS_EMPTY" in body


def test_all_points_truncation_loop_uses_num_points():
    text = app_source()
    body = function_body(text, "void sendAllPoints", "void sendProgress")
    assert "num_points = std::numeric_limits<uint16_t>::max()" in body
    assert re.search(r"for \(size_t i = 0; i < num_points; \+\+i\)", body)
    assert "const auto& p = all_points_[i]" in body
    assert "for (const auto& p : all_points_)" not in body


def test_empty_path_sends_zero_count_frame():
    text = app_source()
    body = function_body(text, "void sendPath", "void replayLatestUploadStateToClient")
    assert "if (path_msg->poses.empty())" in body
    assert "{0xF5, 0x04, 0x00, 0x00, 0x00, 0x5F}" in body
    assert "TX PATH_EMPTY" in body
    assert "N=0" in body


def test_path_truncation_sends_latest_255_points():
    text = app_source()
    body = function_body(text, "void sendPath", "void replayLatestUploadStateToClient")
    assert "const size_t total_points = path_msg->poses.size()" in body
    assert "const size_t num_points = std::min<size_t>(total_points, 255)" in body
    assert "const size_t start_idx = total_points > num_points ? total_points - num_points : 0" in body
    assert "path_msg->poses[start_idx + i]" in body
    assert "path_msg->poses[i]" not in body


def test_new_client_replays_cached_upload_state():
    text = app_source()
    accept_body = function_body(text, "void runTcpServer", "void handleClient")
    assert "configureClientSocketTimeouts(new_socket);" in accept_body
    assert "client_thread_ = std::thread(&RemoteControlNode::handleClient, this, new_socket);" in accept_body
    assert "sendAllPoints();" in accept_body
    assert "replayLatestUploadStateToClient();" in accept_body
    assert accept_body.find("client_thread_ = std::thread(&RemoteControlNode::handleClient, this, new_socket);") < accept_body.find("sendAllPoints();")
    assert accept_body.find("sendAllPoints();") < accept_body.find("replayLatestUploadStateToClient();")

    replay = function_body(text, "void replayLatestUploadStateToClient", "std::vector<Point> parsePoints")
    assert "std::lock_guard<std::mutex> lock(upload_state_mutex_)" in replay
    locked_block = replay[replay.find("std::lock_guard<std::mutex> lock(upload_state_mutex_)"): replay.find("RCLCPP_INFO")]
    assert "sendPacket" not in locked_block
    assert "sendPointCloud(acquired, 0x02)" in replay
    assert "sendPointCloud(unacquired, 0x03)" in replay
    assert "sendProgress(*progress)" in replay
    assert "sendPath(path)" in replay
    assert "sendPosition((*position)[0], (*position)[1], (*position)[2], true, false)" in replay
    assert "sendMaxTemperatureByte(*max_temperature)" in replay

    for member in [
        "upload_state_mutex_",
        "latest_progress_",
        "latest_acquired_points_",
        "latest_unacquired_points_",
        "latest_path_",
        "latest_position_",
        "latest_max_temperature_",
    ]:
        assert member in text


def test_position_upload_throttle_uses_class_level_state():
    text = app_source()
    for snippet in [
        'declare_parameter<double>("position_upload_hz", 5.0)',
        'declare_parameter<double>("position_upload_min_distance", 0.02)',
        "std::mutex position_upload_mutex_",
        "double position_upload_hz_{5.0}",
        "double position_upload_min_distance_{0.02}",
        "rclcpp::Time last_position_sent_time_",
        "bool has_last_sent_position_{false}",
        "double last_sent_x_{0.0}",
        "double last_sent_y_{0.0}",
        "double last_sent_z_{0.0}",
    ]:
        assert snippet in text
    should_send = function_body(text, "bool shouldSendPosition", "void sendPosition")
    assert "1.0 / position_upload_hz_" in should_send
    assert "position_upload_min_distance_" in should_send
    assert "std::sqrt" in should_send
    send_position = function_body(text, "void sendPosition", "void sendMaxTemperatureByte")
    assert "if (!force)" in send_position
    assert "std::lock_guard<std::mutex> lock(position_upload_mutex_)" in send_position
    assert "!shouldSendPosition(x, y, z)" in send_position
    assert "if (update_throttle_state)" in send_position
    assert "last_position_sent_time_ = this->now()" in send_position
    assert "has_last_sent_position_ = true" in send_position


def test_app_ws_uses_cpp17_for_optional_upload_state():
    cmake = app_cmake()
    assert "set(CMAKE_CXX_STANDARD 17)" in cmake


def test_send_failure_shutdowns_active_socket_without_double_closing_fd():
    text = app_source()
    body = function_body(text, "bool sendPacket(const uint8_t *data", "bool sendPacket(const std::vector<uint8_t> &packet)")
    assert "shutdown(client_sock_, SHUT_RDWR);" in body
    assert "client_sock_ = -1;" in body
    assert "closeClientSocketLocked();" not in body


def test_navigation_still_publishes_b2w_odom():
    text = nav_source()
    assert "odom_pub_->publish" in text or "odom_pub_->publish(" in text
    assert "/b2w_odom" in text
