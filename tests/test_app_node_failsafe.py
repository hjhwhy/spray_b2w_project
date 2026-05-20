import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
APP_NODE = ROOT / "app_ws" / "src" / "app_node.cpp"
B2W_TELEOP = ROOT / "b2w_navigation_ws" / "src" / "b2w_teleop.cpp"


def source() -> str:
    return APP_NODE.read_text(encoding="utf-8")


def teleop_source() -> str:
    return B2W_TELEOP.read_text(encoding="utf-8")


def test_pause_falls_back_to_stopmove_when_emergency_service_unavailable():
    text = source()
    assert "handlePauseCommand" in text
    assert "triggerStartAllFailSafeStop(\"pause fallback" in text
    assert "publishSafetyStopMove" in text
    pause_match = re.search(
        r"void handlePauseCommand[\s\S]+?\n    void handleStopCommand",
        text,
    )
    assert pause_match is not None
    assert "allow_damp_fallback" not in pause_match.group(0)


def test_stop_command_uses_stopmove_and_native_verified_process_group_kill():
    text = source()
    assert "handleStopCommand" in text
    assert "triggerStartAllFailSafeStop(\"stop command" in text
    stop_handler = re.search(
        r"void handleStopCommand[\s\S]+?\n    void clearSafetyTimers",
        text,
    )
    assert stop_handler is not None
    assert "publishSafetyDamp" not in stop_handler.group(0)
    assert "/tmp/start_all.pgid" in text
    stop_match = re.search(
        r"int triggerStartAllFailSafeStop[\s\S]+?\n    void handlePauseCommand",
        text,
    )
    assert stop_match is not None
    assert "system(" not in stop_match.group(0)
    assert "readStartAllTargetFromPgidFile" in text
    assert "tag != \"start_all.sh\"" in text
    assert "getpgid(candidate.pid)" in text
    assert "kill(-target.pgid, SIGTERM)" in text
    assert "kill(-target.pgid, SIGKILL)" in text
    assert "processGroupExists(target.pgid)" in text


def test_fail_safe_refuses_untrusted_or_unverified_targets():
    text = source()
    assert "lstat(path.c_str(), &st)" in text
    assert "!S_ISREG(st.st_mode)" in text
    assert "S_IWGRP | S_IWOTH" in text
    assert "st.st_uid != uid && st.st_uid != 0" in text
    assert "target.pgid == getpgrp()" in text
    assert "actual_pgid != target.pgid" in text
    assert "cmdline.find(\"start_all.sh\")" in text
    assert "Falling back from /tmp/start_all.pgid to /tmp/start_all.pid" in text


def test_emergency_stop_response_logs_message_and_falls_back_on_bad_response():
    text = source()
    assert "response->message" in text
    assert "%s response: success=%s message='%s'" in text
    assert "Task paused" in text
    assert "Task already paused" in text
    assert "triggerStartAllFailSafeStop(\"emergency stop bad response" in text


def test_emergency_stop_timeout_timer_is_per_request_not_global_clear():
    text = source()
    match = re.search(
        r"bool requestTrigger\([\s\S]+?\n    bool canAttemptStartAllControl",
        text,
    )
    assert match is not None
    request_trigger = match.group(0)
    assert "registerSafetyTimer(timeout_timer)" in request_trigger
    assert "cancelSafetyTimer(emergency_timeout_id)" in request_trigger
    assert "cancelSafetyTimer(*timeout_id)" in request_trigger
    assert "clearSafetyTimers()" not in request_trigger


def test_client_disconnect_auto_pause_is_delayed_and_cancelled_on_reconnect():
    text = source()
    assert "client_disconnect_auto_pause_seconds" in text
    assert "scheduleDisconnectAutoPause" in text
    assert "cancelDisconnectAutoPause" in text
    assert "std::chrono::duration<double>(client_disconnect_auto_pause_seconds_)" in text
    assert "APP client reconnect/cancel disconnect auto-pause" in text
    assert "APP client disconnected timeout" in text
    assert "handlePauseCommand" in text


def test_reconnect_invalidates_pending_disconnect_timer_token_and_checks_live_socket():
    text = source()
    match = re.search(
        r"void cancelDisconnectAutoPause[\s\S]+?\n    void scheduleDisconnectAutoPause",
        text,
    )
    assert match is not None
    cancel = match.group(0)
    assert "disconnect_auto_pause_pending_->store(false)" in cancel
    assert "disconnect_auto_pause_pending_.reset()" in cancel
    match = re.search(
        r"void scheduleDisconnectAutoPause[\s\S]+?\n    bool requestTrigger",
        text,
    )
    assert match is not None
    schedule = match.group(0)
    assert "disconnect_auto_pause_pending_ != pending" in schedule
    assert "disconnect_auto_pause_pending_.reset()" in schedule
    assert "std::lock_guard<std::mutex> client_lock(client_mutex_)" in schedule
    assert "client_sock_ >= 0" in schedule


def test_disconnect_auto_pause_uses_stopmove_when_start_all_is_not_controllable():
    text = source()
    match = re.search(
        r"void scheduleDisconnectAutoPause[\s\S]+?\n    bool requestTrigger",
        text,
    )
    assert match is not None
    schedule = match.group(0)
    guard = re.search(
        r'if \(!canAttemptStartAllControl\(\)\) \{[\s\S]+?publishSafetyStopMove\("APP client disconnected timeout: start_all not controllable"\);[\s\S]+?return;[\s\S]+?\}',
        schedule,
    )
    assert guard is not None
    assert "publishSafetyDamp" not in guard.group(0)
    assert schedule.find(guard.group(0)) < schedule.find("handlePauseCommand")


def test_all_fail_safe_fallbacks_use_stopmove_not_damp():
    text = source()
    assert "publishSafetyStopMove" in text
    stopmove_match = re.search(
        r"void publishSafetyStopMove[\s\S]+?\n    int triggerStartAllFailSafeStop",
        text,
    )
    assert stopmove_match is not None
    stopmove = stopmove_match.group(0)
    assert "joy_msg.axes.resize(3, 0.0F)" in stopmove
    assert "joy_msg.buttons.resize(0)" in stopmove
    fail_safe_match = re.search(
        r"int triggerStartAllFailSafeStop[\s\S]+?\n    void handlePauseCommand",
        text,
    )
    assert fail_safe_match is not None
    fail_safe = fail_safe_match.group(0)
    assert "publishSafetyStopMove(reason)" in fail_safe
    assert "publishSafetyDamp" not in fail_safe
    assert "allow_damp_fallback" not in fail_safe
    match = re.search(
        r"void scheduleDisconnectAutoPause[\s\S]+?\n    bool requestTrigger",
        text,
    )
    assert match is not None
    schedule = match.group(0)
    assert "handlePauseCommand(\"APP client disconnected timeout\")" in schedule
    assert "publishSafetyDamp" not in schedule
    request_match = re.search(
        r"bool requestTrigger\([\s\S]+?\n    bool canAttemptStartAllControl",
        text,
    )
    assert request_match is not None
    request_trigger = request_match.group(0)
    assert "triggerStartAllFailSafeStop(\"emergency stop response timeout\")" in request_trigger
    assert "triggerStartAllFailSafeStop(\"emergency stop bad response\")" in request_trigger
    assert "triggerStartAllFailSafeStop(\"emergency stop exception\")" in request_trigger
    assert "allow_damp_fallback" not in request_trigger


def test_posture_joy_buttons_are_standdown_standup_only_no_damp():
    text = source()
    teleop = teleop_source()
    assert "publishSafetyDamp" not in text
    assert "allow_damp_fallback" not in text
    assert text.count("joy_msg.buttons[0] = 1") == 1
    assert text.count("joy_msg.buttons[1] = 1") == 1
    assert "case 0x0A: joy_msg.buttons[0] = 1; break;" in text
    assert "case 0x0B: joy_msg.buttons[1] = 1; break;" in text
    assert "joy_msg.buttons[2] = 1" not in text
    assert "instruction_type == 0xFF" not in text
    assert ".Damp()" not in teleop
    assert "buttons[2]" not in teleop
    assert ".StandDown()" in teleop
    assert ".StandUp()" in teleop
    assert ".StopMove()" in teleop


def test_heartbeat_function_code_is_not_damp_or_joy_button():
    text = source()
    teleop = teleop_source()
    assert "func_code == 0xFF" in text
    assert "RX HEARTBEAT" in text
    assert "handleHeartbeatPacket" in text
    assert "instruction_type == 0xFF" not in text
    assert "joy_msg.buttons[2] = 1" not in text
    assert ".Damp()" not in teleop
    assert "buttons[2]" not in teleop


def test_heartbeat_watchdog_has_timeout_and_session_gate():
    text = source()
    assert "heartbeat_timeout_seconds" in text
    assert "heartbeat_required_after_control" in text
    assert "heartbeat_watchdog_timer_" in text
    assert "checkHeartbeatWatchdog" in text
    assert "app_control_session_active_" in text
    assert "last_heartbeat_time_" in text
    assert "const bool was_active = app_control_session_active_" in text
    assert "if (!was_active || !has_heartbeat_)" in text
    assert "APP heartbeat timeout" in text


def test_app_motion_or_posture_commands_activate_heartbeat_session():
    text = source()
    start_case = re.search(r"case 0x01:[\s\S]+?break;", text)
    assert start_case is not None
    assert "markAppControlSessionActive" in start_case.group(0)
    motion_block = re.search(r"if \(instruction_type >= 0x04 && instruction_type <= 0x09\)[\s\S]+?return;", text)
    assert motion_block is not None
    assert "markAppControlSessionActive" in motion_block.group(0)
    posture_block = re.search(r"if \(instruction_type == 0x0A \|\| instruction_type == 0x0B\)[\s\S]+?return;", text)
    assert posture_block is not None
    assert "markAppControlSessionActive" in posture_block.group(0)
    stop_handler = re.search(r"void handleStopCommand[\s\S]+?\n    void clearSafetyTimers", text)
    assert stop_handler is not None
    assert "markAppControlSessionInactive" in stop_handler.group(0)


def test_heartbeat_timeout_uses_disconnect_protection_not_posture_buttons():
    text = source()
    match = re.search(r"void handleHeartbeatTimeoutProtection[\s\S]+?\n    bool requestTrigger", text)
    assert match is not None
    body = match.group(0)
    assert "closeClientSocketLocked" in body
    assert "publishSafetyStopMove" in body
    assert "handlePauseCommand" in body
    assert "StandDown" not in body
    assert "Damp" not in body
    assert "joy_msg.buttons" not in body


def test_send_failure_schedules_disconnect_auto_pause():
    text = source()
    match = re.search(r"bool sendPacket\(const uint8_t \*data[\s\S]+?\n    bool sendPacket\(const std::vector", text)
    assert match is not None
    body = match.group(0)
    assert "sendAllBytesLocked" in body
    assert "closeClientSocketLocked" in body
    assert "scheduleDisconnectAutoPause" in body
    assert body.find("closeClientSocketLocked") < body.find("scheduleDisconnectAutoPause")
