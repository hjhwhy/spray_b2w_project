import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
APP_NODE = ROOT / "app_ws" / "src" / "app_node.cpp"


def source() -> str:
    return APP_NODE.read_text(encoding="utf-8")


def test_pause_falls_back_to_hard_stop_when_emergency_service_unavailable():
    text = source()
    assert "handlePauseCommand" in text
    assert "triggerStartAllFailSafeStop(\"pause fallback" in text
    assert "publishSafetyDamp" in text


def test_stop_command_uses_damp_and_native_verified_process_group_kill():
    text = source()
    assert "handleStopCommand" in text
    assert "triggerStartAllFailSafeStop(\"stop command" in text
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
