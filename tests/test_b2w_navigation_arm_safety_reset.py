import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NAV_NODE = ROOT / "b2w_navigation_ws" / "src" / "main.cpp"


def source() -> str:
    return NAV_NODE.read_text(encoding="utf-8")


def test_emergency_stop_requests_arm_safety_reset_for_arm_related_states():
    text = source()
    assert "arm_reset_on_pause" in text
    assert "arm_safety_reset_timeout_seconds" in text
    assert "bool IsArmRelatedState() const" in text
    assert "void RequestArmSafetyReset" in text

    state_match = re.search(
        r"bool IsArmRelatedState\(\) const[\s\S]+?\n    void RequestArmSafetyReset",
        text,
    )
    assert state_match is not None
    state_body = state_match.group(0)
    assert "EXECUTING_ARM_TASK" in state_body
    assert "TRIGGERING_RELAY" in state_body
    assert "RESETTING_ARM" in state_body

    stop_match = re.search(
        r"void HandleEmergencyStop[\s\S]+?\n    void HandleEraseEmergencyStop",
        text,
    )
    assert stop_match is not None
    stop_body = stop_match.group(0)
    assert "RequestArmSafetyReset" in stop_body
    assert "Emergency stop" in stop_body or "emergency stop" in stop_body


def test_control_loop_polls_arm_safety_reset_before_paused_return():
    text = source()
    loop_match = re.search(
        r"void ControlLoop\(\)[\s\S]+?\n\s*constexpr double kAngularKp",
        text,
    )
    assert loop_match is not None
    loop_prefix = loop_match.group(0)
    assert "PollArmSafetyReset()" in loop_prefix
    assert loop_prefix.find("PollArmSafetyReset()") < loop_prefix.find("if (paused_)")
    paused_block = re.search(r"if \(paused_\) \{[\s\S]+?return;", loop_prefix)
    assert paused_block is not None
    assert "state_ =" not in paused_block.group(0)


def test_arm_safety_reset_is_async_deduplicated_and_times_out():
    text = source()
    request_match = re.search(
        r"void RequestArmSafetyReset[\s\S]+?\n    void PollArmSafetyReset",
        text,
    )
    assert request_match is not None
    request_body = request_match.group(0)
    assert "arm_safety_reset_requested_" in request_body
    assert "async_send_request" in request_body
    assert "z1_reset_arm_client_" in request_body
    assert "IsArmRelatedState()" in request_body

    poll_match = re.search(
        r"void PollArmSafetyReset[\s\S]+?\n    void HandleEmergencyStop",
        text,
    )
    assert poll_match is not None
    poll_body = poll_match.group(0)
    assert "wait_for(std::chrono::milliseconds(0))" in poll_body
    assert "arm_safety_reset_timeout_seconds_" in poll_body
    assert "arm_safety_reset_requested_ = false" in poll_body
