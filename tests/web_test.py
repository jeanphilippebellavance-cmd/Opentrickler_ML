"""
This script is created load the config.html as the Python REST frontend. The script will supply the JSON as the response to certain endpoints.

Usage

    python config_html_rest_test.py

Dependencies from pip:
 - flask
"""

import json
import time
from flask import Flask, request, send_from_directory
import os


script_directory = os.path.dirname(os.path.realpath(__file__))
web_portal_path = os.path.join(script_directory, '..', 'src', 'html', 'web_portal.html')
wizard_path = os.path.join(script_directory, '..', 'src', 'html', 'wizard.html')
static_folder = os.path.join(script_directory, '..', 'src', 'html')


app = Flask(__name__,
            static_url_path='',
            static_folder=static_folder)


current_weight = 0
current_charge_weight_set_point = 0

@app.route("/rest/charge_mode_state")
def rest_charge_mode_status():
    global current_charge_weight_set_point
    global current_weight

    event = 0

    # For every poll it increases the weight
    current_weight += 1.0

    if current_weight > current_charge_weight_set_point + 1:
        current_weight = 0

    if current_charge_weight_set_point == 0:
        state = 0
    elif current_weight < current_charge_weight_set_point:
        state = 1
    else:
        state = 3

    if (current_weight > current_charge_weight_set_point and current_charge_weight_set_point != 0):
        event = 2

    return {"s0": current_charge_weight_set_point,
            "s1": current_weight,
            "s2": state,
            "s3": event,
            "s4": "AR2208"}


@app.route("/rest/scale_action")
def rest_scale_action():
    return {"a0": 1}


@app.route("/rest/profile_summary")
def rest_profile_summary():
    return {"s0":{"0":"AR2208,gr","1":"AR2209,gr","2":"NewProfile2","3":"NewProfile3","4":"NewProfile4","5":"NewProfile5","6":"NewProfile6","7":"NewProfile7"},"s1":2}


@app.route('/rest/scale_config')
def rest_scale_config():
    return {"s0":0,"s1":2}


@app.route('/rest/profile_config')
def rest_profile_config():
    return {"pf":1,"p0":0,"p1":0,"p2":"AR2209,gr","p3":0.025,"p4":0.000,"p5":0.300,"p6":0.100,"p7":5.000,"p8":2.000,"p9":0.000,"p10":10.000,"p11":0.080,"p12":5.000}


@app.route('/rest/charge_mode_config')
def rest_charge_mode_config():
    return {"c1":"#00ff00","c2":"#ffff00","c3":"#ff0000","c4":"#0000ff","c5":3.000,"c6":0.030,"c7":0.020,"c8":0.020,"c9":0}


@app.route('/rest/cleanup_mode_state')
def rest_cleanup_mode_state():
    return {"s0":0,"s1":0.000}


@app.route('/rest/wireless_config')
def rest_wireless_config():
    return {"w0":"dummy_ssid","w2":"3","w3":30000,"w4":True}


@app.route('/rest/coarse_motor_config')
def rest_coarse_motor_config():
    return {"m0":50.000,"m1":200,"m2":800,"m3":256,"m4":5,"m5":110,"m6":0.100,"m7":1.2500000,"m8":False,"m9":False}


@app.route('/rest/fine_motor_config')
def rest_fine_motor_config():
    return {"m0":50.000,"m1":200,"m2":800,"m3":256,"m4":5,"m5":110,"m6":0.100,"m7":2.1052630,"m8":False,"m9":False}


@app.route('/rest/neopixel_led_config')
def rest_neopixel_led_config():
    return {"bl":"#ffffff","l1":"#404040","l2":"#404040"}


@app.route('/rest/button_config')
def rest_button_config():
    return {"b0":True}

@app.route('/rest/servo_gate_config')
def rest_servo_gate_config():
    return {"c0":True,"c1":0.050,"c2":0.125}



@app.route('/rest/system_control')
def rest_system_control():
    if ota_rebooting():
        return "rebooting", 503
    return {"s0":"8381FFF","s1":OTA_STATE["version"],"s2":"test-build","s3":"Debug","s4":False,"s5":False,"s6":False}


# --- OTA endpoint mocks -----------------------------------------------------
#
# Simulates the bootloader-managed OTA state machine so the web portal
# upload UI can be exercised without hardware:
#   session (begin/chunk/finalize/abort) -> apply -> simulated reboot window
#   (503 responses) -> boot_state trying -> confirmed with a bumped version.
#
# Environment knobs:
#   WEB_TEST_OTA_FAIL_OFFSET  chunk offset that fails once (default: none)
#   WEB_TEST_OTA_REBOOT_S     simulated reboot duration in seconds (default 6)
#   WEB_TEST_OTA_APPLY_RESULT confirmed | trying | bootsel (default confirmed)

OTA_MAX_IMAGE = 0x1F6000

OTA_STATE = {
    "version": "2026.07.12-beta.11",
    "fail_offset": int(os.environ.get("WEB_TEST_OTA_FAIL_OFFSET", "-1")),
    "fail_armed": True,
    "reboot_seconds": float(os.environ.get("WEB_TEST_OTA_REBOOT_S", "6")),
    "apply_result": os.environ.get("WEB_TEST_OTA_APPLY_RESULT", "confirmed"),
    "session_active": False,
    "session_verified": False,
    "expected_size": 0,
    "received_size": 0,
    "expected_crc32": 0,
    "boot_state": "none",
    "reboot_until": 0.0,
    "trying_polls": 0,
}


def ota_rebooting():
    """True while the mock is inside its simulated reboot window."""
    if OTA_STATE["reboot_until"] == 0.0:
        return False
    if OTA_STATE["apply_result"] == "bootsel":
        return True  # device never comes back; UI must hit its timeout path
    return time.monotonic() < OTA_STATE["reboot_until"]


def ota_status_payload(success=True, message="ok"):
    """Build a /rest/ota_status response mirroring the firmware JSON shape.

    Args:
        success: Value of the JSON "success" field.
        message: Value of the JSON "message" field.

    Returns:
        Dict matching the firmware's OTA status fields.
    """
    return {
        "success": success,
        "message": message,
        "supported": True,
        "transport": "rest_get_hex",
        "apply_supported": True,
        "bootloader_required": False,
        "bootloader_present": True,
        "boot_state": OTA_STATE["boot_state"],
        "active": OTA_STATE["session_active"],
        "verified": OTA_STATE["session_verified"],
        "metadata_valid": OTA_STATE["session_verified"] or OTA_STATE["boot_state"] != "none",
        "flash_size_bytes": 4 * 1024 * 1024,
        "primary_limit_bytes": OTA_MAX_IMAGE,
        "staging_offset": 0x200000,
        "image_offset": 0x201000,
        "capacity_bytes": 0x1FF000,
        "expected_size": OTA_STATE["expected_size"],
        "received_size": OTA_STATE["received_size"],
        "expected_crc32": "%08X" % OTA_STATE["expected_crc32"],
        "actual_crc32": "%08X" % OTA_STATE["expected_crc32"],
        "staged_size": OTA_STATE["expected_size"] if OTA_STATE["session_verified"] else 0,
        "staged_crc32": "%08X" % OTA_STATE["expected_crc32"],
        "last_error": "" if success else message,
    }


@app.route('/rest/ota_status')
def rest_ota_status():
    if ota_rebooting():
        return "rebooting", 503

    if OTA_STATE["boot_state"] == "trying":
        OTA_STATE["trying_polls"] += 1
        # The real confirm happens instantly at boot; hold "trying" briefly
        # so the UI renders that phase, unless the sim forces a stuck trial.
        if OTA_STATE["apply_result"] != "trying" and OTA_STATE["trying_polls"] >= 2:
            OTA_STATE["boot_state"] = "confirmed"
            OTA_STATE["version"] = OTA_STATE["version"] + "+new"

    return ota_status_payload()


@app.route('/rest/ota_begin')
def rest_ota_begin():
    size = int(request.args.get("size", "0"))
    if size <= 0 or size > OTA_MAX_IMAGE:
        return ota_status_payload(False, "image does not fit OTA slots")

    OTA_STATE["session_active"] = True
    OTA_STATE["session_verified"] = False
    OTA_STATE["expected_size"] = size
    OTA_STATE["received_size"] = 0
    OTA_STATE["expected_crc32"] = int(request.args.get("crc32", "0"), 0)
    OTA_STATE["fail_armed"] = True
    return ota_status_payload(True, "OTA upload started")


@app.route('/rest/ota_chunk')
def rest_ota_chunk():
    if not OTA_STATE["session_active"]:
        return ota_status_payload(False, "no OTA upload is active")

    offset = int(request.args.get("offset", "-1"))
    data = request.args.get("data", "")
    if offset != OTA_STATE["received_size"] or not data:
        return ota_status_payload(False, "chunk offset rejected")

    if offset == OTA_STATE["fail_offset"] and OTA_STATE["fail_armed"]:
        OTA_STATE["fail_armed"] = False
        return ota_status_payload(False, "page program failed")

    OTA_STATE["received_size"] += len(data) // 2
    return ota_status_payload(True, "chunk accepted")


@app.route('/rest/ota_finalize')
def rest_ota_finalize():
    if not OTA_STATE["session_active"]:
        return ota_status_payload(False, "no OTA upload is active")
    if OTA_STATE["received_size"] != OTA_STATE["expected_size"]:
        return ota_status_payload(False, "upload is incomplete")

    OTA_STATE["session_active"] = False
    OTA_STATE["session_verified"] = True
    return ota_status_payload(True, "image staged and verified")


@app.route('/rest/ota_abort')
def rest_ota_abort():
    OTA_STATE["session_active"] = False
    OTA_STATE["session_verified"] = False
    OTA_STATE["expected_size"] = 0
    OTA_STATE["received_size"] = 0
    return ota_status_payload(True, "OTA upload aborted")


@app.route('/rest/ota_apply')
def rest_ota_apply():
    if not OTA_STATE["session_verified"]:
        return ota_status_payload(False, "no verified staged firmware is available")

    OTA_STATE["session_verified"] = False
    OTA_STATE["boot_state"] = "trying"
    OTA_STATE["trying_polls"] = 0
    OTA_STATE["reboot_until"] = time.monotonic() + OTA_STATE["reboot_seconds"]
    return ota_status_payload(True, "install scheduled; device reboots into the bootloader")


def saved_ai_model():
    return {
        "valid": True,
        "enabled": True,
        "coarse_sample_count": 12,
        "fine_sample_count": 12,
        "fine_recovery_sample_count": 4,
        "coarse_best_speed_rps": 6.815,
        "coarse_best_flow_gps": 5.323,
        "coarse_tail_gn": 8.553,
        "coarse_trim_speed_rps": 5.630,
        "coarse_trim_flow_gps": 2.750,
        "coarse_trim_tail_gn": 8.498,
        "fine_best_speed_rps": 5.704,
        "fine_best_flow_gps": 1.086,
        "fine_tail_gn": 0.178,
        "fine_recovery_speed_rps": 0.080,
        "fine_recovery_flow_gps": 0.035,
        "fine_recovery_tail_gn": 0.061,
        "recommended_fine_window_gn": 0.960,
        "runtime_bias_gn": 0.0,
        "fine_fast_tail_gn": 0.341,
        "fine_fast_tail_confidence": 0.831,
        "fine_micro_tail_gn": 0.061,
        "fine_micro_tail_confidence": 0.808,
        "fine_tube_profile": "balanced",
        "machine_calibration": {
            "valid": True,
            "coarse_sample_count": 8,
            "fine_sample_count": 8,
            "scale_sample_period_ms": 80,
            "recommended_bulk_handoff_gn": 13.638,
            "recommended_trim_stop_gn": 2.258,
            "coarse_tail_p95_gn": 9.140,
            "fine_tail_p95_gn": 0.060,
        },
    }


@app.route('/rest/ai_tuning_status')
def rest_ai_tuning_status():
    return {
        "state": "idle",
        "is_active": False,
        "is_complete": False,
        "drops_completed": 0,
        "drops_max": 0,
        "progress_percent": 0,
        "profile_idx": 1,
        "requested_target_weight": 38.4,
        "status_message": "Ready",
        "error_message": "",
        "saved_model": saved_ai_model(),
        "working_model": {"valid": False, "enabled": False},
    }


@app.route('/rest/ai_tuning_history')
def rest_ai_tuning_history():
    return {
        "profile_idx": 1,
        "observation_count": 24,
        "model": saved_ai_model(),
        "coarse_samples": [],
        "fine_samples": [],
        "fine_recovery_samples": [],
        "runtime_stats": {
            "valid": True,
            "observation_count": 24,
            "fast_finish_count": 8,
            "fast_finish_over_rate": 0.875,
            "recovery_phase_count": 16,
            "recovery_over_rate": 0.375,
            "fast_finish_tail_p90_gn": 0.341,
        },
        "observations": [],
        "summary": {"matching_observations": 24, "avg_error_gn": 0.0527, "avg_time_ms": 7734},
    }


@app.route('/rest/ai_tuning_config')
def rest_ai_tuning_config():
    return {
        "coarse_budget_gn": 120,
        "fine_budget_gn": 36,
        "coarse_sample_count": 12,
        "fine_sample_count": 12,
        "coarse_sample_target_gn": 8.0,
        "fine_sample_target_gn": 1.75,
        "noise_margin": 0.05,
        "time_cost_weight": 1.0,
        "error_cost_weight": 8.0,
    }


@app.route("/")
def web_portal():
    with open(web_portal_path) as fp:
        page = fp.read()
    return page


@app.route('/wizard')
def web_wizard():
    with open(wizard_path) as fp:
        page = fp.read()
    return page


app.run(
    host="127.0.0.1",
    port=int(os.environ.get("WEB_TEST_PORT", "5000")),
    debug=False,
    use_reloader=False,
)
