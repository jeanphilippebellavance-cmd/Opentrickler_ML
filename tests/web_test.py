"""
This script is created load the config.html as the Python REST frontend. The script will supply the JSON as the response to certain endpoints.

Usage

    python config_html_rest_test.py

Dependencies from pip:
 - flask
"""

import json
from flask import Flask, send_from_directory
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
    return {"s0":"8381FFF","s1":"2026.07.12-beta.11","s2":"test-build","s3":"Debug","s4":False,"s5":False,"s6":False}


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
