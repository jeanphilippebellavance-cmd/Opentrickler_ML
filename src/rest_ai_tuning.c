#include "rest_ai_tuning.h"
#include "ai_tuning.h"
#include "profile.h"
#include "http_rest.h"
#include "common.h"
#include "app.h"
#include "app_state.h"
#include "mini_12864_module.h"
#include "charge_mode.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <FreeRTOS.h>
#include <queue.h>

extern QueueHandle_t encoder_event_queue;
extern charge_mode_config_t charge_mode_config;

static char ai_tuning_json_buffer[24576];
static ai_tuning_session_t rest_session_copy;
static ai_profile_model_t rest_saved_model;
static ai_tuning_history_t rest_history_copy;

static bool send_buffer_overflow_error(struct fs_file *file) {
    static const char overflow_error[] = "HTTP/1.1 500 Internal Server Error\r\n"
        "Content-Type: application/json\r\n\r\n"
        "{\"success\":false,\"error\":\"Response buffer overflow\"}";
    file->data = overflow_error;
    file->len = sizeof(overflow_error) - 1;
    file->index = file->len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}

static bool finalize_json_response(struct fs_file *file, int len) {
    if (len < 0 || len >= (int)sizeof(ai_tuning_json_buffer)) {
        return send_buffer_overflow_error(file);
    }

    file->data = ai_tuning_json_buffer;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}

static bool append_jsonf(int *len, const char *fmt, ...) {
    if (len == NULL || *len < 0 || *len >= (int)sizeof(ai_tuning_json_buffer)) {
        return false;
    }

    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(ai_tuning_json_buffer + *len,
                            sizeof(ai_tuning_json_buffer) - (size_t)(*len),
                            fmt,
                            args);
    va_end(args);

    if (written < 0 || written >= (int)(sizeof(ai_tuning_json_buffer) - (size_t)(*len))) {
        return false;
    }

    *len += written;
    return true;
}

static void format_json_float(char *buffer, size_t buffer_len, float value, int decimals) {
    if (buffer == NULL || buffer_len == 0) {
        return;
    }
    if (!isfinite(value)) {
        snprintf(buffer, buffer_len, "null");
        return;
    }
    snprintf(buffer, buffer_len, "%.*f", decimals, value);
}

static const char* ai_tuning_state_to_string(ai_tuning_state_t state) {
    switch (state) {
        case AI_TUNING_IDLE:                    return "idle";
        case AI_TUNING_CHARACTERIZING_COARSE:   return "characterizing_coarse";
        case AI_TUNING_CHARACTERIZING_FINE:     return "characterizing_fine";
        case AI_TUNING_CALIBRATING_COARSE:      return "calibrating_coarse";
        case AI_TUNING_CALIBRATING_FINE:        return "calibrating_fine";
        case AI_TUNING_READY_TO_SAVE:           return "ready_to_save";
        case AI_TUNING_ERROR:                   return "error";
        default:                                return "unknown";
    }
}

static const char* ai_motor_mode_to_string(ai_motor_mode_t mode) {
    switch (mode) {
        case AI_MOTOR_MODE_COARSE_ONLY: return "coarse";
        case AI_MOTOR_MODE_FINE_ONLY:   return "fine";
        default:                        return "normal";
    }
}

static bool parse_steering_action(const char* value, ai_steering_action_t* action_out) {
    if (value == NULL || action_out == NULL) {
        return false;
    }
    if (strcmp(value, "faster") == 0) {
        *action_out = AI_STEERING_FASTER;
        return true;
    }
    if (strcmp(value, "safer") == 0) {
        *action_out = AI_STEERING_SAFER;
        return true;
    }
    if (strcmp(value, "fine_finish_faster") == 0) {
        *action_out = AI_STEERING_FINE_FINISH_FASTER;
        return true;
    }
    if (strcmp(value, "bulk_closer") == 0) {
        *action_out = AI_STEERING_BULK_CLOSER;
        return true;
    }
    if (strcmp(value, "undo_last") == 0) {
        *action_out = AI_STEERING_UNDO_LAST;
        return true;
    }
    return false;
}

static const char* flow_sample_safety_class(const ai_flow_sample_t* sample, bool coarse) {
    if (sample == NULL ||
        sample->speed_rps <= 0.0f ||
        sample->flow_gps <= 0.001f ||
        sample->delivered_weight <= 0.0f) {
        return "invalid";
    }

    float tail = fmaxf(sample->tail_weight, 0.0f);
    float tail_ratio = tail / fmaxf(sample->delivered_weight, 0.001f);
    if (coarse) {
        if (tail > 4.0f || tail_ratio > 0.55f) {
            return "blocked";
        }
        if (tail > 2.5f || tail_ratio > 0.35f) {
            return "caution";
        }
        return "safe";
    }

    if (tail > 0.55f || tail_ratio > 0.42f) {
        return "caution";
    }
    return "safe";
}

static bool append_model_json(int *len, const char *key, const ai_profile_model_t *model) {
    if (model == NULL) {
        return append_jsonf(len, ",\"%s\":null", key);
    }

    if (!append_jsonf(len,
                      ",\"%s\":{"
                      "\"valid\":%s,"
                      "\"enabled\":%s,"
                      "\"coarse_sample_count\":%u,"
                      "\"fine_sample_count\":%u,"
                      "\"fine_recovery_sample_count\":%u,"
                      "\"coarse_flow_slope\":%.6f,"
                      "\"coarse_flow_intercept\":%.6f,"
                      "\"coarse_tail_gn\":%.4f,"
                      "\"coarse_best_speed_rps\":%.4f,"
                      "\"coarse_best_flow_gps\":%.4f,"
                      "\"coarse_trim_speed_rps\":%.4f,"
                      "\"coarse_trim_flow_gps\":%.4f,"
                      "\"coarse_trim_tail_gn\":%.4f,"
                      "\"fine_flow_slope\":%.6f,"
                      "\"fine_flow_intercept\":%.6f,"
                      "\"fine_tail_gn\":%.4f,"
                      "\"fine_best_speed_rps\":%.4f,"
                      "\"fine_best_flow_gps\":%.4f,"
                      "\"fine_recovery_speed_rps\":%.4f,"
                      "\"fine_recovery_flow_gps\":%.4f,"
                      "\"fine_recovery_tail_gn\":%.4f,"
                      "\"recommended_fine_window_gn\":%.4f,"
                      "\"runtime_bias_gn\":%.4f,"
                      "\"fine_fast_flow_gps\":%.4f,"
                      "\"fine_fast_tail_gn\":%.4f,"
                      "\"fine_fast_tail_confidence\":%.4f,"
                      "\"fine_micro_flow_gps\":%.4f,"
                      "\"fine_micro_tail_gn\":%.4f,"
                      "\"fine_micro_tail_confidence\":%.4f,"
                      "\"fine_stop_safety_bias_gn\":%.4f,"
                      "\"fine_tube_profile_id\":%u,"
                      "\"fine_tube_profile\":\"%s\","
                      "\"steering_bulk_bias_gn\":%.4f,"
                      "\"steering_fine_bias_gn\":%.4f,"
                      "\"steering_recovery_speed_scale\":%.4f,"
                      "\"steering_undo_available\":%s,"
                      "\"steering_count\":%u,"
                      "\"machine_calibration\":{"
                      "\"valid\":%s,"
                      "\"coarse_sample_count\":%u,"
                      "\"fine_sample_count\":%u,"
                      "\"scale_sample_period_ms\":%.1f,"
                      "\"coarse_first_response_ms\":%.1f,"
                      "\"coarse_settle_ms\":%.1f,"
                      "\"coarse_tail_avg_gn\":%.4f,"
                      "\"coarse_tail_p95_gn\":%.4f,"
                      "\"coarse_uncertainty_gn\":%.4f,"
                      "\"coarse_open_loop_flow_gps\":%.4f,"
                      "\"trim_open_loop_flow_gps\":%.4f,"
                      "\"fine_first_response_ms\":%.1f,"
                      "\"fine_settle_ms\":%.1f,"
                      "\"fine_tail_avg_gn\":%.4f,"
                      "\"fine_tail_p95_gn\":%.4f,"
                      "\"fine_uncertainty_gn\":%.4f,"
                      "\"fine_open_loop_flow_gps\":%.4f,"
                      "\"micro_open_loop_flow_gps\":%.4f,"
                      "\"recommended_bulk_handoff_gn\":%.4f,"
                      "\"recommended_trim_stop_gn\":%.4f,"
                      "\"post_finish_watch_ms\":%.1f"
                      "}"
                      "}",
                      key,
                      model->valid ? "true" : "false",
                      model->enabled ? "true" : "false",
                      model->coarse_sample_count,
                      model->fine_sample_count,
                      model->fine_recovery_sample_count,
                      model->coarse_flow_slope,
                      model->coarse_flow_intercept,
                      model->coarse_tail_gn,
                      model->coarse_best_speed_rps,
                      model->coarse_best_flow_gps,
                      model->coarse_trim_speed_rps,
                      model->coarse_trim_flow_gps,
                      model->coarse_trim_tail_gn,
                      model->fine_flow_slope,
                      model->fine_flow_intercept,
                      model->fine_tail_gn,
                      model->fine_best_speed_rps,
                      model->fine_best_flow_gps,
                      model->fine_recovery_speed_rps,
                      model->fine_recovery_flow_gps,
                      model->fine_recovery_tail_gn,
                      model->recommended_fine_window_gn,
                      model->runtime_bias_gn,
                      model->fine_fast_flow_gps,
                      model->fine_fast_tail_gn,
                      model->fine_fast_tail_confidence,
                      model->fine_micro_flow_gps,
                      model->fine_micro_tail_gn,
                      model->fine_micro_tail_confidence,
                      model->fine_stop_safety_bias_gn,
                      model->fine_tube_profile,
                      ai_tuning_fine_tube_profile_to_string((ai_fine_tube_profile_t)model->fine_tube_profile),
                      model->steering_bulk_bias_gn,
                      model->steering_fine_bias_gn,
                      model->steering_recovery_speed_scale,
                      model->steering_undo_available ? "true" : "false",
                      model->steering_count,
                      model->machine.valid ? "true" : "false",
                      model->machine.coarse_sample_count,
                      model->machine.fine_sample_count,
                      model->machine.scale_sample_period_ms,
                      model->machine.coarse_first_response_ms,
                      model->machine.coarse_settle_ms,
                      model->machine.coarse_tail_avg_gn,
                      model->machine.coarse_tail_p95_gn,
                      model->machine.coarse_uncertainty_gn,
                      model->machine.coarse_open_loop_flow_gps,
                      model->machine.trim_open_loop_flow_gps,
                      model->machine.fine_first_response_ms,
                      model->machine.fine_settle_ms,
                      model->machine.fine_tail_avg_gn,
                      model->machine.fine_tail_p95_gn,
                      model->machine.fine_uncertainty_gn,
                      model->machine.fine_open_loop_flow_gps,
                      model->machine.micro_open_loop_flow_gps,
                      model->machine.recommended_bulk_handoff_gn,
                      model->machine.recommended_trim_stop_gn,
                      model->machine.post_finish_watch_ms)) {
        return false;
    }

    return true;
}

static bool append_sample_array(int *len, const char *key, const ai_flow_sample_t *samples, uint8_t count) {
    if (!append_jsonf(len, ",\"%s\":[", key)) {
        return false;
    }

    bool coarse = key != NULL && strstr(key, "coarse") != NULL;
    for (uint8_t idx = 0; idx < count; idx++) {
        float tail_ratio = samples[idx].tail_weight /
                           fmaxf(samples[idx].delivered_weight, 0.001f);
        if (!append_jsonf(len,
                          "%s{\"speed_rps\":%.4f,\"motor_on_time_ms\":%.1f,"
                          "\"delivered_weight\":%.4f,\"tail_weight\":%.4f,"
                          "\"tail_ratio\":%.4f,\"flow_gps\":%.4f,\"safety\":\"%s\"}",
                          idx == 0 ? "" : ",",
                          samples[idx].speed_rps,
                          samples[idx].motor_on_time_ms,
                          samples[idx].delivered_weight,
                          samples[idx].tail_weight,
                          tail_ratio,
                          samples[idx].flow_gps,
                          flow_sample_safety_class(&samples[idx], coarse))) {
            return false;
        }
    }

    return append_jsonf(len, "]");
}

bool http_rest_ai_tuning_start(struct fs_file *file, int num_params,
                               char *params[], char *values[]) {
    int profile_idx = -1;
    float target_weight = 30.0f;

    for (int idx = 0; idx < num_params; idx++) {
        if (strcmp(params[idx], "profile_idx") == 0) {
            profile_idx = atoi(values[idx]);
        }
        else if (strcmp(params[idx], "target") == 0) {
            target_weight = strtof(values[idx], NULL);
        }
    }

    if (profile_idx < 0 || profile_idx >= MAX_PROFILE_CNT) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Invalid profile_idx (must be 0-%d)\"}",
            http_json_header, MAX_PROFILE_CNT - 1);
        return finalize_json_response(file, len);
    }

    if (target_weight <= 0.0f) {
        target_weight = 30.0f;
    }

    profile_t* profile = profile_select((uint8_t)profile_idx);
    if (profile == NULL) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Failed to select profile\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    if (!ai_tuning_start(profile, target_weight)) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Failed to start AI characterization\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    charge_mode_config.target_charge_weight = target_weight;

    bool entering_charge_mode = !charge_mode_is_menu_active() ||
                                charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT;
    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_ZERO;
    if (entering_charge_mode) {
        exit_state = APP_STATE_ENTER_CHARGE_MODE_FROM_REST;
        ButtonEncoderEvent_t button_event = OVERRIDE_FROM_REST;
        if (encoder_event_queue == NULL ||
            xQueueSend(encoder_event_queue, &button_event, pdMS_TO_TICKS(100)) != pdTRUE) {
            ai_tuning_cancel();
            int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
                "%s{\"success\":false,\"error\":\"Failed to enter charge mode\"}",
                http_json_header);
            return finalize_json_response(file, len);
        }
    }

    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":true,\"message\":\"%s\",\"profile\":\"%s\",\"target_weight\":%.2f}",
        http_json_header,
        entering_charge_mode ? "AI characterization started - entering charge mode"
                             : "AI characterization started",
        profile->name,
        target_weight);
    return finalize_json_response(file, len);
}

bool http_rest_ai_machine_calibration_start(struct fs_file *file, int num_params,
                                            char *params[], char *values[]) {
    int profile_idx = -1;
    float target_weight = 40.0f;

    for (int idx = 0; idx < num_params; idx++) {
        if (strcmp(params[idx], "profile_idx") == 0) {
            profile_idx = atoi(values[idx]);
        }
        else if (strcmp(params[idx], "target") == 0) {
            target_weight = strtof(values[idx], NULL);
        }
    }

    if (profile_idx < 0 || profile_idx >= MAX_PROFILE_CNT) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Invalid profile_idx (must be 0-%d)\"}",
            http_json_header, MAX_PROFILE_CNT - 1);
        return finalize_json_response(file, len);
    }

    if (target_weight <= 0.0f) {
        target_weight = 40.0f;
    }

    profile_t* profile = profile_select((uint8_t)profile_idx);
    if (profile == NULL) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Failed to select profile\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    if (!ai_tuning_start_machine_calibration(profile, target_weight)) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Run and save powder characterization before machine calibration\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    charge_mode_config.target_charge_weight = target_weight;

    bool entering_charge_mode = !charge_mode_is_menu_active() ||
                                charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT;
    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_ZERO;
    if (entering_charge_mode) {
        exit_state = APP_STATE_ENTER_CHARGE_MODE_FROM_REST;
        ButtonEncoderEvent_t button_event = OVERRIDE_FROM_REST;
        if (encoder_event_queue == NULL ||
            xQueueSend(encoder_event_queue, &button_event, pdMS_TO_TICKS(100)) != pdTRUE) {
            ai_tuning_cancel();
            int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
                "%s{\"success\":false,\"error\":\"Failed to enter charge mode\"}",
                http_json_header);
            return finalize_json_response(file, len);
        }
    }

    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":true,\"message\":\"%s\",\"profile\":\"%s\",\"target_weight\":%.2f}",
        http_json_header,
        entering_charge_mode ? "OpenTrickler machine calibration started - entering charge mode"
                             : "OpenTrickler machine calibration started",
        profile->name,
        target_weight);
    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_status(struct fs_file *file, int num_params,
                                char *params[], char *values[]) {
    int requested_profile_idx = -1;
    for (int idx = 0; idx < num_params; idx++) {
        if (strcmp(params[idx], "profile_idx") == 0) {
            requested_profile_idx = atoi(values[idx]);
        }
    }

    memset(&rest_session_copy, 0, sizeof(rest_session_copy));
    if (!ai_tuning_get_session_copy(&rest_session_copy)) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"AI tuning state busy\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    memset(&rest_saved_model, 0, sizeof(rest_saved_model));
    bool is_active = (rest_session_copy.state == AI_TUNING_CHARACTERIZING_COARSE ||
                      rest_session_copy.state == AI_TUNING_CHARACTERIZING_FINE ||
                      rest_session_copy.state == AI_TUNING_CALIBRATING_COARSE ||
                      rest_session_copy.state == AI_TUNING_CALIBRATING_FINE);
    bool is_complete = (rest_session_copy.state == AI_TUNING_READY_TO_SAVE);
    uint8_t profile_idx = (requested_profile_idx >= 0 && requested_profile_idx < MAX_PROFILE_CNT)
        ? (uint8_t)requested_profile_idx
        : ((is_active || is_complete) && rest_session_copy.target_profile_idx < MAX_PROFILE_CNT
              ? rest_session_copy.target_profile_idx
              : (uint8_t)profile_get_selected_idx());
    bool have_saved_model = ai_tuning_get_enabled_model_copy(profile_idx, &rest_saved_model);
    ai_tuning_plan_t plan = {};
    bool have_plan = ai_tuning_get_active_plan(&plan);
    uint8_t progress = ai_tuning_get_progress_percent();

    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"state\":\"%s\",\"is_active\":%s,\"is_complete\":%s,\"drops_completed\":%u,\"drops_max\":%u,\"progress_percent\":%u,\"profile_idx\":%u,\"requested_target_weight\":%.4f,\"status_message\":\"%s\",\"error_message\":\"%s\"",
        http_json_header,
        ai_tuning_state_to_string(rest_session_copy.state),
        is_active ? "true" : "false",
        is_complete ? "true" : "false",
        rest_session_copy.drops_completed,
        rest_session_copy.total_samples_planned,
        progress,
        profile_idx,
        rest_session_copy.requested_target_weight,
        rest_session_copy.status_message,
        rest_session_copy.error_message);

    if (len < 0 || len >= (int)sizeof(ai_tuning_json_buffer)) {
        return send_buffer_overflow_error(file);
    }

    if (have_plan && plan.valid) {
        if (!append_jsonf(&len,
                          ",\"current_plan\":{\"motor_mode\":\"%s\",\"speed_rps\":%.4f,\"motor_on_time_ms\":%.1f,\"target_weight\":%.4f,\"stage_budget_used_gn\":%.4f,\"stage_budget_limit_gn\":%.4f,\"sample_index\":%u,\"total_samples\":%u,\"description\":\"%s\"}",
                          ai_motor_mode_to_string(plan.motor_mode),
                          plan.speed_rps,
                          plan.motor_on_time_ms,
                          plan.target_weight,
                          plan.stage_budget_used_gn,
                          plan.stage_budget_limit_gn,
                          plan.sample_index,
                          plan.total_samples,
                          plan.description)) {
            return send_buffer_overflow_error(file);
        }
    }

    if (rest_session_copy.drops_completed > 0) {
        uint8_t last_idx = (rest_session_copy.drop_write_idx + AI_TUNING_DROP_BUF_SIZE - 1) % AI_TUNING_DROP_BUF_SIZE;
        const ai_drop_telemetry_t* last = &rest_session_copy.drops[last_idx];
        if (!append_jsonf(&len,
                          ",\"last_drop\":{\"sample_number\":%u,\"motor_mode\":\"%s\",\"speed_rps\":%.4f,\"motor_on_time_ms\":%.1f,\"start_weight\":%.4f,\"stop_weight\":%.4f,\"final_weight\":%.4f,\"delivered_weight\":%.4f,\"tail_weight\":%.4f,\"target_weight\":%.4f,\"overthrow\":%.4f,\"coarse_time_ms\":%.1f,\"fine_time_ms\":%.1f,\"total_time_ms\":%.1f,\"first_response_time_ms\":%.1f,\"settle_time_ms\":%.1f,\"scale_sample_period_ms\":%.1f}",
                          last->sample_number,
                          ai_motor_mode_to_string(last->motor_mode),
                          last->speed_rps,
                          last->motor_on_time_ms,
                          last->start_weight,
                          last->stop_weight,
                          last->final_weight,
                          last->delivered_weight,
                          last->tail_weight,
                          last->target_weight,
                          last->overthrow,
                          last->coarse_time_ms,
                          last->fine_time_ms,
                          last->total_time_ms,
                          last->first_response_time_ms,
                          last->settle_time_ms,
                          last->scale_sample_period_ms)) {
            return send_buffer_overflow_error(file);
        }
    }

    if (!append_model_json(&len, "working_model", &rest_session_copy.working_model)) {
        return send_buffer_overflow_error(file);
    }

    if (have_saved_model) {
        if (!append_model_json(&len, "saved_model", &rest_saved_model)) {
            return send_buffer_overflow_error(file);
        }
    }
    else if (!append_jsonf(&len, ",\"saved_model\":null")) {
        return send_buffer_overflow_error(file);
    }

    if (!append_jsonf(&len, "}")) {
        return send_buffer_overflow_error(file);
    }

    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_apply(struct fs_file *file, int num_params,
                               char *params[], char *values[]) {
    (void)num_params;
    (void)params;
    (void)values;

    if (!ai_tuning_apply_params()) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"No completed AI model ready to save\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":true,\"message\":\"AI model saved for this profile\"}",
        http_json_header);
    return finalize_json_response(file, len);
}

bool http_rest_ai_steering(struct fs_file *file, int num_params,
                           char *params[], char *values[]) {
    int profile_idx = (int)profile_get_selected_idx();
    ai_steering_action_t action = AI_STEERING_FASTER;
    bool have_action = false;

    for (int idx = 0; idx < num_params; idx++) {
        if (strcmp(params[idx], "profile_idx") == 0) {
            profile_idx = atoi(values[idx]);
        }
        else if (strcmp(params[idx], "action") == 0) {
            have_action = parse_steering_action(values[idx], &action);
        }
    }

    if (profile_idx < 0 || profile_idx >= MAX_PROFILE_CNT) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Invalid profile_idx\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }
    if (!have_action) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Invalid steering action\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    if (!ai_tuning_apply_steering((uint8_t)profile_idx, action)) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"No active saved AI model to steer, or undo is unavailable\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    ai_profile_model_t model = {0};
    bool have_model = ai_tuning_get_enabled_model_copy((uint8_t)profile_idx, &model);
    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":true,\"action\":\"%s\",\"profile_idx\":%d",
        http_json_header,
        ai_tuning_steering_action_to_string(action),
        profile_idx);
    if (len < 0 || len >= (int)sizeof(ai_tuning_json_buffer)) {
        return send_buffer_overflow_error(file);
    }
    if (have_model) {
        if (!append_model_json(&len, "model", &model)) {
            return send_buffer_overflow_error(file);
        }
    }
    if (!append_jsonf(&len, "}")) {
        return send_buffer_overflow_error(file);
    }
    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_cancel(struct fs_file *file, int num_params,
                                char *params[], char *values[]) {
    (void)num_params;
    (void)params;
    (void)values;

    if (!ai_tuning_cancel()) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Failed to cancel AI tuning\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":true,\"message\":\"AI tuning cancelled\"}",
        http_json_header);
    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_history(struct fs_file *file, int num_params,
                                 char *params[], char *values[]) {
    int requested_profile_idx = (int)profile_get_selected_idx();
    for (int idx = 0; idx < num_params; idx++) {
        if (strcmp(params[idx], "profile_idx") == 0) {
            requested_profile_idx = atoi(values[idx]);
        }
    }

    if (requested_profile_idx < 0 || requested_profile_idx >= MAX_PROFILE_CNT) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"Invalid profile_idx\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    memset(&rest_history_copy, 0, sizeof(rest_history_copy));
    if (!ai_tuning_get_history_copy(&rest_history_copy)) {
        int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
            "%s{\"success\":false,\"error\":\"AI tuning history busy\"}",
            http_json_header);
        return finalize_json_response(file, len);
    }

    const ai_profile_model_t* model = &rest_history_copy.models[requested_profile_idx];
    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"profile_idx\":%d,\"observation_count\":%u",
        http_json_header,
        requested_profile_idx,
        rest_history_copy.observation_count);
    if (len < 0 || len >= (int)sizeof(ai_tuning_json_buffer)) {
        return send_buffer_overflow_error(file);
    }

    if (!append_model_json(&len, "model", model)) {
        return send_buffer_overflow_error(file);
    }
    if (!append_sample_array(&len, "coarse_samples", model->coarse_samples, model->coarse_sample_count)) {
        return send_buffer_overflow_error(file);
    }
    if (!append_sample_array(&len, "fine_samples", model->fine_samples, model->fine_sample_count)) {
        return send_buffer_overflow_error(file);
    }
    if (!append_sample_array(&len,
                             "fine_recovery_samples",
                             model->fine_recovery_samples,
                             model->fine_recovery_sample_count)) {
        return send_buffer_overflow_error(file);
    }

    if (!append_jsonf(&len, ",\"observations\":[")) {
        return send_buffer_overflow_error(file);
    }

    int oldest_idx = (rest_history_copy.observation_count >= AI_RUNTIME_OBSERVATION_COUNT)
        ? rest_history_copy.observation_next_idx
        : 0;
    bool first_observation = true;
    float avg_error = 0.0f;
    float avg_time = 0.0f;
    uint16_t matching_count = 0;

    for (int i = 0; i < rest_history_copy.observation_count; i++) {
        int obs_idx = (oldest_idx + i) % AI_RUNTIME_OBSERVATION_COUNT;
        const ai_runtime_observation_t* obs = &rest_history_copy.observations[obs_idx];
        if (obs->profile_idx != (uint8_t)requested_profile_idx) {
            continue;
        }

        avg_error += obs->final_error_gn;
        avg_time += obs->total_time_ms;
        matching_count++;

        char coarse_stop[16];
        char after_coarse[16];
        char coarse_tail[16];
        char fine_stop[16];
        char after_fine[16];
        char fine_tail[16];
        char post_finish_peak[16];
        char recovery_start[16];
        char recovery_end[16];
        char recovery_motor_on[16];
        format_json_float(coarse_stop, sizeof(coarse_stop), obs->coarse_stop_weight_gn, 4);
        format_json_float(after_coarse, sizeof(after_coarse), obs->after_coarse_settle_gn, 4);
        format_json_float(coarse_tail, sizeof(coarse_tail), obs->observed_coarse_tail_gn, 4);
        format_json_float(fine_stop, sizeof(fine_stop), obs->fine_stop_weight_gn, 4);
        format_json_float(after_fine, sizeof(after_fine), obs->after_fine_settle_gn, 4);
        format_json_float(fine_tail, sizeof(fine_tail), obs->observed_fine_tail_gn, 4);
        format_json_float(post_finish_peak, sizeof(post_finish_peak), obs->post_finish_peak_weight_gn, 4);
        format_json_float(recovery_start, sizeof(recovery_start), obs->recovery_start_weight_gn, 4);
        format_json_float(recovery_end, sizeof(recovery_end), obs->recovery_end_weight_gn, 4);
        format_json_float(recovery_motor_on, sizeof(recovery_motor_on), obs->recovery_motor_on_ms, 1);

        if (!append_jsonf(&len,
                          "%s{\"target_weight\":%.4f,\"final_error_gn\":%.4f,\"total_time_ms\":%.1f,"
                          "\"coarse_stop_weight_gn\":%s,\"after_coarse_settle_gn\":%s,"
                          "\"observed_coarse_tail_gn\":%s,\"fine_stop_weight_gn\":%s,"
                          "\"after_fine_settle_gn\":%s,\"observed_fine_tail_gn\":%s,"
                          "\"post_finish_peak_weight_gn\":%s,\"recovery_start_weight_gn\":%s,"
                          "\"recovery_end_weight_gn\":%s,\"recovery_motor_on_ms\":%s,"
                          "\"recovery_stall_count\":%u,\"recovery_exit_reason\":%u}",
                          first_observation ? "" : ",",
                          obs->target_weight,
                          obs->final_error_gn,
                          obs->total_time_ms,
                          coarse_stop,
                          after_coarse,
                          coarse_tail,
                          fine_stop,
                          after_fine,
                          fine_tail,
                          post_finish_peak,
                          recovery_start,
                          recovery_end,
                          recovery_motor_on,
                          obs->recovery_stall_count,
                          obs->recovery_exit_reason)) {
            return send_buffer_overflow_error(file);
        }
        first_observation = false;
    }

    if (!append_jsonf(&len, "]")) {
        return send_buffer_overflow_error(file);
    }

    if (matching_count > 0) {
        avg_error /= matching_count;
        avg_time /= matching_count;
    }

    if (!append_jsonf(&len,
                      ",\"summary\":{\"matching_observations\":%u,\"avg_error_gn\":%.4f,\"avg_time_ms\":%.1f}}",
                      matching_count,
                      avg_error,
                      avg_time)) {
        return send_buffer_overflow_error(file);
    }

    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_apply_refined(struct fs_file *file, int num_params,
                                       char *params[], char *values[]) {
    (void)num_params;
    (void)params;
    (void)values;

    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":false,\"error\":\"Deprecated endpoint. Use /rest/ai_tuning_apply to save the learned model.\"}",
        http_json_header);
    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_clear_history(struct fs_file *file, int num_params,
                                       char *params[], char *values[]) {
    (void)num_params;
    (void)params;
    (void)values;

    ai_tuning_clear_history();
    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":true,\"message\":\"AI history and saved models cleared\"}",
        http_json_header);
    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_config_get(struct fs_file *file, int num_params,
                                    char *params[], char *values[]) {
    (void)num_params;
    (void)params;
    (void)values;

    ai_tuning_config_t* cfg = ai_tuning_get_config();
    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"coarse_budget_gn\":%.2f,\"fine_budget_gn\":%.2f,\"coarse_sample_count\":%u,\"fine_sample_count\":%u,\"coarse_sample_target_gn\":%.2f,\"fine_sample_target_gn\":%.2f,\"noise_margin\":%.4f,\"time_cost_weight\":%.4f,\"error_cost_weight\":%.4f}",
        http_json_header,
        cfg->coarse_budget_gn,
        cfg->fine_budget_gn,
        cfg->coarse_sample_count,
        cfg->fine_sample_count,
        cfg->coarse_sample_target_gn,
        cfg->fine_sample_target_gn,
        cfg->noise_margin,
        cfg->time_cost_weight,
        cfg->error_cost_weight);
    return finalize_json_response(file, len);
}

bool http_rest_ai_tuning_config_set(struct fs_file *file, int num_params,
                                    char *params[], char *values[]) {
    ai_tuning_config_t* cfg = ai_tuning_get_config();

    for (int idx = 0; idx < num_params; idx++) {
        if (strcmp(params[idx], "coarse_budget_gn") == 0) {
            cfg->coarse_budget_gn = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "fine_budget_gn") == 0) {
            cfg->fine_budget_gn = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "coarse_sample_count") == 0) {
            cfg->coarse_sample_count = (uint8_t)atoi(values[idx]);
        }
        else if (strcmp(params[idx], "fine_sample_count") == 0) {
            cfg->fine_sample_count = (uint8_t)atoi(values[idx]);
        }
        else if (strcmp(params[idx], "coarse_sample_target_gn") == 0) {
            cfg->coarse_sample_target_gn = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "fine_sample_target_gn") == 0) {
            cfg->fine_sample_target_gn = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "noise_margin") == 0) {
            cfg->noise_margin = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "time_cost_weight") == 0) {
            cfg->time_cost_weight = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "error_cost_weight") == 0) {
            cfg->error_cost_weight = strtof(values[idx], NULL);
        }
    }

    cfg->coarse_budget_gn = fmaxf(20.0f, fminf(500.0f, cfg->coarse_budget_gn));
    cfg->fine_budget_gn = fmaxf(5.0f, fminf(150.0f, cfg->fine_budget_gn));
    cfg->coarse_sample_count = (uint8_t)fmaxf(2.0f, fminf((float)AI_TUNING_STAGE_SAMPLE_COUNT, (float)cfg->coarse_sample_count));
    cfg->fine_sample_count = (uint8_t)fmaxf(2.0f, fminf((float)AI_TUNING_STAGE_SAMPLE_COUNT, (float)cfg->fine_sample_count));
    cfg->coarse_sample_target_gn = fmaxf(2.0f, fminf(50.0f, cfg->coarse_sample_target_gn));
    cfg->fine_sample_target_gn = fmaxf(0.2f, fminf(10.0f, cfg->fine_sample_target_gn));
    cfg->noise_margin = fmaxf(0.005f, fminf(0.25f, cfg->noise_margin));
    cfg->time_cost_weight = fmaxf(0.1f, fminf(20.0f, cfg->time_cost_weight));
    cfg->error_cost_weight = fmaxf(0.1f, fminf(50.0f, cfg->error_cost_weight));

    ai_tuning_save_config();
    int len = snprintf(ai_tuning_json_buffer, sizeof(ai_tuning_json_buffer),
        "%s{\"success\":true,\"message\":\"AI configuration saved\"}",
        http_json_header);
    return finalize_json_response(file, len);
}

bool rest_ai_tuning_init(void) {
    rest_register_handler("/rest/ai_tuning_start", http_rest_ai_tuning_start);
    rest_register_handler("/rest/ai_machine_calibration_start", http_rest_ai_machine_calibration_start);
    rest_register_handler("/rest/ai_tuning_status", http_rest_ai_tuning_status);
    rest_register_handler("/rest/ai_tuning_apply", http_rest_ai_tuning_apply);
    rest_register_handler("/rest/ai_steering", http_rest_ai_steering);
    rest_register_handler("/rest/ai_tuning_cancel", http_rest_ai_tuning_cancel);
    rest_register_handler("/rest/ai_tuning_history", http_rest_ai_tuning_history);
    rest_register_handler("/rest/ai_tuning_apply_refined", http_rest_ai_tuning_apply_refined);
    rest_register_handler("/rest/ai_tuning_clear_history", http_rest_ai_tuning_clear_history);
    rest_register_handler("/rest/ai_tuning_config", http_rest_ai_tuning_config_get);
    rest_register_handler("/rest/ai_tuning_config_set", http_rest_ai_tuning_config_set);

    printf("AI Tuning REST endpoints registered\n");
    return true;
}
