#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>
#include <u8g2.h>
#include <math.h>

#include "app.h"
#include "FloatRingBuffer.h"
#include "mini_12864_module.h"
#include "display.h"
#include "scale.h"
#include "motors.h"
#include "charge_mode.h"
#include "eeprom.h"
#include "neopixel_led.h"
#include "profile.h"
#include "common.h"
#include "servo_gate.h"
#include "ai_tuning.h"


uint8_t charge_weight_digits[] = {0, 0, 0, 0, 0};

charge_mode_config_t charge_mode_config;

// Scale related
extern scale_config_t scale_config;
extern servo_gate_t servo_gate;


const eeprom_charge_mode_data_t default_charge_mode_data = {
    .charge_mode_data_rev = 0,

    .coarse_stop_threshold = 4,
    .fine_stop_threshold = 0.03,

    .set_point_sd_margin = 0.02,
    .set_point_mean_margin = 0.02,
    .coarse_stop_gate_ratio = 0,   // NEW
    .decimal_places = DP_2,

    // Precharges
    .precharge_enable = false,
    .precharge_time_ms = 1000,
    .precharge_speed_rps = 2,

    // AI tuning time targets: 10s is a good charge, 7-9s is an excellent one.
    .coarse_time_target_ms = 6000,
    .total_time_target_ms = 10000,

    // ML data collection disabled by default
    .ml_data_collection_enabled = false,

    // Auto zero disabled by default
    .auto_zero_on_cup_return = false,

    // Pulse mode defaults (disabled by default)
    .pulse_mode_enabled = false,
    .pulse_threshold = 0.5f,        // Start pulsing when within 0.5 grains (range: 0.3-1.0)
    .pulse_duration_ms = 30,        // 30ms motor burst
    .pulse_wait_ms = 150,           // 150ms wait for scale

    // Scale stabilization defaults
    .stabilization_enabled = false,     // adaptive by default
    .stabilization_time_ms = 2000,      // 2s fixed wait when enabled

    // LED related
    .neopixel_normal_charge_colour = RGB_COLOUR_GREEN,        // green
    .neopixel_under_charge_colour = RGB_COLOUR_YELLOW,        // yellow
    .neopixel_over_charge_colour = RGB_COLOUR_RED,            // red
    .neopixel_not_ready_colour = RGB_COLOUR_BLUE,             // blue
};

// Configures
TaskHandle_t scale_measurement_render_task_handler = NULL;
static char title_string[30];

static TickType_t charge_start_tick = 0;
static float last_charge_elapsed_seconds = 0.0f;
static volatile bool charge_mode_menu_active = false;
static bool last_charge_was_ai_tuning = false;
static bool last_charge_used_runtime_model = false;

// Deferred ML recording (set in charge loop, written during cup removal to avoid flash blocking)
static bool ml_record_pending = false;
static float ml_coarse_time_ms = 0.0f;
static float ml_fine_time_ms = 0.0f;

// Deferred AI tuning recording (same pattern - defer to cup removal for settled scale reading)
static bool ai_record_pending = false;
static ai_drop_telemetry_t ai_pending_telemetry;

// Menu system
extern AppState_t exit_state;
extern QueueHandle_t encoder_event_queue;
extern neopixel_led_config_t neopixel_led_config;


// Definitions
typedef enum {
    CHARGE_MODE_EVENT_NO_EVENT = (1 << 0),
    CHARGE_MODE_EVENT_UNDER_CHARGE = (1 << 1),
    CHARGE_MODE_EVENT_OVER_CHARGE = (1 << 2),
} ChargeModeEventBit_t;

typedef enum {
    RECOVERY_EXIT_NONE = 0,
    RECOVERY_EXIT_TOLERANCE = 1,
    RECOVERY_EXIT_GUARD_STOP = 2,
    RECOVERY_EXIT_TIMEOUT = 3,
    RECOVERY_EXIT_OVER = 4,
    RECOVERY_EXIT_ABORT = 5,
} RecoveryExitReason_t;

static const float SCALE_MAX_REASONABLE_WEIGHT_GN = 10000.0f;
static const uint8_t FINAL_WEIGHT_SETTLE_SAMPLE_COUNT = 5;
static const float CHARGE_RESULT_TOLERANCE_GN = 0.0205f;
static const uint32_t CHARGE_PROGRESS_ORANGE = 0xFF8000ul;

static bool last_final_weight_valid = false;
static float last_final_weight_gn = 0.0f;
static bool live_runtime_model_active = false;
static char live_charge_phase[24] = "idle";
static float live_final_target_gn = 0.0f;
static float live_bulk_handoff_margin_gn = 0.0f;
static float live_trim_stop_margin_gn = 0.0f;
static float live_fine_window_gn = 0.0f;
static float live_remaining_gn = 0.0f;
static float live_stop_margin_gn = 0.0f;
static uint32_t live_salvage_cycles = 0;
static float live_coarse_stop_weight_gn = NAN;
static float live_after_coarse_settle_weight_gn = NAN;
static float live_observed_coarse_tail_gn = NAN;
static float live_fine_stop_weight_gn = NAN;
static float live_after_fine_settle_weight_gn = NAN;
static float live_observed_fine_tail_gn = NAN;
static float live_post_finish_peak_weight_gn = NAN;
static float live_recovery_start_weight_gn = NAN;
static float live_recovery_end_weight_gn = NAN;
static uint32_t live_recovery_motor_on_ms = 0;
static uint32_t live_recovery_stall_count = 0;
static uint8_t live_recovery_exit_reason = RECOVERY_EXIT_NONE;
static uint32_t live_bulk_deadline_ms = 0;
static float live_coarse_command_rps = 0.0f;
static float live_fine_command_rps = 0.0f;
static char live_ai_decision[96] = "";
static float live_model_bulk_speed_rps = 0.0f;
static float live_model_bulk_flow_gps = 0.0f;
static float live_model_bulk_tail_gn = 0.0f;
static float live_model_fine_speed_rps = 0.0f;
static float live_model_fine_flow_gps = 0.0f;
static float live_model_fine_tail_gn = 0.0f;
static ai_tuning_session_t charge_mode_ai_session_snapshot;
static ai_profile_model_t charge_mode_runtime_model;
static ai_profile_model_t charge_mode_salvage_model;

static bool charge_mode_is_valid_scale_measurement(float measurement);
static bool charge_mode_is_cup_removed_measurement(float measurement);
static float charge_mode_acceptance_tolerance(void);

static void charge_mode_reset_live_metrics(void) {
    live_runtime_model_active = false;
    last_charge_used_runtime_model = false;
    snprintf(live_charge_phase, sizeof(live_charge_phase), "idle");
    live_final_target_gn = 0.0f;
    live_bulk_handoff_margin_gn = 0.0f;
    live_trim_stop_margin_gn = 0.0f;
    live_fine_window_gn = 0.0f;
    live_remaining_gn = 0.0f;
    live_stop_margin_gn = 0.0f;
    live_salvage_cycles = 0;
    live_coarse_stop_weight_gn = NAN;
    live_after_coarse_settle_weight_gn = NAN;
    live_observed_coarse_tail_gn = NAN;
    live_fine_stop_weight_gn = NAN;
    live_after_fine_settle_weight_gn = NAN;
    live_observed_fine_tail_gn = NAN;
    live_post_finish_peak_weight_gn = NAN;
    live_recovery_start_weight_gn = NAN;
    live_recovery_end_weight_gn = NAN;
    live_recovery_motor_on_ms = 0;
    live_recovery_stall_count = 0;
    live_recovery_exit_reason = RECOVERY_EXIT_NONE;
    live_bulk_deadline_ms = 0;
    live_coarse_command_rps = 0.0f;
    live_fine_command_rps = 0.0f;
    live_ai_decision[0] = '\0';
    live_model_bulk_speed_rps = 0.0f;
    live_model_bulk_flow_gps = 0.0f;
    live_model_bulk_tail_gn = 0.0f;
    live_model_fine_speed_rps = 0.0f;
    live_model_fine_flow_gps = 0.0f;
    live_model_fine_tail_gn = 0.0f;
}

static void charge_mode_command_motor(motor_select_t selected_motor, float new_velocity) {
    if (selected_motor == SELECT_COARSE_TRICKLER_MOTOR || selected_motor == SELECT_BOTH_MOTOR) {
        live_coarse_command_rps = new_velocity;
    }
    if (selected_motor == SELECT_FINE_TRICKLER_MOTOR || selected_motor == SELECT_BOTH_MOTOR) {
        live_fine_command_rps = new_velocity;
    }

    motor_set_speed(selected_motor, new_velocity);
}

static void charge_mode_set_live_phase(const char* phase,
                                       float remaining_gn,
                                       float stop_margin_gn) {
    snprintf(live_charge_phase,
             sizeof(live_charge_phase),
             "%s",
             phase != NULL ? phase : "unknown");
    live_remaining_gn = remaining_gn;
    live_stop_margin_gn = stop_margin_gn;
}

static void charge_mode_set_result_colour(rgbw_u32_t colour) {
    neopixel_led_set_colour(colour, colour, colour, true);
}

static rgbw_u32_t charge_mode_make_colour(uint32_t raw_colour) {
    rgbw_u32_t colour = {};
    colour._raw_colour = raw_colour;
    return colour;
}

static void charge_mode_set_progress_colour(float remaining_weight,
                                            float target_weight) {
    if (target_weight <= 0.0f || !isfinite(remaining_weight)) {
        return;
    }

    rgbw_u32_t colour = {};
    if (remaining_weight > fmaxf(target_weight * 0.12f, 4.0f)) {
        colour = charge_mode_make_colour(CHARGE_PROGRESS_ORANGE);
    }
    else if (remaining_weight > charge_mode_acceptance_tolerance()) {
        colour = charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour;
    }
    else {
        return;
    }

    neopixel_led_set_colour(colour, colour, colour, false);
}

static float charge_mode_choose_result_measurement(float settled_weight,
                                                   bool settled_valid,
                                                   float live_weight) {
    return settled_valid ? settled_weight : live_weight;
}

static void charge_mode_update_true_final_measurement(float measurement) {
    if (!charge_mode_is_valid_scale_measurement(measurement)) {
        return;
    }

    float target_weight = charge_mode_config.target_charge_weight;
    bool ai_sample_measurement =
        (last_charge_was_ai_tuning || ai_record_pending) &&
        ai_pending_telemetry.target_weight > 0.0f;
    if (ai_sample_measurement) {
        target_weight = ai_pending_telemetry.target_weight;
    }

    if (target_weight > 0.0f && !ai_sample_measurement &&
        measurement < target_weight * 0.50f) {
        return;
    }
    if (target_weight > 0.0f) {
        float plausible_over_limit = ai_sample_measurement
            ? fmaxf(80.0f, target_weight * 5.0f)
            : fmaxf(15.0f, target_weight * 0.50f);
        if (measurement > target_weight + plausible_over_limit) {
            return;
        }
        if (last_final_weight_valid) {
            float plausible_jump_limit = ai_sample_measurement
                ? plausible_over_limit
                : fmaxf(3.0f, target_weight * 0.12f);
            if (measurement > last_final_weight_gn + plausible_jump_limit) {
                return;
            }
        }
    }

    if (!isfinite(live_post_finish_peak_weight_gn) ||
        measurement > live_post_finish_peak_weight_gn) {
        live_post_finish_peak_weight_gn = measurement;
    }

    if (!last_final_weight_valid || measurement > last_final_weight_gn) {
        last_final_weight_gn = measurement;
        last_final_weight_valid = true;
    }

    // Keep final/peak truth separate from phase tail telemetry. Fine tail is
    // captured by mark_fine_stop() immediately after the motor stops; updating
    // it here would fold active recovery delivery into "tail" and poison the
    // learned finish model.
}

static void charge_mode_apply_result_state(bool suppress_charge_result,
                                           bool measurement_valid,
                                           float measured_weight,
                                           float tolerance) {
    if (suppress_charge_result || !measurement_valid) {
        charge_mode_set_result_colour(charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour);
        charge_mode_config.charge_mode_event &= ~(CHARGE_MODE_EVENT_UNDER_CHARGE | CHARGE_MODE_EVENT_OVER_CHARGE);
        return;
    }

    float error = charge_mode_config.target_charge_weight - measured_weight;
    if (error < -tolerance) {
        charge_mode_set_result_colour(charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour);
        charge_mode_config.charge_mode_event &= ~CHARGE_MODE_EVENT_UNDER_CHARGE;
        charge_mode_config.charge_mode_event |= CHARGE_MODE_EVENT_OVER_CHARGE;
    }
    else if (error > tolerance) {
        charge_mode_set_result_colour(charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour);
        charge_mode_config.charge_mode_event &= ~CHARGE_MODE_EVENT_OVER_CHARGE;
        charge_mode_config.charge_mode_event |= CHARGE_MODE_EVENT_UNDER_CHARGE;
    }
    else {
        charge_mode_set_result_colour(charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour);
        charge_mode_config.charge_mode_event &= ~(CHARGE_MODE_EVENT_UNDER_CHARGE | CHARGE_MODE_EVENT_OVER_CHARGE);
    }
}

static bool charge_mode_is_valid_scale_measurement(float measurement) {
    return isfinite(measurement) && fabsf(measurement) < SCALE_MAX_REASONABLE_WEIGHT_GN;
}

static float charge_mode_acceptance_tolerance(void) {
    return CHARGE_RESULT_TOLERANCE_GN;
}

static bool charge_mode_is_underload_measurement(float measurement) {
    return isfinite(measurement) && measurement <= -SCALE_MAX_REASONABLE_WEIGHT_GN;
}

static bool charge_mode_is_cup_removed_measurement(float measurement) {
    return charge_mode_is_valid_scale_measurement(measurement) &&
           measurement + 10.0f < charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin;
}

static bool charge_mode_try_get_current_measurement(float* measurement_out) {
    if (measurement_out == NULL) {
        return false;
    }

    float measurement = scale_get_current_measurement();
    if (!charge_mode_is_valid_scale_measurement(measurement)) {
        return false;
    }

    *measurement_out = measurement;
    return true;
}

static bool charge_mode_capture_settled_measurement(uint32_t max_wait_ms,
                                                    uint8_t sample_count,
                                                    float* measurement_out) {
    if (measurement_out == NULL) {
        return false;
    }

    if (sample_count == 0) {
        sample_count = 1;
    }

    FloatRingBuffer settle_buffer(sample_count);
    TickType_t start_tick = xTaskGetTickCount();

    while ((uint32_t)((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS) < max_wait_ms) {
        float measurement = 0.0f;
        if (scale_block_wait_for_next_measurement(200, &measurement) &&
            charge_mode_is_valid_scale_measurement(measurement)) {
            settle_buffer.enqueue(measurement);
            if (settle_buffer.getCounter() >= sample_count &&
                settle_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin) {
                *measurement_out = settle_buffer.getMean();
                return true;
            }
        }
    }

    if (settle_buffer.getCounter() > 0) {
        *measurement_out = settle_buffer.getMean();
        return true;
    }

    return charge_mode_try_get_current_measurement(measurement_out);
}

static bool charge_mode_force_zero_and_wait_for_ai(void) {
    if (scale_config.scale_handle == NULL ||
        scale_config.scale_handle->force_zero == NULL) {
        return false;
    }

    snprintf(title_string, sizeof(title_string), "AI Taring...");
    charge_mode_set_live_phase("ai_tare", 0.0f, 0.0f);
    charge_mode_set_result_colour(charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour);

    scale_config.scale_handle->force_zero();
    vTaskDelay(pdMS_TO_TICKS(350));

    FloatRingBuffer tare_buffer(5);
    TickType_t start_tick = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(2500);
    while ((xTaskGetTickCount() - start_tick) < timeout_ticks) {
        if (button_wait_for_input(false) == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return false;
        }

        float measurement = 0.0f;
        if (scale_block_wait_for_next_measurement(250, &measurement) &&
            charge_mode_is_valid_scale_measurement(measurement)) {
            tare_buffer.enqueue(measurement);
            if (tare_buffer.getCounter() >= 5 &&
                tare_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin &&
                fabsf(tare_buffer.getMean()) <
                    fmaxf(charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin, 0.030f)) {
                return true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return false;
}

static void charge_mode_format_json_weight(char* buffer, size_t buffer_len, float measurement) {
    if (buffer == NULL || buffer_len == 0) {
        return;
    }

    if (!charge_mode_is_valid_scale_measurement(measurement) || isnanf(measurement)) {
        snprintf(buffer, buffer_len, "\"nan\"");
    }
    else if (isinff(measurement)) {
        snprintf(buffer, buffer_len, "\"inf\"");
    }
    else {
        snprintf(buffer, buffer_len, "%0.3f", measurement);
    }
}

static float charge_mode_measure_positive_momentum(float latest,
                                                   float previous,
                                                   float older) {
    float momentum = 0.0f;

    if (charge_mode_is_valid_scale_measurement(latest) &&
        charge_mode_is_valid_scale_measurement(previous)) {
        momentum = fmaxf(momentum, latest - previous);
    }

    if (charge_mode_is_valid_scale_measurement(previous) &&
        charge_mode_is_valid_scale_measurement(older)) {
        momentum = fmaxf(momentum, previous - older);
    }

    return fmaxf(momentum, 0.0f);
}

static float charge_mode_estimate_positive_flow_gps(float latest,
                                                    float previous,
                                                    float older,
                                                    float latest_dt_s,
                                                    float previous_dt_s) {
    float flow = 0.0f;

    if (latest_dt_s > 0.02f &&
        charge_mode_is_valid_scale_measurement(latest) &&
        charge_mode_is_valid_scale_measurement(previous)) {
        flow = fmaxf(flow, (latest - previous) / latest_dt_s);
    }

    if (previous_dt_s > 0.02f &&
        charge_mode_is_valid_scale_measurement(previous) &&
        charge_mode_is_valid_scale_measurement(older)) {
        flow = fmaxf(flow, (previous - older) / previous_dt_s);
    }

    return fmaxf(flow, 0.0f);
}

static float charge_mode_estimate_scale_period_seconds(float latest_dt_s,
                                                       float previous_dt_s) {
    float period_s = latest_dt_s;
    if (period_s <= 0.02f) {
        period_s = previous_dt_s;
    }
    if (period_s <= 0.02f) {
        period_s = 0.75f;
    }
    return fmaxf(0.25f, fminf(period_s, 1.20f));
}

static float ai_model_predict_flow_gps(const ai_profile_model_t* model,
                                       ai_motor_mode_t motor_mode,
                                       float speed_rps) {
    if (model == NULL) {
        return 0.0f;
    }

    float slope = 0.0f;
    float intercept = 0.0f;
    uint8_t count = 0;
    const ai_flow_sample_t* samples = NULL;
    if (motor_mode == AI_MOTOR_MODE_FINE_ONLY) {
        slope = model->fine_flow_slope;
        intercept = model->fine_flow_intercept;
        count = model->fine_sample_count;
        samples = model->fine_samples;
    }
    else {
        slope = model->coarse_flow_slope;
        intercept = model->coarse_flow_intercept;
        count = model->coarse_sample_count;
        samples = model->coarse_samples;
    }

    float flow = slope * speed_rps + intercept;
    if (flow > 0.0f) {
        return flow;
    }

    if (count == 0 || samples == NULL) {
        return 0.0f;
    }

    uint8_t nearest_idx = 0;
    float nearest_delta = fabsf(samples[0].speed_rps - speed_rps);
    for (uint8_t idx = 1; idx < count; idx++) {
        float delta = fabsf(samples[idx].speed_rps - speed_rps);
        if (delta < nearest_delta) {
            nearest_delta = delta;
            nearest_idx = idx;
        }
    }

    float sample_speed = fmaxf(samples[nearest_idx].speed_rps, 0.1f);
    return fmaxf(0.0f, samples[nearest_idx].flow_gps * (speed_rps / sample_speed));
}

static float ai_model_estimate_speed_for_fine_flow(const ai_profile_model_t* model,
                                                   float target_flow_gps,
                                                   float reference_speed_rps,
                                                   float reference_flow_gps,
                                                   float min_speed_rps,
                                                   float max_speed_rps) {
    if (target_flow_gps <= 0.0f) {
        return min_speed_rps;
    }

    float speed = 0.0f;
    if (model != NULL && model->fine_flow_slope > 0.001f) {
        speed = (target_flow_gps - model->fine_flow_intercept) / model->fine_flow_slope;
    }
    if ((!isfinite(speed) || speed <= 0.0f) &&
        reference_speed_rps > 0.0f &&
        reference_flow_gps > 0.005f) {
        speed = reference_speed_rps * (target_flow_gps / reference_flow_gps);
    }

    if (!isfinite(speed) || speed <= 0.0f) {
        speed = min_speed_rps;
    }
    return fmaxf(min_speed_rps, fminf(speed, max_speed_rps));
}

static bool ai_model_choose_flow_sample(const ai_flow_sample_t* samples,
                                        uint8_t count,
                                        bool prefer_trim,
                                        float noise_margin,
                                        float* speed_rps,
                                        float* flow_gps,
                                        float* tail_gn) {
    if (samples == NULL || count == 0 ||
        speed_rps == NULL || flow_gps == NULL || tail_gn == NULL) {
        return false;
    }

    bool found = false;
    float best_score = prefer_trim ? 1000000.0f : -1000000.0f;
    for (uint8_t idx = 0; idx < count; idx++) {
        const ai_flow_sample_t* sample = &samples[idx];
        if (sample->delivered_weight <= noise_margin * 2.0f ||
            sample->flow_gps <= 0.001f ||
            sample->speed_rps <= 0.0f) {
            continue;
        }

        float tail = fmaxf(sample->tail_weight, 0.0f);
        float flow = fmaxf(sample->flow_gps, 0.001f);
        float delivered = fmaxf(sample->delivered_weight, 0.0f);
        if (!prefer_trim &&
            (tail > 4.0f ||
             (delivered > 0.0f && tail > delivered * 0.55f))) {
            continue;
        }
        float score = prefer_trim
            ? (tail * 10.0f + (1.0f / flow))
            : (flow / (1.0f + tail * 0.35f));

        if ((!found) ||
            (prefer_trim && score < best_score) ||
            (!prefer_trim && score > best_score)) {
            found = true;
            best_score = score;
            *speed_rps = sample->speed_rps;
            *flow_gps = sample->flow_gps;
            *tail_gn = tail;
        }
    }

    if (!found && !prefer_trim) {
        best_score = 1000000.0f;
        for (uint8_t idx = 0; idx < count; idx++) {
            const ai_flow_sample_t* sample = &samples[idx];
            if (sample->delivered_weight <= noise_margin * 2.0f ||
                sample->flow_gps <= 0.001f ||
                sample->speed_rps <= 0.0f) {
                continue;
            }

            float tail = fmaxf(sample->tail_weight, 0.0f);
            float flow = fmaxf(sample->flow_gps, 0.001f);
            float score = tail * 10.0f + (1.0f / flow);
            if (!found || score < best_score) {
                found = true;
                best_score = score;
                *speed_rps = sample->speed_rps;
                *flow_gps = flow;
                *tail_gn = tail;
            }
        }
    }

    return found;
}

static float ai_model_estimate_coarse_tail_guard(const ai_profile_model_t* model,
                                                 float average_tail_gn,
                                                 float target_weight_gn) {
    float max_tail = fmaxf(average_tail_gn, 0.0f);

    if (model != NULL) {
        for (uint8_t idx = 0; idx < model->coarse_sample_count; idx++) {
            const ai_flow_sample_t* sample = &model->coarse_samples[idx];
            if (sample->delivered_weight <= 0.0f ||
                sample->flow_gps <= 0.001f ||
                sample->speed_rps <= 0.0f) {
                continue;
            }
            max_tail = fmaxf(max_tail, fmaxf(sample->tail_weight, 0.0f));
        }
    }

    float guard_tail = fmaxf(fmaxf(average_tail_gn, 0.0f) * 1.02f,
                             max_tail * 0.72f);
    float guard_cap = fmaxf(4.50f, target_weight_gn * 0.28f);
    return fmaxf(0.0f, fminf(guard_tail, guard_cap));
}

static uint32_t charge_mode_compute_open_loop_stop_ms(float remaining_at_start_gn,
                                                      float stop_margin_gn,
                                                      float flow_gps,
                                                      float guard_seconds) {
    if (flow_gps <= 0.05f || remaining_at_start_gn <= stop_margin_gn) {
        return 0;
    }

    float run_seconds = (remaining_at_start_gn - stop_margin_gn) / flow_gps;
    run_seconds -= fmaxf(guard_seconds, 0.0f);
    if (run_seconds <= 0.0f) {
        return 0;
    }

    return (uint32_t)lroundf(fminf(run_seconds * 1000.0f, 8000.0f));
}


static void format_elapsed_time(char *buffer, size_t len, TickType_t start_tick) {
    TickType_t now = xTaskGetTickCount();
    uint32_t elapsed_ticks = now - start_tick;

    // Tick to milliseconds
    float elapsed_seconds = (float)(elapsed_ticks * portTICK_PERIOD_MS) / 1000.0f;

    snprintf(buffer, len, "%.2f s", elapsed_seconds);
}

static bool charge_mode_salvage_undercharge(float* measurement_io) {
    if (measurement_io == NULL ||
        last_charge_was_ai_tuning ||
        last_charge_used_runtime_model ||
        !charge_mode_is_valid_scale_measurement(*measurement_io)) {
        return false;
    }

    const float target_weight = charge_mode_config.target_charge_weight;
    const float stop_threshold = charge_mode_acceptance_tolerance();
    float under_weight = target_weight - *measurement_io;
    if (target_weight <= 0.0f || under_weight <= stop_threshold) {
        return false;
    }
    if (under_weight > 0.30f) {
        return false;
    }

    profile_t* current_profile = profile_get_selected();
    if (current_profile == NULL) {
        return false;
    }

    const float fine_motor_max_speed = fminf((float)get_motor_max_speed(SELECT_FINE_TRICKLER_MOTOR), 6.0f);
    const float profile_fine_max_speed = current_profile->fine_max_flow_speed_rps;
    const float fine_trickler_min_speed = fmaxf(get_motor_min_speed(SELECT_FINE_TRICKLER_MOTOR),
                                                current_profile->fine_min_flow_speed_rps);

    memset(&charge_mode_salvage_model, 0, sizeof(charge_mode_salvage_model));
    ai_profile_model_t& model = charge_mode_salvage_model;
    bool have_model = ai_tuning_get_enabled_model_copy((uint8_t)profile_get_selected_idx(), &model) &&
                      model.valid &&
                      model.enabled;
    const float fine_trickler_max_speed = have_model
        ? fine_motor_max_speed
        : fminf(fine_motor_max_speed, profile_fine_max_speed);
    if (fine_trickler_max_speed <= 0.0f) {
        return false;
    }
    float salvage_max_speed = have_model && model.fine_best_speed_rps > 0.0f
        ? model.fine_best_speed_rps * 0.95f
        : fmaxf(fine_trickler_min_speed, fine_trickler_max_speed * 0.75f);
    salvage_max_speed = fmaxf(fine_trickler_min_speed,
                              fminf(salvage_max_speed, fine_trickler_max_speed));
    float salvage_min_speed = fmaxf(fine_trickler_min_speed,
                                    fminf(salvage_max_speed,
                                          fmaxf(fine_trickler_min_speed * 1.10f,
                                                fine_trickler_max_speed * 0.18f)));

    float reference_flow = have_model
        ? ai_model_predict_flow_gps(&model, AI_MOTOR_MODE_FINE_ONLY, salvage_max_speed)
        : 0.0f;
    if (reference_flow <= 0.005f && have_model &&
        model.fine_best_flow_gps > 0.005f &&
        model.fine_best_speed_rps > 0.0f) {
        reference_flow = model.fine_best_flow_gps *
                         (salvage_max_speed / model.fine_best_speed_rps);
    }
    if (reference_flow <= 0.005f) {
        reference_flow = 0.08f;
    }

    const float fine_tail = have_model ? fmaxf(model.fine_tail_gn, 0.0f) : 0.0f;
    const float bias_margin = have_model ? fminf(fmaxf(model.runtime_bias_gn, 0.0f), 0.35f) * 0.15f : 0.0f;

    snprintf(title_string, sizeof(title_string), "Fine Finish");
    if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
        servo_gate_set_ratio(SERVO_GATE_RATIO_OPEN, false);
    }

    bool attempted = false;
    TickType_t salvage_start_tick = 0;
    TickType_t loop_start_tick = xTaskGetTickCount();
    float expected_flow = fmaxf(reference_flow * 0.75f, 0.025f);
    uint32_t salvage_budget_ms = (uint32_t)lroundf(fmaxf(4000.0f,
                                                         fminf(25000.0f,
                                                               (under_weight / expected_flow) * 1800.0f + 2500.0f)));
    int scale_fail_count = 0;
    float previous_weight = *measurement_io;
    float older_weight = *measurement_io;
    float best_under_weight = under_weight;
    TickType_t last_progress_tick = loop_start_tick;

    while ((xTaskGetTickCount() - loop_start_tick) < pdMS_TO_TICKS(salvage_budget_ms)) {
        if (button_wait_for_input(false) == BUTTON_RST_PRESSED) {
            charge_mode_command_motor(SELECT_FINE_TRICKLER_MOTOR, 0);
            if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
                servo_gate_set_ratio(SERVO_GATE_RATIO_CLOSED, true);
            }
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return true;
        }

        under_weight = target_weight - *measurement_io;
        if (under_weight <= stop_threshold) {
            break;
        }
        if (under_weight <= 0.0f) {
            break;
        }

        if (under_weight < best_under_weight - 0.01f) {
            best_under_weight = under_weight;
            last_progress_tick = xTaskGetTickCount();
        }

        float control_span = fmaxf(0.12f, fminf(1.40f, under_weight * 1.8f));
        float curve_position = fmaxf(0.0f, fminf(1.0f, under_weight / control_span));
        float urgency_boost = 1.0f;
        if (under_weight > 0.80f) {
            urgency_boost = 1.0f;
        }
        else if (under_weight > 0.35f) {
            urgency_boost = 0.88f;
        }
        else if (under_weight > 0.12f) {
            urgency_boost = 0.72f;
        }
        else {
            urgency_boost = 0.55f;
        }
        float salvage_speed = salvage_min_speed +
            (salvage_max_speed - salvage_min_speed) * curve_position * curve_position * urgency_boost;
        salvage_speed = fmaxf(salvage_min_speed, fminf(salvage_speed, salvage_max_speed));

        if ((xTaskGetTickCount() - last_progress_tick) > pdMS_TO_TICKS(2200) &&
            under_weight > stop_threshold * 2.0f) {
            salvage_speed = salvage_max_speed;
        }

        float salvage_flow = have_model
            ? ai_model_predict_flow_gps(&model, AI_MOTOR_MODE_FINE_ONLY, salvage_speed)
            : 0.0f;
        if (salvage_flow <= 0.005f && salvage_max_speed > 0.0f) {
            salvage_flow = reference_flow * (salvage_speed / salvage_max_speed);
        }
        salvage_flow = fmaxf(salvage_flow, 0.01f);

        float momentum_margin = charge_mode_measure_positive_momentum(*measurement_io,
                                                                      previous_weight,
                                                                      older_weight) * 0.45f;
        float stop_margin = fine_tail * 0.35f +
                            salvage_flow * 0.14f +
                            bias_margin +
                            momentum_margin;
        stop_margin = fmaxf(stop_threshold * 1.50f,
                            fminf(0.08f, stop_margin));

        charge_mode_set_live_phase("salvage", under_weight, stop_margin);
        if (under_weight <= stop_margin) {
            break;
        }

        if (!attempted) {
            salvage_start_tick = xTaskGetTickCount();
        }

        attempted = true;
        live_salvage_cycles++;
        charge_mode_command_motor(SELECT_FINE_TRICKLER_MOTOR, salvage_speed);

        float latest_weight = *measurement_io;
        if (scale_block_wait_for_next_measurement(160, &latest_weight) &&
            charge_mode_is_valid_scale_measurement(latest_weight)) {
            older_weight = previous_weight;
            previous_weight = *measurement_io;
            *measurement_io = latest_weight;
            charge_mode_update_true_final_measurement(latest_weight);
            scale_fail_count = 0;
        }
        else {
            scale_fail_count++;
            vTaskDelay(pdMS_TO_TICKS(20));
            if (scale_fail_count >= 6) {
                break;
            }
        }
    }

    charge_mode_command_motor(SELECT_FINE_TRICKLER_MOTOR, 0);
    if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
        servo_gate_set_ratio(SERVO_GATE_RATIO_CLOSED, true);
    }

    if (attempted) {
        TickType_t now = xTaskGetTickCount();
        ml_fine_time_ms += (float)((now - salvage_start_tick) * portTICK_PERIOD_MS);
        last_charge_elapsed_seconds = (float)((now - charge_start_tick) * portTICK_PERIOD_MS) / 1000.0f;

        float settled_weight = *measurement_io;
        if (charge_mode_capture_settled_measurement(1200,
                                                    FINAL_WEIGHT_SETTLE_SAMPLE_COUNT,
                                                    &settled_weight)) {
            charge_mode_update_true_final_measurement(settled_weight);
            *measurement_io = settled_weight;
        }
    }

    return attempted;
}


void scale_measurement_render_task(void *p) {
    char current_weight_string[WEIGHT_STRING_LEN];
    char time_buffer[16];

    u8g2_t *display_handler = get_display_handler();

    while (true) {
        TickType_t last_render_tick = xTaskGetTickCount();

        u8g2_ClearBuffer(display_handler);

        // Set font for title and timer
        u8g2_SetFont(display_handler, u8g2_font_helvB08_tr);

        // Format the timer string based on current state
        if (charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_COMPLETE) {
            format_elapsed_time(time_buffer, sizeof(time_buffer), charge_start_tick);
        } else if (charge_mode_config.charge_mode_state == CHARGE_MODE_STABILIZING ||
                   charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_CUP_REMOVAL ||
                   charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_CUP_RETURN ||
                   charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_ZERO) {
            snprintf(time_buffer, sizeof(time_buffer), "%.2f s", last_charge_elapsed_seconds);
        } else {
            snprintf(time_buffer, sizeof(time_buffer), "--.- s");
        }

        // Calculate x positions
        uint8_t screen_width = u8g2_GetDisplayWidth(display_handler);
        uint8_t time_width = u8g2_GetStrWidth(display_handler, time_buffer);

        // Draw title on left
        u8g2_DrawStr(display_handler, 5, 10, title_string);

        // Draw timer on right edge
        u8g2_DrawStr(display_handler, screen_width - time_width - 5, 10, time_buffer);  // 5 px padding from edge

        // Draw line under title
        u8g2_DrawHLine(display_handler, 0, 13, screen_width);

        // Current weight (only show values > -1.0)
        memset(current_weight_string, 0x0, sizeof(current_weight_string));
        float scale_measurement = scale_get_current_measurement();
        if (charge_mode_is_valid_scale_measurement(scale_measurement) && scale_measurement > -1.0f) {
            float_to_string(current_weight_string, scale_measurement, charge_mode_config.eeprom_charge_mode_data.decimal_places);
        } else {
            strcpy(current_weight_string, "---");
        }

        // Draw current weight value
        u8g2_SetFont(display_handler, u8g2_font_profont22_tf);
        u8g2_DrawStr(display_handler, 26, 35, current_weight_string);

        // Draw profile name
        profile_t *current_profile = profile_get_selected();
        u8g2_SetFont(display_handler, u8g2_font_helvR08_tr);
        u8g2_DrawStr(display_handler, 5, 61, current_profile->name);

        u8g2_SendBuffer(display_handler);

        vTaskDelayUntil(&last_render_tick, pdMS_TO_TICKS(20));
    }
}


void charge_mode_wait_for_zero() {
    charge_mode_set_live_phase("wait_zero", 0.0f, 0.0f);
    // Set colour to not ready
    neopixel_led_set_colour(
        neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        true
    );
    
    // Wait for 5 measurements and wait for stable
    FloatRingBuffer data_buffer(10);

    // Update current status
    snprintf(title_string, sizeof(title_string), "Waiting for Zero");

    // Stop condition: 10 stable measurements in 200ms apart (2 seconds minimum)
    while (true) {
        TickType_t last_measurement_tick = xTaskGetTickCount();

        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }
        else if (button_encoder_event == BUTTON_ENCODER_PRESSED) {
            scale_config.scale_handle->force_zero();
        }

        // Perform measurement (max delay 300 seconds   )
        float current_measurement;
        if (scale_block_wait_for_next_measurement(300, &current_measurement) &&
            charge_mode_is_valid_scale_measurement(current_measurement)){
            data_buffer.enqueue(current_measurement);
        }

        // Generate stop condition
        if (data_buffer.getCounter() >= 10){
            if (data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin &&
                fabsf(data_buffer.getMean()) < charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin) {
                break;
            }
        }

        // Wait for minimum 300 ms (but can skip if previously wait already)
        vTaskDelayUntil(&last_measurement_tick, pdMS_TO_TICKS(300));
    }

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_COMPLETE;
}

void charge_mode_wait_for_complete() {

    charge_start_tick = xTaskGetTickCount();
    last_final_weight_valid = false;
    last_final_weight_gn = 0.0f;
    ai_record_pending = false;
    ml_record_pending = false;
    charge_mode_reset_live_metrics();
    charge_mode_set_live_phase("dispense", charge_mode_config.target_charge_weight, 0.0f);
    charge_mode_config.charge_mode_event &= ~(CHARGE_MODE_EVENT_UNDER_CHARGE | CHARGE_MODE_EVENT_OVER_CHARGE);

    memset(&charge_mode_ai_session_snapshot, 0, sizeof(charge_mode_ai_session_snapshot));
    bool ai_session_available = ai_tuning_get_session_copy(&charge_mode_ai_session_snapshot);
    bool ai_active = ai_session_available &&
                     (charge_mode_ai_session_snapshot.state == AI_TUNING_CHARACTERIZING_COARSE ||
                      charge_mode_ai_session_snapshot.state == AI_TUNING_CHARACTERIZING_FINE ||
                      charge_mode_ai_session_snapshot.state == AI_TUNING_CALIBRATING_COARSE ||
                      charge_mode_ai_session_snapshot.state == AI_TUNING_CALIBRATING_FINE);
    last_charge_was_ai_tuning = ai_active;

    ai_tuning_plan_t ai_plan = {};
    bool ai_plan_available = ai_active &&
                             ai_tuning_get_active_plan(&ai_plan) &&
                             ai_plan.valid;

    profile_t *current_profile = profile_get_selected();
    uint8_t selected_profile_idx = (uint8_t)profile_get_selected_idx();
    memset(&charge_mode_runtime_model, 0, sizeof(charge_mode_runtime_model));
    ai_profile_model_t& runtime_model = charge_mode_runtime_model;
    bool runtime_model_enabled = !ai_active &&
                                 ai_tuning_get_enabled_model_copy(selected_profile_idx, &runtime_model) &&
                                 runtime_model.valid &&
                                 runtime_model.enabled;
    ai_tuning_config_t* ai_config = ai_tuning_get_config();
    live_runtime_model_active = runtime_model_enabled;
    last_charge_used_runtime_model = runtime_model_enabled;

    charge_mode_set_result_colour(charge_mode_make_colour(CHARGE_PROGRESS_ORANGE));

    if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
        servo_gate_set_ratio(SERVO_GATE_RATIO_OPEN, false);
    }

    char target_weight_string[WEIGHT_STRING_LEN];
    float_to_string(target_weight_string,
                    charge_mode_config.target_charge_weight,
                    charge_mode_config.eeprom_charge_mode_data.decimal_places);
    if (ai_plan_available) {
        snprintf(title_string, sizeof(title_string), "AI %u/%u %.2frps",
                 (unsigned int)ai_plan.sample_index,
                 (unsigned int)ai_plan.total_samples,
                 ai_plan.speed_rps);
    }
    else {
        snprintf(title_string, sizeof(title_string), "Target: %s", target_weight_string);
    }

    const float coarse_motor_max_speed = fminf((float)get_motor_max_speed(SELECT_COARSE_TRICKLER_MOTOR), 8.0f);
    const float fine_motor_max_speed = fminf((float)get_motor_max_speed(SELECT_FINE_TRICKLER_MOTOR), 6.0f);
    const float coarse_trickler_max_speed = fminf(coarse_motor_max_speed,
                                                  current_profile->coarse_max_flow_speed_rps);
    const float coarse_trickler_min_speed = fmaxf(get_motor_min_speed(SELECT_COARSE_TRICKLER_MOTOR),
                                                  current_profile->coarse_min_flow_speed_rps);
    const float fine_trickler_max_speed = fminf(fine_motor_max_speed,
                                                current_profile->fine_max_flow_speed_rps);
    const float fine_trickler_min_speed = fmaxf(get_motor_min_speed(SELECT_FINE_TRICKLER_MOTOR),
                                                current_profile->fine_min_flow_speed_rps);
    const float ai_coarse_trickler_max_speed = coarse_motor_max_speed;
    const float ai_fine_trickler_max_speed = fine_motor_max_speed;

    TickType_t coarse_stop_tick = 0;
    float coarse_stop_weight = 0.0f;
    bool coarse_stop_weight_valid = false;

    auto is_plausible_coarse_stop_weight = [&](float measured_weight) -> bool {
        const float target_weight = fmaxf(charge_mode_config.target_charge_weight, 1.0f);
        return charge_mode_is_valid_scale_measurement(measured_weight) &&
               measured_weight > fmaxf(0.20f, target_weight * 0.03f) &&
               measured_weight < target_weight + fmaxf(6.0f, target_weight * 0.20f);
    };

    auto stop_all_motors = [&]() {
        charge_mode_command_motor(SELECT_COARSE_TRICKLER_MOTOR, 0);
        charge_mode_command_motor(SELECT_FINE_TRICKLER_MOTOR, 0);
    };

    auto emergency_exit = [&]() {
        stop_all_motors();
        motor_enable(SELECT_COARSE_TRICKLER_MOTOR, false);
        motor_enable(SELECT_FINE_TRICKLER_MOTOR, false);
        if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
            servo_gate_set_ratio(SERVO_GATE_RATIO_CLOSED, false);
        }
        if (ai_active) {
            ai_tuning_cancel();
        }
        charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
    };

    auto mark_coarse_stop = [&](float measured_weight, bool apply_coarse_stop_gate_ratio) {
        charge_mode_command_motor(SELECT_COARSE_TRICKLER_MOTOR, 0);
        if (coarse_stop_tick == 0) {
            coarse_stop_tick = xTaskGetTickCount();
        }
        if (is_plausible_coarse_stop_weight(measured_weight)) {
            coarse_stop_weight = measured_weight;
            coarse_stop_weight_valid = true;
            live_coarse_stop_weight_gn = measured_weight;
        }

        if (apply_coarse_stop_gate_ratio &&
            servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
            float ratio = charge_mode_config.eeprom_charge_mode_data.coarse_stop_gate_ratio;
            servo_gate_set_ratio(ratio, false);
        }
    };

    auto wait_with_abort = [&](uint32_t duration_ms) -> bool {
        TickType_t start_tick = xTaskGetTickCount();
        while (true) {
            uint32_t elapsed_ms = (uint32_t)((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS);
            if (elapsed_ms >= duration_ms) {
                break;
            }

            if (button_wait_for_input(false) == BUTTON_RST_PRESSED) {
                emergency_exit();
                return false;
            }

            uint32_t remaining_ms = duration_ms - elapsed_ms;
            uint32_t sleep_ms = remaining_ms < 20 ? remaining_ms : 20;
            TickType_t sleep_ticks = pdMS_TO_TICKS(sleep_ms > 0 ? sleep_ms : 1);
            if (sleep_ticks == 0) {
                sleep_ticks = 1;
            }
            vTaskDelay(sleep_ticks);
        }
        return true;
    };

    auto run_motor_for_duration = [&](motor_select_t motor, float speed_rps, uint32_t duration_ms) -> bool {
        if (duration_ms == 0 || speed_rps <= 0.0f) {
            return true;
        }

        charge_mode_command_motor(motor, speed_rps);
        bool ok = wait_with_abort(duration_ms);
        charge_mode_command_motor(motor, 0);
        return ok;
    };

    auto get_latest_measurement = [&](uint32_t timeout_ms, float fallback_value) -> float {
        float measurement = fallback_value;
        if (scale_block_wait_for_next_measurement(timeout_ms, &measurement) &&
            charge_mode_is_valid_scale_measurement(measurement)) {
            return measurement;
        }
        float live_measurement = scale_get_current_measurement();
        if (charge_mode_is_valid_scale_measurement(live_measurement)) {
            return live_measurement;
        }
        if (!charge_mode_is_valid_scale_measurement(fallback_value)) {
            return 0.0f;
        }
        return fallback_value;
    };

    auto capture_coarse_stop_measurement = [&](uint32_t timeout_ms, float fallback_value) -> float {
        if (is_plausible_coarse_stop_weight(fallback_value)) {
            return fallback_value;
        }

        TickType_t start_tick = xTaskGetTickCount();
        float best_weight = NAN;
        while ((uint32_t)((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS) < timeout_ms) {
            float measurement = 0.0f;
            if (scale_block_wait_for_next_measurement(60, &measurement) &&
                charge_mode_is_valid_scale_measurement(measurement)) {
                if (is_plausible_coarse_stop_weight(measurement)) {
                    return measurement;
                }
                if (!charge_mode_is_cup_removed_measurement(measurement)) {
                    best_weight = measurement;
                }
            }
        }

        return is_plausible_coarse_stop_weight(best_weight) ? best_weight : NAN;
    };

    auto wait_for_settled_measurement = [&](uint32_t max_wait_ms, float fallback_value) -> float {
        FloatRingBuffer settle_buffer(5);
        TickType_t start_tick = xTaskGetTickCount();
        float latest = fallback_value;

        while ((uint32_t)((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS) < max_wait_ms) {
            if (button_wait_for_input(false) == BUTTON_RST_PRESSED) {
                emergency_exit();
                return latest;
            }

            float measurement = latest;
            if (scale_block_wait_for_next_measurement(120, &measurement) &&
                charge_mode_is_valid_scale_measurement(measurement)) {
                latest = measurement;
                settle_buffer.enqueue(measurement);
                if (settle_buffer.getCounter() >= 4 &&
                    settle_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin) {
                    break;
                }
            }
        }

        return latest;
    };

    auto update_coarse_tail_telemetry = [&](float stop_weight, float settled_weight) {
        if (charge_mode_is_valid_scale_measurement(settled_weight)) {
            live_after_coarse_settle_weight_gn = settled_weight;
        }
        if (!is_plausible_coarse_stop_weight(stop_weight) ||
            !charge_mode_is_valid_scale_measurement(settled_weight) ||
            settled_weight < stop_weight - 0.10f) {
            return;
        }

        float observed_tail = settled_weight - stop_weight;
        float max_plausible_tail = fmaxf(8.0f, charge_mode_config.target_charge_weight * 0.30f);
        if (observed_tail >= 0.0f && observed_tail <= max_plausible_tail) {
            live_observed_coarse_tail_gn = observed_tail;
        }
    };

    auto capture_ai_stop_response = [&](TickType_t stop_tick,
                                        float stop_weight,
                                        ai_drop_telemetry_t* telemetry) -> float {
        if (telemetry == nullptr) {
            return stop_weight;
        }

        FloatRingBuffer settle_buffer(5);
        float latest = charge_mode_is_valid_scale_measurement(stop_weight) ? stop_weight : 0.0f;
        float reference = latest;
        TickType_t previous_sample_tick = 0;
        uint32_t sample_count = 0;
        float sample_period_sum_ms = 0.0f;
        bool first_response_seen = false;
        bool settle_seen = false;
        const float response_threshold =
            fmaxf(charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin * 1.5f, 0.012f);
        TickType_t start_tick = xTaskGetTickCount();

        while ((uint32_t)((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS) < 1400u) {
            float measurement = latest;
            if (scale_block_wait_for_next_measurement(140, &measurement) &&
                charge_mode_is_valid_scale_measurement(measurement)) {
                TickType_t sample_tick = xTaskGetTickCount();
                latest = measurement;
                settle_buffer.enqueue(measurement);

                if (previous_sample_tick != 0) {
                    sample_period_sum_ms +=
                        (float)((sample_tick - previous_sample_tick) * portTICK_PERIOD_MS);
                    sample_count++;
                }
                previous_sample_tick = sample_tick;

                if (!first_response_seen &&
                    measurement > reference + response_threshold) {
                    telemetry->first_response_time_ms =
                        (float)((sample_tick - stop_tick) * portTICK_PERIOD_MS);
                    first_response_seen = true;
                }

                if (!settle_seen &&
                    settle_buffer.getCounter() >= 4 &&
                    settle_buffer.getSd() <
                        charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin) {
                    telemetry->settle_time_ms =
                        (float)((sample_tick - stop_tick) * portTICK_PERIOD_MS);
                    settle_seen = true;
                    break;
                }
            }
        }

        if (!first_response_seen) {
            telemetry->first_response_time_ms =
                (float)((xTaskGetTickCount() - stop_tick) * portTICK_PERIOD_MS);
        }
        if (!settle_seen) {
            telemetry->settle_time_ms =
                (float)((xTaskGetTickCount() - stop_tick) * portTICK_PERIOD_MS);
        }
        telemetry->scale_sample_period_ms = sample_count > 0
            ? sample_period_sum_ms / (float)sample_count
            : 0.0f;
        return latest;
    };

    auto mark_fine_stop = [&](float stop_weight, float settled_weight) {
        // Keep the first fine stop as fast-finish telemetry. Recovery has its own
        // start/end/tail fields and must not overwrite the phase being learned.
        if (!charge_mode_is_valid_scale_measurement(live_fine_stop_weight_gn) &&
            charge_mode_is_valid_scale_measurement(stop_weight)) {
            live_fine_stop_weight_gn = stop_weight;
        }
        if (!charge_mode_is_valid_scale_measurement(live_after_fine_settle_weight_gn) &&
            charge_mode_is_valid_scale_measurement(settled_weight)) {
            live_after_fine_settle_weight_gn = settled_weight;
        }
        if (!isfinite(live_observed_fine_tail_gn) &&
            charge_mode_is_valid_scale_measurement(stop_weight) &&
            charge_mode_is_valid_scale_measurement(settled_weight)) {
            live_observed_fine_tail_gn = fmaxf(0.0f, settled_weight - stop_weight);
        }
    };

    bool record_ai_drop = false;
    ai_drop_telemetry_t pending_ai_drop = {};

    if (ai_active) {
        if (!ai_plan_available) {
            emergency_exit();
            return;
        }

        ai_motor_mode_t sample_motor_mode = ai_plan.motor_mode;
        float plan_speed_rps = (sample_motor_mode == AI_MOTOR_MODE_FINE_ONLY)
            ? fmaxf(fine_trickler_min_speed, fminf(ai_plan.speed_rps, ai_fine_trickler_max_speed))
            : fmaxf(coarse_trickler_min_speed, fminf(ai_plan.speed_rps, ai_coarse_trickler_max_speed));
        uint32_t plan_on_time_ms = (uint32_t)lroundf(fmaxf(0.0f, ai_plan.motor_on_time_ms));

        float start_weight = get_latest_measurement(300, scale_get_current_measurement());
        TickType_t motor_start_tick = xTaskGetTickCount();
        charge_mode_set_live_phase(sample_motor_mode == AI_MOTOR_MODE_FINE_ONLY ? "ai_fine" : "ai_coarse",
                                   ai_plan.target_weight,
                                   0.0f);

        bool ok = true;
        if (sample_motor_mode == AI_MOTOR_MODE_FINE_ONLY) {
            ok = run_motor_for_duration(SELECT_FINE_TRICKLER_MOTOR, plan_speed_rps, plan_on_time_ms);
        }
        else {
            ok = run_motor_for_duration(SELECT_COARSE_TRICKLER_MOTOR, plan_speed_rps, plan_on_time_ms);
            if (ok) {
                float stop_weight = capture_coarse_stop_measurement(320, start_weight);
                mark_coarse_stop(stop_weight, true);
            }
        }

        if (!ok) {
            return;
        }

        TickType_t motor_end_tick = xTaskGetTickCount();
        float stop_weight = get_latest_measurement(250, start_weight);
        if (sample_motor_mode == AI_MOTOR_MODE_COARSE_ONLY) {
            stop_weight = capture_coarse_stop_measurement(320, stop_weight);
            if (!coarse_stop_weight_valid) {
                mark_coarse_stop(stop_weight, true);
            }
        }

        memset(&pending_ai_drop, 0, sizeof(pending_ai_drop));
        pending_ai_drop.sample_number = ai_plan.sample_index;
        pending_ai_drop.motor_mode = sample_motor_mode;
        pending_ai_drop.speed_rps = plan_speed_rps;
        pending_ai_drop.motor_on_time_ms = (float)((motor_end_tick - motor_start_tick) * portTICK_PERIOD_MS);
        pending_ai_drop.start_weight = start_weight;
        pending_ai_drop.stop_weight = stop_weight;
        pending_ai_drop.target_weight = ai_plan.target_weight;
        pending_ai_drop.coarse_time_ms = (sample_motor_mode == AI_MOTOR_MODE_COARSE_ONLY)
            ? pending_ai_drop.motor_on_time_ms
            : 0.0f;
        pending_ai_drop.fine_time_ms = (sample_motor_mode == AI_MOTOR_MODE_FINE_ONLY)
            ? pending_ai_drop.motor_on_time_ms
            : 0.0f;
        pending_ai_drop.total_time_ms = pending_ai_drop.motor_on_time_ms;
        float response_weight = capture_ai_stop_response(motor_end_tick,
                                                         stop_weight,
                                                         &pending_ai_drop);
        pending_ai_drop.final_weight = response_weight;
        pending_ai_drop.delivered_weight =
            fmaxf(0.0f, pending_ai_drop.final_weight - pending_ai_drop.start_weight);
        pending_ai_drop.tail_weight =
            charge_mode_is_valid_scale_measurement(pending_ai_drop.stop_weight)
                ? fmaxf(0.0f, pending_ai_drop.final_weight - pending_ai_drop.stop_weight)
                : 0.0f;
        pending_ai_drop.overthrow = pending_ai_drop.final_weight - pending_ai_drop.target_weight;
        record_ai_drop = true;
    }
    else if (runtime_model_enabled) {
        float bulk_speed = runtime_model.coarse_best_speed_rps;
        float bulk_flow = runtime_model.coarse_best_flow_gps;
        float bulk_tail = runtime_model.coarse_tail_gn;
        bool bulk_choice_safe = ai_model_choose_flow_sample(runtime_model.coarse_samples,
                                                            runtime_model.coarse_sample_count,
                                                            false,
                                                            ai_config->noise_margin,
                                                            &bulk_speed,
                                                            &bulk_flow,
                                                            &bulk_tail);

            float trim_speed = runtime_model.coarse_trim_speed_rps;
            float trim_flow = runtime_model.coarse_trim_flow_gps;
            float trim_tail = runtime_model.coarse_trim_tail_gn;
        if (trim_speed <= 0.0f || trim_flow <= 0.0f) {
            (void)ai_model_choose_flow_sample(runtime_model.coarse_samples,
                                              runtime_model.coarse_sample_count,
                                              true,
                                              ai_config->noise_margin,
                                              &trim_speed,
                                              &trim_flow,
                                              &trim_tail);
        }

        float fine_speed = runtime_model.fine_best_speed_rps;
        float fine_flow = runtime_model.fine_best_flow_gps;
        float fine_tail = runtime_model.fine_tail_gn;
        (void)ai_model_choose_flow_sample(runtime_model.fine_samples,
                                          runtime_model.fine_sample_count,
                                          false,
                                          ai_config->noise_margin,
                                          &fine_speed,
                                          &fine_flow,
                                          &fine_tail);

        bulk_speed = fmaxf(coarse_trickler_min_speed, fminf(bulk_speed, ai_coarse_trickler_max_speed));
        trim_speed = fmaxf(coarse_trickler_min_speed, fminf(trim_speed, ai_coarse_trickler_max_speed));
        fine_speed = fmaxf(fine_trickler_min_speed, fminf(fine_speed, ai_fine_trickler_max_speed));
        if (bulk_flow <= 0.0f) {
            bulk_flow = ai_model_predict_flow_gps(&runtime_model, AI_MOTOR_MODE_COARSE_ONLY, bulk_speed);
        }
        if (trim_flow <= 0.0f) {
            trim_flow = ai_model_predict_flow_gps(&runtime_model, AI_MOTOR_MODE_COARSE_ONLY, trim_speed);
        }
        if (fine_flow <= 0.0f) {
            fine_flow = ai_model_predict_flow_gps(&runtime_model, AI_MOTOR_MODE_FINE_ONLY, fine_speed);
        }

        const float characterized_bulk_flow = bulk_flow;
        const float characterized_trim_flow = trim_flow;
        const float characterized_fine_flow = fine_flow;
        const bool bulk_matches_saved_best =
            fabsf(bulk_speed - runtime_model.coarse_best_speed_rps) <= 0.05f;
        const bool bulk_matches_trim =
            trim_speed > 0.0f && fabsf(bulk_speed - trim_speed) <= 0.05f;
        const bool machine_calibrated = runtime_model.machine.valid;
        if (machine_calibrated) {
            if (runtime_model.machine.coarse_open_loop_flow_gps > 0.05f &&
                bulk_matches_saved_best &&
                !bulk_matches_trim) {
                bulk_flow = runtime_model.machine.coarse_open_loop_flow_gps;
            }
            else if (runtime_model.machine.trim_open_loop_flow_gps > 0.05f &&
                     bulk_matches_trim) {
                bulk_flow = runtime_model.machine.trim_open_loop_flow_gps;
            }
            if (runtime_model.machine.trim_open_loop_flow_gps > 0.05f) {
                trim_flow = runtime_model.machine.trim_open_loop_flow_gps;
            }
            if (runtime_model.machine.fine_open_loop_flow_gps > 0.005f) {
                fine_flow = runtime_model.machine.fine_open_loop_flow_gps;
            }
        }

        // For stop timing, underestimating flow is dangerous. Short machine-cal pulses
        // can under-read fast coarse flow, so never let them slow the safety model down.
        if (characterized_bulk_flow > 0.05f) {
            bulk_flow = fmaxf(bulk_flow, characterized_bulk_flow * 0.92f);
        }
        if (characterized_trim_flow > 0.05f) {
            trim_flow = fmaxf(trim_flow, characterized_trim_flow * 0.85f);
        }
        if (characterized_fine_flow > 0.005f) {
            fine_flow = fmaxf(fine_flow, characterized_fine_flow * 0.85f);
        }

        if (fine_speed <= 0.0f || fine_flow <= ai_config->noise_margin * 0.25f ||
            bulk_speed <= 0.0f || bulk_flow <= 0.0f) {
            runtime_model_enabled = false;
        }
        else {
            const float target_weight = charge_mode_config.target_charge_weight;
            const float positive_bias =
                fminf(fmaxf(runtime_model.runtime_bias_gn, 0.0f),
                      fmaxf(0.75f, target_weight * 0.03f));
            const float target_tolerance = charge_mode_acceptance_tolerance();
            float machine_coarse_tail_guard = 0.0f;
            if (machine_calibrated) {
                const float machine_avg_tail =
                    isfinite(runtime_model.machine.coarse_tail_avg_gn)
                        ? fmaxf(runtime_model.machine.coarse_tail_avg_gn, 0.0f)
                        : 0.0f;
                const float machine_p95_tail =
                    isfinite(runtime_model.machine.coarse_tail_p95_gn)
                        ? fmaxf(runtime_model.machine.coarse_tail_p95_gn, machine_avg_tail)
                        : machine_avg_tail;
                const float machine_uncertainty =
                    isfinite(runtime_model.machine.coarse_uncertainty_gn)
                        ? fmaxf(runtime_model.machine.coarse_uncertainty_gn, 0.0f)
                        : 0.0f;
                machine_coarse_tail_guard =
                    fmaxf(machine_p95_tail, machine_avg_tail + machine_uncertainty * 0.65f);
                machine_coarse_tail_guard =
                    fminf(machine_coarse_tail_guard, fmaxf(20.0f, target_weight * 0.55f));
            }
            bool derived_trim_coarse = false;
            const bool native_trim_coarse =
                trim_speed > 0.0f &&
                trim_flow > 0.0f &&
                trim_speed < bulk_speed - 0.03f &&
                trim_flow < bulk_flow * 0.95f;
            const float production_safe_coarse_tail =
                fmaxf(2.50f, fminf(4.00f, target_weight * 0.10f));
            bool rejected_coarse_choice = false;
            float rejected_coarse_speed = bulk_speed;
            float rejected_coarse_tail = bulk_tail;
            if (native_trim_coarse &&
                (bulk_tail > fmaxf(6.0f, target_weight * 0.14f) ||
                 (bulk_flow > trim_flow * 2.50f && trim_tail <= fmaxf(5.0f, target_weight * 0.12f)))) {
                rejected_coarse_choice = true;
                rejected_coarse_speed = bulk_speed;
                rejected_coarse_tail = bulk_tail;
                bulk_speed = trim_speed;
                bulk_flow = trim_flow;
                bulk_tail = trim_tail;
                bulk_choice_safe = false;
            }
            if (bulk_tail > production_safe_coarse_tail &&
                native_trim_coarse &&
                trim_tail <= bulk_tail) {
                rejected_coarse_choice = true;
                rejected_coarse_speed = bulk_speed;
                rejected_coarse_tail = bulk_tail;
                bulk_speed = trim_speed;
                bulk_flow = trim_flow;
                bulk_tail = trim_tail;
                bulk_choice_safe = false;
            }

            if (!native_trim_coarse &&
                bulk_speed > coarse_trickler_min_speed + 0.04f &&
                bulk_flow > 0.05f) {
                float glide_speed = fmaxf(coarse_trickler_min_speed,
                                          fminf(fmaxf(0.18f, bulk_speed * 0.34f),
                                                fminf(0.42f, bulk_speed * 0.55f)));
                if (glide_speed < bulk_speed - 0.03f) {
                    float scaled_glide_flow =
                        bulk_flow * (glide_speed / fmaxf(bulk_speed, 0.001f)) * 0.78f;
                    float predicted_glide_flow =
                        ai_model_predict_flow_gps(&runtime_model,
                                                  AI_MOTOR_MODE_COARSE_ONLY,
                                                  glide_speed);
                    if (predicted_glide_flow > 0.05f) {
                        scaled_glide_flow = fminf(predicted_glide_flow,
                                                  scaled_glide_flow * 1.25f);
                    }
                    trim_speed = glide_speed;
                    trim_flow = fmaxf(0.05f, scaled_glide_flow);
                    float learned_tail_floor = machine_calibrated
                        ? runtime_model.machine.coarse_uncertainty_gn * 0.45f +
                              runtime_model.machine.coarse_tail_avg_gn * 0.20f
                        : bulk_tail * 0.20f;
                    trim_tail = fmaxf(ai_config->noise_margin * 2.0f,
                                      fmaxf(learned_tail_floor,
                                            fminf(bulk_tail * 0.34f,
                                                  fmaxf(0.80f, target_weight * 0.030f))));
                    trim_tail = fminf(trim_tail, fmaxf(1.60f, target_weight * 0.045f));
                    derived_trim_coarse = true;
                }
            }

            const float planning_bulk_tail = fmaxf(bulk_tail, machine_coarse_tail_guard);
            if (rejected_coarse_choice) {
                snprintf(live_ai_decision,
                         sizeof(live_ai_decision),
                         "coarse %.2frps tail %.2f unsafe, using %.2frps curve",
                         rejected_coarse_speed,
                         rejected_coarse_tail,
                         bulk_speed);
            }
            else {
                snprintf(live_ai_decision,
                         sizeof(live_ai_decision),
                         "%s coarse %.2frps %s %.2f, %s %.2frps, fine %.2frps",
                         bulk_choice_safe ? "safe" : "least-risk",
                         bulk_speed,
                         machine_coarse_tail_guard > bulk_tail + 0.10f ? "guard" : "tail",
                         planning_bulk_tail,
                         derived_trim_coarse ? "glide" : "trim",
                         trim_speed,
                         fine_speed);
            }

            const float fine_tail_cap = fmaxf(0.75f, fminf(2.00f, target_weight * 0.05f));
            const float fast_tail_confidence =
                fmaxf(0.0f, fminf(1.0f, runtime_model.fine_fast_tail_confidence));
            const float micro_tail_confidence =
                fmaxf(0.0f, fminf(1.0f, runtime_model.fine_micro_tail_confidence));
            const bool low_tail_tube =
                runtime_model.fine_tube_profile == AI_FINE_TUBE_PROFILE_LOW_FLOW_LOW_TAIL;
            const bool high_tail_tube =
                runtime_model.fine_tube_profile == AI_FINE_TUBE_PROFILE_HIGH_TAIL;
            float learned_fast_tail = isfinite(runtime_model.fine_fast_tail_gn)
                ? runtime_model.fine_fast_tail_gn
                : fine_tail;
            learned_fast_tail = fmaxf(0.0f, fminf(learned_fast_tail, fine_tail_cap));
            if (fast_tail_confidence >= 0.35f) {
                fine_tail = learned_fast_tail;
            }
            else {
                fine_tail = fmaxf(fine_tail, learned_fast_tail);
            }

            float learned_micro_tail = isfinite(runtime_model.fine_micro_tail_gn)
                ? runtime_model.fine_micro_tail_gn
                : runtime_model.fine_recovery_tail_gn;
            learned_micro_tail = fmaxf(0.0f, fminf(learned_micro_tail, fminf(0.25f, fine_tail_cap)));
            float finish_safety_bias = isfinite(runtime_model.fine_stop_safety_bias_gn)
                ? runtime_model.fine_stop_safety_bias_gn
                : (0.055f * (1.0f - fast_tail_confidence) + 0.010f);
            finish_safety_bias = fmaxf(0.004f, fminf(finish_safety_bias, 0.110f));
            const float steering_bulk_bias = isfinite(runtime_model.steering_bulk_bias_gn)
                ? runtime_model.steering_bulk_bias_gn
                : 0.0f;
            const float steering_fine_bias = isfinite(runtime_model.steering_fine_bias_gn)
                ? runtime_model.steering_fine_bias_gn
                : 0.0f;
            const float steering_recovery_speed_scale =
                fmaxf(-0.35f,
                      fminf(0.60f,
                            isfinite(runtime_model.steering_recovery_speed_scale)
                                ? runtime_model.steering_recovery_speed_scale
                                : 0.0f));

            live_model_bulk_speed_rps = bulk_speed;
            live_model_bulk_flow_gps = bulk_flow;
            live_model_bulk_tail_gn = planning_bulk_tail;
            live_model_fine_speed_rps = fine_speed;
            live_model_fine_flow_gps = fine_flow;
            live_model_fine_tail_gn = fine_tail;
            const float final_target = target_weight;
            const float recovery_target = target_weight;
            trim_tail = fmaxf(trim_tail, ai_config->noise_margin * 1.5f);
            float coarse_tail_guard = ai_model_estimate_coarse_tail_guard(&runtime_model,
                                                                          planning_bulk_tail,
                                                                          target_weight);
            coarse_tail_guard = fmaxf(coarse_tail_guard, machine_coarse_tail_guard);

            float desired_fine_window = fmaxf(0.85f, fminf(1.10f, target_weight * 0.025f));
            float fine_window = fmaxf(runtime_model.recommended_fine_window_gn * 0.65f,
                                      fine_tail * 1.25f + ai_config->noise_margin * 1.6f);
            fine_window = fmaxf(fine_window,
                                coarse_tail_guard * 0.04f +
                                    trim_tail * 0.05f +
                                    ai_config->noise_margin * 1.2f);
            fine_window = fmaxf(fine_window, desired_fine_window);
            fine_window = fminf(fine_window, fmaxf(desired_fine_window * 1.45f,
                                                   target_weight * 0.035f));

            float fine_prime_speed = fmaxf(fine_trickler_min_speed,
                                            fminf(fine_speed * 0.70f,
                                                  ai_fine_trickler_max_speed * 0.85f));
            float fine_prime_flow = ai_model_predict_flow_gps(&runtime_model,
                                                              AI_MOTOR_MODE_FINE_ONLY,
                                                              fine_prime_speed);
            if (fine_prime_flow <= 0.005f && fine_speed > 0.0f) {
                fine_prime_flow = fine_flow * (fine_prime_speed / fine_speed);
            }
            bool fine_assist_available = fine_prime_speed > 0.0f &&
                                         fine_prime_flow > ai_config->noise_margin * 0.20f;

            float trim_tail_guard = fmaxf(trim_tail, ai_config->noise_margin * 2.0f);
            if (derived_trim_coarse) {
                trim_tail_guard = fmaxf(trim_tail_guard,
                                        fminf(coarse_tail_guard * 0.28f,
                                              fmaxf(0.95f, target_weight * 0.035f)));
            }
            else {
                trim_tail_guard = fmaxf(trim_tail_guard,
                                        fminf(coarse_tail_guard * 0.45f,
                                              fmaxf(1.50f, target_weight * 0.055f)));
            }
            if (machine_coarse_tail_guard > 0.0f) {
                // Coarse trim is still the coarse motor. If calibration or runtime
                // throws show a large coarse flight tail, do not let an old low cap
                // pretend this phase can safely stop within a fine-sized window.
                float learned_trim_floor = fmaxf(trim_tail,
                                                 machine_coarse_tail_guard *
                                                     (derived_trim_coarse ? 0.72f : 0.85f));
                trim_tail_guard = fmaxf(trim_tail_guard, learned_trim_floor);
            }
            const float trim_tail_cap = machine_coarse_tail_guard > 0.0f
                ? fmaxf(10.0f, target_weight * 0.32f)
                : fmaxf(2.20f, target_weight * 0.070f);
            trim_tail_guard = fminf(trim_tail_guard, trim_tail_cap);

            float transition_tail = fmaxf(trim_tail_guard * 0.72f,
                                          fine_tail * 0.35f + target_tolerance);
            transition_tail = fminf(transition_tail, fmaxf(1.80f, target_weight * 0.085f));
            float effective_bulk_tail = fmaxf(transition_tail + trim_tail_guard * 0.45f,
                                              coarse_tail_guard * 0.70f);
            effective_bulk_tail = fminf(effective_bulk_tail, fmaxf(3.20f, target_weight * 0.22f));
            if (machine_coarse_tail_guard > 0.0f) {
                effective_bulk_tail = fmaxf(effective_bulk_tail, machine_coarse_tail_guard);
            }
            float coarse_handoff_bias = fminf(positive_bias * 0.45f,
                                              fmaxf(0.35f, target_weight * 0.025f));
            float learned_overcharge_guard = fminf(0.70f, positive_bias * 0.78f);
            // The first coarse burst is open-loop because the scale response lags powder flight.
            // Keep fine assist out of this burst so the model has one fast flow to bound.
            float bulk_phase_flow = bulk_flow;
            float trim_phase_flow = trim_flow + (fine_assist_available ? fine_prime_flow : 0.0f);
            float bulk_lag_margin = fminf(bulk_phase_flow * 0.08f,
                                          fmaxf(0.25f, effective_bulk_tail * 0.28f));
            float trim_lag_margin = fminf(trim_phase_flow * 0.10f,
                                          fmaxf(0.10f, trim_tail_guard * 0.18f));
            float trim_stop_margin = fine_window +
                                     transition_tail +
                                     trim_lag_margin +
                                     coarse_handoff_bias * 0.45f +
                                     learned_overcharge_guard;
            float bulk_handoff_margin = fine_window +
                                        effective_bulk_tail +
                                        transition_tail * 0.25f +
                                        bulk_lag_margin +
                                        coarse_handoff_bias +
                                        learned_overcharge_guard * 0.35f;
            if (machine_calibrated) {
                if (machine_coarse_tail_guard > 0.0f) {
                    const float machine_uncertainty =
                        isfinite(runtime_model.machine.coarse_uncertainty_gn)
                            ? fmaxf(runtime_model.machine.coarse_uncertainty_gn, 0.0f)
                            : 0.0f;
                    const float machine_latency_s =
                        (fmaxf(runtime_model.machine.scale_sample_period_ms, 0.0f) +
                         fmaxf(runtime_model.machine.coarse_first_response_ms, 0.0f)) /
                        1000.0f;
                    const float machine_latency_margin =
                        fminf(fmaxf(0.85f, target_weight * 0.040f),
                              bulk_phase_flow * machine_latency_s * 0.85f);
                    const float machine_uncertainty_margin =
                        fminf(fmaxf(2.25f, target_weight * 0.070f),
                              machine_uncertainty * 0.70f);
                    const float machine_min_bulk_margin =
                        fine_window +
                        machine_coarse_tail_guard +
                        machine_uncertainty_margin +
                        machine_latency_margin;
                    bulk_handoff_margin =
                        fmaxf(bulk_handoff_margin, machine_min_bulk_margin);
                }
                if (runtime_model.machine.recommended_bulk_handoff_gn > 0.0f) {
                    bulk_handoff_margin =
                        fmaxf(bulk_handoff_margin,
                              runtime_model.machine.recommended_bulk_handoff_gn);
                }
                if (runtime_model.machine.recommended_trim_stop_gn > 0.0f) {
                    trim_stop_margin =
                        fmaxf(trim_stop_margin,
                              runtime_model.machine.recommended_trim_stop_gn);
                }
            }
            bulk_handoff_margin += steering_bulk_bias;
            trim_stop_margin += steering_bulk_bias * 0.45f;
            const float max_bulk_handoff_margin =
                desired_fine_window + fmaxf(18.0f, target_weight * 0.55f) +
                positive_bias * 0.60f;
            const float max_trim_stop_margin =
                desired_fine_window + fmaxf(4.75f, target_weight * 0.14f) +
                positive_bias * 0.45f;
            trim_stop_margin = fminf(trim_stop_margin, max_trim_stop_margin);
            trim_stop_margin = fmaxf(trim_stop_margin,
                                     fine_window + target_tolerance * 3.0f);
            bulk_handoff_margin = fminf(bulk_handoff_margin, max_bulk_handoff_margin);
            bulk_handoff_margin = fmaxf(bulk_handoff_margin,
                                        trim_stop_margin + 0.75f);
            bool have_trim_coarse = trim_speed > 0.0f &&
                                    trim_flow > 0.0f &&
                                    trim_speed < bulk_speed - 0.01f &&
                                    trim_flow < bulk_flow * 0.95f &&
                                    trim_tail_guard <= fmaxf(4.00f, target_weight * 0.10f);
            if (!have_trim_coarse &&
                trim_speed > 0.0f &&
                trim_flow > 0.0f &&
                trim_tail_guard > fmaxf(4.00f, target_weight * 0.10f)) {
                snprintf(live_ai_decision,
                         sizeof(live_ai_decision),
                         "coarse %.2frps guard %.2f, trim tail %.2f unsafe -> fine",
                         bulk_speed,
                         planning_bulk_tail,
                         trim_tail_guard);
            }
            const bool controllable_bulk =
                (bulk_tail <= production_safe_coarse_tail + (derived_trim_coarse ? 0.85f : 0.0f)) &&
                (!have_trim_coarse || bulk_speed <= trim_speed * 4.25f);
            bool bulk_running = target_weight > bulk_handoff_margin;
            bool trim_running = false;
            TickType_t bulk_deadline_tick = 0;
            live_final_target_gn = final_target;
            live_bulk_handoff_margin_gn = bulk_handoff_margin;
            live_trim_stop_margin_gn = trim_stop_margin;
            live_fine_window_gn = fine_window;
            bool final_recovery_active = false;
            bool micro_heal_active = false;
            TickType_t final_recovery_start_tick = 0;
            TickType_t recovery_motor_start_tick = 0;
            TickType_t recovery_last_progress_tick = 0;
            bool recovery_motor_running = false;
            const float micro_heal_entry_gn = fmaxf(0.18f, target_tolerance * 7.0f);
            const float recovery_force_feed_gn = fmaxf(0.030f, target_tolerance * 1.55f);

            auto stop_recovery_motor_timer = [&]() {
                if (recovery_motor_running) {
                    live_recovery_motor_on_ms +=
                        (uint32_t)((xTaskGetTickCount() - recovery_motor_start_tick) *
                                   portTICK_PERIOD_MS);
                    recovery_motor_running = false;
                }
            };

            auto command_recovery_fine_motor = [&](float speed_rps) {
                if (final_recovery_active && speed_rps > 0.0f) {
                    if (!recovery_motor_running) {
                        recovery_motor_start_tick = xTaskGetTickCount();
                        recovery_motor_running = true;
                    }
                }
                else {
                    stop_recovery_motor_timer();
                }
                charge_mode_command_motor(SELECT_FINE_TRICKLER_MOTOR, speed_rps);
            };

            auto command_curved_coarse_motor = [&](float remaining_weight,
                                                   float stop_margin,
                                                   bool bulk_phase) {
                float high_speed = bulk_phase ? bulk_speed : trim_speed;
                float low_speed = coarse_trickler_min_speed;
                if (bulk_phase && have_trim_coarse) {
                    low_speed = trim_speed;
                }
                else if (bulk_phase) {
                    low_speed = fmaxf(coarse_trickler_min_speed, bulk_speed * 0.45f);
                }
                else {
                    low_speed = fmaxf(coarse_trickler_min_speed, trim_speed * 0.45f);
                }

                high_speed = fmaxf(low_speed, high_speed);
                float curve_span = bulk_phase
                    ? fmaxf(5.0f, target_weight * 0.18f)
                    : fmaxf(1.5f, target_weight * 0.055f);
                float curve_position =
                    fmaxf(0.0f,
                          fminf(1.0f,
                                (remaining_weight - stop_margin) / curve_span));
                float curved_speed =
                    low_speed + (high_speed - low_speed) * powf(curve_position, 1.60f);

                charge_mode_command_motor(SELECT_COARSE_TRICKLER_MOTOR, curved_speed);
            };

            auto should_enter_trim_glide = [&](float remaining_weight) -> bool {
                if (!have_trim_coarse) {
                    return false;
                }

                // If bulk stopped safely above the coarse glide stop margin, use glide.
                // The previous gate also required roughly four seconds of fine-only work,
                // which skipped glide exactly in the 6-8gn gap where glide is most useful.
                float trim_entry_margin = trim_stop_margin +
                                          fmaxf(target_tolerance * 2.0f, 0.06f);
                return remaining_weight > trim_entry_margin;
            };

            auto start_trim_glide = [&](float remaining_weight) {
                trim_running = true;
                command_curved_coarse_motor(remaining_weight, trim_stop_margin, false);
                if (fine_assist_available &&
                    remaining_weight > fine_window + target_tolerance * 2.0f) {
                    charge_mode_command_motor(SELECT_FINE_TRICKLER_MOTOR, fine_prime_speed);
                }
                charge_mode_set_live_phase("trim", remaining_weight, trim_stop_margin);
            };

            auto run_guarded_coarse_topup = [&](float settled_weight) -> float {
                if (have_trim_coarse ||
                    machine_coarse_tail_guard <= 0.0f ||
                    !charge_mode_is_valid_scale_measurement(settled_weight) ||
                    charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
                    return settled_weight;
                }

                // Continuous coarse trim is unsafe on high-tail powders, but a few
                // bounded top-up bursts can move work away from the fine tube while
                // preserving the settle-after-each-burst safety behavior.
                const float reserve_gn =
                    fmaxf(fine_window + 4.60f,
                          fminf(machine_coarse_tail_guard * 0.62f + fine_window,
                                fmaxf(7.20f, target_weight * 0.24f)));
                const float topup_goal_weight = final_target - reserve_gn;
                if (settled_weight >= topup_goal_weight) {
                    return settled_weight;
                }

                float pulse_speed = trim_speed > 0.0f
                    ? trim_speed * 0.32f
                    : bulk_speed * 0.28f;
                pulse_speed = fmaxf(coarse_trickler_min_speed,
                                    fminf(pulse_speed,
                                          fminf(bulk_speed * 0.36f, 2.35f)));

                float estimated_flow =
                    bulk_phase_flow * (pulse_speed / fmaxf(bulk_speed, 0.001f)) * 0.60f;
                if (machine_calibrated &&
                    runtime_model.machine.trim_open_loop_flow_gps > 0.05f &&
                    trim_speed > 0.0f) {
                    float trim_scaled_flow =
                        runtime_model.machine.trim_open_loop_flow_gps *
                        (pulse_speed / fmaxf(trim_speed, 0.001f)) * 0.58f;
                    estimated_flow = fmaxf(estimated_flow, trim_scaled_flow);
                }
                estimated_flow = fmaxf(0.35f, fminf(estimated_flow, 4.50f));

                for (uint8_t topup_idx = 0; topup_idx < 2; topup_idx++) {
                    float gap_to_goal = topup_goal_weight - settled_weight;
                    if (gap_to_goal <= fmaxf(0.85f, target_tolerance * 6.0f)) {
                        break;
                    }

                    float desired_drop =
                        fminf(gap_to_goal * 0.42f,
                              fmaxf(0.85f, target_weight * 0.045f));
                    uint32_t pulse_ms = (uint32_t)lroundf(
                        (desired_drop / fmaxf(estimated_flow, 0.05f)) * 1000.0f);
                    uint32_t pulse_cap_ms = topup_idx == 0 ? 760u : 560u;
                    pulse_ms = (uint32_t)fmaxf(140.0f,
                                               fminf((float)pulse_ms,
                                                     (float)pulse_cap_ms));

                    charge_mode_set_live_phase("coarse_topup",
                                               final_target - settled_weight,
                                               reserve_gn);
                    if (!run_motor_for_duration(SELECT_COARSE_TRICKLER_MOTOR,
                                                pulse_speed,
                                                pulse_ms)) {
                        return settled_weight;
                    }

                    float stop_weight = capture_coarse_stop_measurement(180, NAN);
                    mark_coarse_stop(stop_weight, true);
                    float new_settled = wait_for_settled_measurement(
                        720,
                        charge_mode_is_valid_scale_measurement(stop_weight)
                            ? stop_weight
                            : settled_weight);
                    update_coarse_tail_telemetry(stop_weight, new_settled);
                    if (charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
                        return new_settled;
                    }
                    if (!charge_mode_is_valid_scale_measurement(new_settled)) {
                        break;
                    }

                    float delivered = new_settled - settled_weight;
                    settled_weight = new_settled;
                    if (delivered > 0.05f) {
                        float observed_flow = delivered / ((float)pulse_ms / 1000.0f);
                        estimated_flow = estimated_flow * 0.35f +
                                         fminf(observed_flow, 8.0f) * 0.65f;
                        if (delivered > desired_drop * 2.20f) {
                            break;
                        }
                    }
                    else {
                        estimated_flow = fmaxf(0.35f, estimated_flow * 0.72f);
                    }
                }

                return settled_weight;
            };

            if (bulk_running) {
                float deadline_flow = bulk_phase_flow * (machine_calibrated ? 1.15f : 1.05f);
                uint32_t hard_stop_ms = charge_mode_compute_open_loop_stop_ms(target_weight,
                                                                              bulk_handoff_margin,
                                                                              deadline_flow,
                                                                              machine_calibrated ? 0.28f : 0.35f);
                if (hard_stop_ms > 0) {
                    uint32_t max_bulk_ms = machine_calibrated
                        ? (uint32_t)lroundf(fmaxf(controllable_bulk ? 650.0f : 140.0f,
                                                  fminf(controllable_bulk ? 3200.0f : 420.0f,
                                                        (target_weight *
                                                         (controllable_bulk ? 0.72f : 0.42f) /
                                                         fmaxf(deadline_flow, 0.05f)) *
                                                            1000.0f)))
                        : (uint32_t)lroundf(fmaxf(controllable_bulk ? 800.0f : 180.0f,
                                                  fminf(controllable_bulk ? 3600.0f : 700.0f,
                                                        (target_weight *
                                                         (controllable_bulk ? 0.78f : 0.55f) /
                                                         fmaxf(deadline_flow, 0.05f)) *
                                                            1000.0f)));
                    hard_stop_ms = (uint32_t)fminf((float)hard_stop_ms, (float)max_bulk_ms);
                }
                if (hard_stop_ms > 0) {
                    command_curved_coarse_motor(target_weight, bulk_handoff_margin, true);
                    bulk_deadline_tick = xTaskGetTickCount() + pdMS_TO_TICKS(hard_stop_ms);
                    live_bulk_deadline_ms = hard_stop_ms;
                    charge_mode_set_live_phase("bulk", target_weight, bulk_handoff_margin);
                }
                else {
                    bulk_running = false;
                }
            }
            if (!bulk_running &&
                should_enter_trim_glide(final_target)) {
                start_trim_glide(final_target);
            }

            int scale_fail_count = 0;
            float previous_weight = NAN;
            float older_weight = NAN;
            TickType_t previous_sample_tick = 0;
            float previous_sample_period_s = 0.0f;
            while (true) {
                if (button_wait_for_input(false) == BUTTON_RST_PRESSED) {
                    emergency_exit();
                    return;
                }

                if (bulk_running && bulk_deadline_tick != 0 &&
                    (int32_t)(xTaskGetTickCount() - bulk_deadline_tick) >= 0) {
                    float stop_weight = capture_coarse_stop_measurement(220, previous_weight);
                    bulk_running = false;
                    mark_coarse_stop(stop_weight, true);
                    charge_mode_command_motor(SELECT_FINE_TRICKLER_MOTOR, 0);
                    float refreshed_stop_weight = capture_coarse_stop_measurement(180, stop_weight);
                    if (is_plausible_coarse_stop_weight(refreshed_stop_weight)) {
                        stop_weight = refreshed_stop_weight;
                        mark_coarse_stop(stop_weight, false);
                    }
                    float settled_weight = wait_for_settled_measurement(520, stop_weight);
                    update_coarse_tail_telemetry(stop_weight, settled_weight);
                    if (charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
                        return;
                    }
                    if (!have_trim_coarse) {
                        settled_weight = run_guarded_coarse_topup(settled_weight);
                    }
                    float remaining_after_bulk = final_target - settled_weight;
                    previous_weight = settled_weight;
                    older_weight = settled_weight;
                    previous_sample_tick = 0;
                    previous_sample_period_s = 0.0f;
                    if (should_enter_trim_glide(remaining_after_bulk)) {
                        start_trim_glide(remaining_after_bulk);
                    }
                    continue;
                }

                float current_weight = 0.0f;
                uint32_t scale_wait_ms = (bulk_running || trim_running) ? 80 : 200;
                if (bulk_running && bulk_deadline_tick != 0) {
                    int32_t remaining_ticks = (int32_t)(bulk_deadline_tick - xTaskGetTickCount());
                    if (remaining_ticks > 0) {
                        uint32_t remaining_ms = (uint32_t)remaining_ticks * portTICK_PERIOD_MS;
                        scale_wait_ms = fmaxf(10.0f, fminf((float)scale_wait_ms, (float)remaining_ms));
                    }
                }
                if (!scale_block_wait_for_next_measurement(scale_wait_ms, &current_weight)) {
                    scale_fail_count++;
                    int scale_fail_limit = (bulk_running || trim_running) ? 35 : 10;
                    if (scale_fail_count >= scale_fail_limit) {
                        emergency_exit();
                        return;
                    }
                    continue;
                }

                if (!charge_mode_is_valid_scale_measurement(current_weight)) {
                    scale_fail_count++;
                    int scale_fail_limit = (bulk_running || trim_running) ? 35 : 10;
                    if (scale_fail_count >= scale_fail_limit) {
                        emergency_exit();
                        return;
                    }
                    continue;
                }
                scale_fail_count = 0;

                TickType_t current_sample_tick = xTaskGetTickCount();
                float latest_sample_period_s = 0.0f;
                if (previous_sample_tick != 0 && current_sample_tick > previous_sample_tick) {
                    latest_sample_period_s =
                        (float)((current_sample_tick - previous_sample_tick) * portTICK_PERIOD_MS) / 1000.0f;
                }

                float active_fine_target = final_recovery_active ? recovery_target : final_target;
                float remaining_weight = active_fine_target - current_weight;
                charge_mode_set_progress_colour(target_weight - current_weight, target_weight);
                float momentum_margin = charge_mode_measure_positive_momentum(current_weight,
                                                                              previous_weight,
                                                                              older_weight);
                float observed_phase_flow = charge_mode_estimate_positive_flow_gps(current_weight,
                                                                                   previous_weight,
                                                                                   older_weight,
                                                                                   latest_sample_period_s,
                                                                                   previous_sample_period_s);
                float scale_sample_period_s = charge_mode_estimate_scale_period_seconds(latest_sample_period_s,
                                                                                        previous_sample_period_s);
                if (latest_sample_period_s > 0.02f) {
                    previous_sample_period_s = latest_sample_period_s;
                }
                previous_sample_tick = current_sample_tick;
                older_weight = previous_weight;
                previous_weight = current_weight;

                if (bulk_running) {
                    float bulk_stop_margin = bulk_handoff_margin + momentum_margin * 0.65f;
                    charge_mode_set_live_phase("bulk", remaining_weight, bulk_stop_margin);
                    command_curved_coarse_motor(remaining_weight, bulk_stop_margin, true);
                    float effective_bulk_flow = fmaxf(bulk_phase_flow, observed_phase_flow);
                    float time_to_bulk_stop_s = (remaining_weight - bulk_stop_margin) /
                                                fmaxf(effective_bulk_flow, 0.05f);
                    float predictive_horizon_s = fminf(1.25f,
                                                       fmaxf(0.35f, scale_sample_period_s * 1.05f));
                    float predictive_guard_s = fminf(0.18f,
                                                     fmaxf(0.05f, scale_sample_period_s * 0.18f));
                    if (remaining_weight > bulk_stop_margin &&
                        effective_bulk_flow > 0.05f &&
                        time_to_bulk_stop_s > 0.0f &&
                        time_to_bulk_stop_s <= predictive_horizon_s) {
                        uint32_t final_bulk_run_ms = (uint32_t)lroundf(
                            fmaxf(0.0f, (time_to_bulk_stop_s - predictive_guard_s) * 1000.0f));
                        if (final_bulk_run_ms > 0 && !wait_with_abort(final_bulk_run_ms)) {
                            return;
                        }

                        bulk_running = false;
                        float stop_weight = capture_coarse_stop_measurement(220, current_weight);
                        mark_coarse_stop(stop_weight, true);
                        charge_mode_command_motor(SELECT_FINE_TRICKLER_MOTOR, 0);
                        float refreshed_stop_weight = capture_coarse_stop_measurement(180, stop_weight);
                        if (is_plausible_coarse_stop_weight(refreshed_stop_weight)) {
                            stop_weight = refreshed_stop_weight;
                            mark_coarse_stop(stop_weight, false);
                        }
                        current_weight = wait_for_settled_measurement(650, stop_weight);
                        update_coarse_tail_telemetry(stop_weight, current_weight);
                        if (charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
                            return;
                        }
                        if (!have_trim_coarse) {
                            current_weight = run_guarded_coarse_topup(current_weight);
                        }
                        remaining_weight = final_target - current_weight;
                        previous_weight = current_weight;
                        older_weight = current_weight;
                        previous_sample_tick = 0;
                        previous_sample_period_s = 0.0f;
                        if (should_enter_trim_glide(remaining_weight)) {
                            start_trim_glide(remaining_weight);
                        }
                        continue;
                    }
                    if (remaining_weight <= bulk_stop_margin) {
                        bulk_running = false;
                        float stop_weight = capture_coarse_stop_measurement(220, current_weight);
                        mark_coarse_stop(stop_weight, true);
                        charge_mode_command_motor(SELECT_FINE_TRICKLER_MOTOR, 0);
                        current_weight = wait_for_settled_measurement(550, stop_weight);
                        update_coarse_tail_telemetry(stop_weight, current_weight);
                        if (charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
                            return;
                        }
                        if (!have_trim_coarse) {
                            current_weight = run_guarded_coarse_topup(current_weight);
                        }
                        remaining_weight = final_target - current_weight;
                        previous_weight = current_weight;
                        older_weight = current_weight;
                        previous_sample_tick = 0;
                        previous_sample_period_s = 0.0f;
                        if (should_enter_trim_glide(remaining_weight)) {
                            start_trim_glide(remaining_weight);
                        }
                    }
                    continue;
                }

                if (trim_running) {
                    float trim_stop_live = trim_stop_margin + momentum_margin * 0.55f;
                    charge_mode_set_live_phase("trim", remaining_weight, trim_stop_live);
                    command_curved_coarse_motor(remaining_weight, trim_stop_live, false);
                    if (remaining_weight <= trim_stop_live) {
                        trim_running = false;
                        float stop_weight = capture_coarse_stop_measurement(180, current_weight);
                        mark_coarse_stop(stop_weight, true);
                        charge_mode_command_motor(SELECT_FINE_TRICKLER_MOTOR, 0);
                        current_weight = wait_for_settled_measurement(450, stop_weight);
                        update_coarse_tail_telemetry(stop_weight, current_weight);
                        if (charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
                            return;
                        }
                        previous_weight = current_weight;
                        older_weight = current_weight;
                        previous_sample_tick = 0;
                        previous_sample_period_s = 0.0f;
                    }
                    continue;
                }

                if (remaining_weight <= charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold &&
                    (!final_recovery_active || remaining_weight <= target_tolerance)) {
                    float fine_stop_weight = current_weight;
                    if (final_recovery_active) {
                        stop_recovery_motor_timer();
                    }
                    stop_all_motors();
                    current_weight = wait_for_settled_measurement(900, fine_stop_weight);
                    mark_fine_stop(fine_stop_weight, current_weight);
                    if (charge_mode_is_valid_scale_measurement(current_weight)) {
                        charge_mode_update_true_final_measurement(current_weight);
                    }

                    float settled_under_weight = target_weight - current_weight;
                    if (charge_mode_is_valid_scale_measurement(current_weight) &&
                        settled_under_weight > target_tolerance) {
                        if (!final_recovery_active) {
                            final_recovery_active = true;
                            final_recovery_start_tick = xTaskGetTickCount();
                            recovery_last_progress_tick = final_recovery_start_tick;
                            live_recovery_start_weight_gn = current_weight;
                            live_recovery_exit_reason = RECOVERY_EXIT_NONE;
                        }
                        live_recovery_end_weight_gn = current_weight;
                        micro_heal_active = settled_under_weight <= micro_heal_entry_gn;
                        previous_weight = current_weight;
                        older_weight = current_weight;
                        previous_sample_tick = 0;
                        previous_sample_period_s = 0.0f;
                        charge_mode_set_live_phase(micro_heal_active ? "micro_heal" : "fine_recover",
                                                   settled_under_weight,
                                                   micro_heal_active ? target_tolerance * 0.50f
                                                                     : target_tolerance);
                        continue;
                    }

                    if (final_recovery_active) {
                        live_recovery_end_weight_gn = current_weight;
                        live_recovery_exit_reason =
                            current_weight - target_weight > target_tolerance
                                ? RECOVERY_EXIT_OVER
                                : RECOVERY_EXIT_TOLERANCE;
                    }
                    else {
                        live_recovery_exit_reason =
                            current_weight - target_weight > target_tolerance
                                ? RECOVERY_EXIT_OVER
                                : RECOVERY_EXIT_GUARD_STOP;
                    }
                    break;
                }

                const bool micro_finish_zone =
                    micro_heal_active ||
                    final_recovery_active ||
                    remaining_weight <= fmaxf(0.12f, target_tolerance * 4.0f);
                const float active_tail_confidence =
                    micro_finish_zone ? micro_tail_confidence : fast_tail_confidence;
                const float predicted_fine_tail =
                    micro_finish_zone ? learned_micro_tail : fine_tail;
                float learned_tail_guard =
                    predicted_fine_tail +
                    finish_safety_bias * (1.10f - active_tail_confidence * 0.45f) +
                    ai_config->noise_margin * (micro_finish_zone ? 0.12f : 0.25f);
                learned_tail_guard += steering_fine_bias;
                if (low_tail_tube && active_tail_confidence >= 0.35f) {
                    learned_tail_guard -= target_tolerance * 0.35f;
                }
                else if (high_tail_tube) {
                    learned_tail_guard += target_tolerance * 0.60f;
                }
                float min_tail_guard = micro_finish_zone
                    ? (low_tail_tube ? target_tolerance * 0.35f : target_tolerance * 0.65f)
                    : target_tolerance * 1.20f;
                float max_tail_guard = micro_finish_zone
                    ? (low_tail_tube ? 0.070f : 0.135f)
                    : fmaxf(0.16f, fine_window * 0.45f);
                learned_tail_guard = fmaxf(min_tail_guard,
                                           fminf(max_tail_guard, learned_tail_guard));

                float finish_buffer_gn = micro_finish_zone
                    ? fmaxf(target_tolerance * 1.50f, learned_tail_guard * 0.85f)
                    : fmaxf(0.12f, fminf(0.24f, learned_tail_guard + fine_tail * 0.35f));
                finish_buffer_gn = fmaxf(low_tail_tube ? target_tolerance * 1.20f : 0.035f,
                                         fminf(finish_buffer_gn, micro_finish_zone ? 0.12f : 0.26f));

                const float finish_time_target_s = micro_finish_zone ? 2.20f : 1.45f;
                float fine_speed_ceiling = fine_speed;
                float recovery_micro_speed = runtime_model.fine_recovery_speed_rps > 0.0f
                    ? runtime_model.fine_recovery_speed_rps
                    : fmaxf(fine_trickler_min_speed, fminf(0.20f, fine_speed * 0.04f));
                recovery_micro_speed *= (1.0f + steering_recovery_speed_scale);
                recovery_micro_speed = fmaxf(fine_trickler_min_speed,
                                             fminf(recovery_micro_speed,
                                                   fminf(low_tail_tube ? 0.75f : 0.55f, fine_speed)));
                float recovery_micro_flow = runtime_model.fine_recovery_flow_gps > 0.001f
                    ? runtime_model.fine_recovery_flow_gps
                    : ai_model_predict_flow_gps(&runtime_model,
                                                AI_MOTOR_MODE_FINE_ONLY,
                                                recovery_micro_speed);
                if (runtime_model.fine_micro_flow_gps > 0.001f &&
                    micro_tail_confidence >= 0.25f) {
                    recovery_micro_flow = runtime_model.fine_micro_flow_gps;
                }
                if (machine_calibrated &&
                    runtime_model.machine.micro_open_loop_flow_gps > 0.001f) {
                    recovery_micro_flow = fmaxf(recovery_micro_flow,
                                                runtime_model.machine.micro_open_loop_flow_gps);
                }
                if (recovery_micro_flow <= 0.001f && fine_speed > 0.0f) {
                    recovery_micro_flow = fine_flow * (recovery_micro_speed / fine_speed);
                }
                recovery_micro_flow = fmaxf(recovery_micro_flow, 0.004f);
                if (final_recovery_active) {
                    float under_weight = fmaxf(recovery_target - current_weight, 0.0f);
                    float recovery_speed_ratio = 0.08f;
                    if (micro_heal_active) {
                        recovery_speed_ratio = recovery_micro_speed / fmaxf(fine_speed, 0.001f);
                    }
                    else if (under_weight > 0.35f) {
                        recovery_speed_ratio = 0.30f;
                    }
                    else if (under_weight > 0.18f) {
                        recovery_speed_ratio = 0.18f;
                    }
                    else if (under_weight > 0.10f) {
                        recovery_speed_ratio = 0.12f;
                    }
                    else if (low_tail_tube && under_weight > target_tolerance * 1.60f) {
                        recovery_speed_ratio = 0.10f;
                    }
                    fine_speed_ceiling = fmaxf(fine_trickler_min_speed,
                                               fminf(fine_speed, fine_speed * recovery_speed_ratio));
                    if (micro_heal_active) {
                        fine_speed_ceiling = fmaxf(fine_trickler_min_speed,
                                                   fminf(low_tail_tube ? recovery_micro_speed * 1.35f
                                                                      : recovery_micro_speed,
                                                         fine_speed_ceiling));
                    }
                }

                float finish_min_flow = fmaxf(finish_buffer_gn / finish_time_target_s,
                                              ai_config->noise_margin * 0.50f);
                float finish_min_speed = ai_model_estimate_speed_for_fine_flow(&runtime_model,
                                                                               finish_min_flow,
                                                                               fine_speed,
                                                                               fine_flow,
                                                                               fine_trickler_min_speed,
                                                                               fine_speed_ceiling);
                if (final_recovery_active) {
                    float under_weight = fmaxf(recovery_target - current_weight, 0.0f);
                    float recovery_speed_floor = fine_speed * (low_tail_tube ? 0.035f : 0.020f);
                    if (micro_heal_active) {
                        recovery_speed_floor = low_tail_tube
                            ? recovery_micro_speed * 1.08f
                            : recovery_micro_speed;
                    }
                    else if (under_weight > 0.25f) {
                        recovery_speed_floor = fine_speed * 0.060f;
                    }
                    else if (under_weight > 0.08f) {
                        recovery_speed_floor = fine_speed * 0.035f;
                    }
                    finish_min_speed = fmaxf(finish_min_speed,
                                             fminf(fine_speed_ceiling,
                                                   fmaxf(fine_trickler_min_speed,
                                                         recovery_speed_floor)));
                }

                float active_fine_window = final_recovery_active
                    ? fmaxf(0.45f, fminf(fine_window * 0.55f, 1.40f))
                    : fine_window;
                float control_span = fmaxf(active_fine_window - finish_buffer_gn, 0.30f);
                float curve_position = fmaxf(0.0f,
                                             fminf(1.0f,
                                                   (remaining_weight - finish_buffer_gn) / control_span));
                float curved_speed = finish_min_speed +
                    (fine_speed_ceiling - finish_min_speed) * powf(curve_position, 1.45f);
                curved_speed = fmaxf(finish_min_speed, fminf(curved_speed, fine_speed_ceiling));

                float curved_flow = ai_model_predict_flow_gps(&runtime_model,
                                                              AI_MOTOR_MODE_FINE_ONLY,
                                                              curved_speed);
                if (curved_flow <= 0.005f && fine_speed > 0.0f) {
                    curved_flow = fine_flow * (curved_speed / fine_speed);
                }
                curved_flow = fmaxf(curved_flow, 0.005f);

                float scale_lag_margin = micro_finish_zone
                    ? fminf(curved_flow * 0.18f, fmaxf(0.004f, learned_tail_guard * 0.35f))
                    : fminf(curved_flow * 0.26f, fmaxf(0.018f, finish_buffer_gn * 0.55f));
                float fine_tail_margin = predicted_fine_tail;
                float bias_stop_margin =
                                         finish_safety_bias * (1.0f - active_tail_confidence * 0.35f) +
                                         steering_fine_bias;
                float fine_stop_margin = fine_tail_margin +
                                         scale_lag_margin +
                                         momentum_margin * (micro_finish_zone ? 0.20f : 0.50f) +
                                         bias_stop_margin;
                fine_stop_margin = fmaxf(fine_stop_margin,
                                         fmaxf(micro_finish_zone
                                                   ? (low_tail_tube ? target_tolerance * 0.35f
                                                                    : target_tolerance * 0.55f)
                                                   : target_tolerance * 1.05f,
                                               finish_buffer_gn * 0.45f));
                float max_fine_stop_margin = micro_finish_zone
                    ? fmaxf(learned_tail_guard,
                            fminf(low_tail_tube ? 0.085f : 0.145f, learned_tail_guard * 1.35f))
                    : fmaxf(fine_window * 0.75f, 0.16f);
                if (micro_finish_zone) {
                    fine_stop_margin = fmaxf(fine_stop_margin, learned_tail_guard);
                }
                fine_stop_margin = fminf(fine_stop_margin, max_fine_stop_margin);
                if (micro_heal_active) {
                    fine_stop_margin = learned_micro_tail * (low_tail_tube ? 0.45f : 0.75f) +
                                       recovery_micro_flow * (low_tail_tube ? 0.025f : 0.040f) +
                                       momentum_margin * 0.12f +
                                       fmaxf(steering_fine_bias, -target_tolerance * 0.35f);
                    fine_stop_margin = fmaxf(low_tail_tube ? target_tolerance * 0.30f
                                                           : target_tolerance * 0.45f,
                                             fminf(low_tail_tube ? target_tolerance * 0.85f
                                                                : target_tolerance * 1.15f,
                                                   fine_stop_margin));
                    if (recovery_last_progress_tick != 0 &&
                        (xTaskGetTickCount() - recovery_last_progress_tick) >
                            pdMS_TO_TICKS(low_tail_tube ? 1500 : 2200) &&
                        remaining_weight > target_tolerance * 1.40f) {
                        live_recovery_stall_count++;
                        recovery_last_progress_tick = xTaskGetTickCount();
                        fine_speed_ceiling = fminf(fine_speed, fmaxf(fine_speed_ceiling,
                                                                     recovery_micro_speed *
                                                                         (low_tail_tube ? 2.00f : 1.50f)));
                        curved_speed = fmaxf(curved_speed, fine_speed_ceiling);
                    }
                }
                bool recovery_approach_active =
                    final_recovery_active &&
                    !micro_heal_active &&
                    charge_mode_is_valid_scale_measurement(current_weight) &&
                    remaining_weight > micro_heal_entry_gn;
                if (recovery_approach_active) {
                    float approach_stop_margin =
                        fmaxf(micro_heal_entry_gn,
                              fminf(0.260f,
                                    learned_tail_guard + target_tolerance * 2.5f));
                    fine_stop_margin = fmaxf(fine_stop_margin, approach_stop_margin);
                    fine_stop_margin = fminf(fine_stop_margin,
                                             fmaxf(0.240f,
                                                   micro_heal_entry_gn +
                                                       target_tolerance * 2.0f));

                    float approach_speed_cap =
                        fmaxf(recovery_micro_speed * 5.5f, fine_speed * 0.11f);
                    if (remaining_weight > 0.45f) {
                        approach_speed_cap = fmaxf(approach_speed_cap,
                                                   fine_speed * 0.16f);
                    }
                    approach_speed_cap = fmaxf(fine_trickler_min_speed,
                                               fminf(approach_speed_cap,
                                                     fminf(fine_speed_ceiling,
                                                           fine_speed * 0.22f)));
                    curved_speed = fminf(curved_speed, approach_speed_cap);
                }

                float recovery_feed_stop_margin = low_tail_tube
                    ? target_tolerance * 0.85f
                    : fmaxf(target_tolerance * 1.15f,
                            fminf(high_tail_tube ? 0.110f : 0.085f,
                                  learned_micro_tail * (high_tail_tube ? 0.85f : 0.72f) +
                                      target_tolerance * (high_tail_tube ? 0.50f : 0.35f)));
                bool recovery_must_feed =
                    final_recovery_active &&
                    charge_mode_is_valid_scale_measurement(current_weight) &&
                    remaining_weight > fmaxf(recovery_force_feed_gn, recovery_feed_stop_margin) &&
                    remaining_weight <= micro_heal_entry_gn;
                if (recovery_must_feed) {
                    // Recovery is allowed to be slow, but not motionless while
                    // the live scale still shows a salvageable underthrow.
                    fine_stop_margin = fminf(fine_stop_margin, recovery_feed_stop_margin);
                    float feed_floor = fmaxf(fine_trickler_min_speed, recovery_micro_speed);
                    if (remaining_weight > 0.12f) {
                        feed_floor = fmaxf(feed_floor, recovery_micro_speed * 1.75f);
                    }
                    else if (remaining_weight > 0.07f) {
                        feed_floor = fmaxf(feed_floor, recovery_micro_speed * 1.35f);
                    }
                    feed_floor = fminf(feed_floor, fine_speed_ceiling);
                    curved_speed = fmaxf(curved_speed, feed_floor);
                }
                charge_mode_set_live_phase(micro_heal_active
                                               ? "micro_heal"
                                               : (recovery_approach_active
                                                      ? "fine_recover"
                                                      : (micro_finish_zone ? "fine_micro" : "fine_fast")),
                                           remaining_weight,
                                           fine_stop_margin);

                if (remaining_weight <= fine_stop_margin && !recovery_must_feed) {
                    float fine_stop_weight = current_weight;
                    if (final_recovery_active) {
                        stop_recovery_motor_timer();
                    }
                    charge_mode_command_motor(SELECT_FINE_TRICKLER_MOTOR, 0);
                    current_weight = wait_for_settled_measurement(1400, current_weight);
                    mark_fine_stop(fine_stop_weight, current_weight);
                    if (charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
                        return;
                    }
                    if (charge_mode_is_valid_scale_measurement(current_weight)) {
                        charge_mode_update_true_final_measurement(current_weight);
                    }

                    float settled_under_weight = target_weight - current_weight;
                    if (charge_mode_is_valid_scale_measurement(current_weight) &&
                        settled_under_weight > target_tolerance) {
                        if (!final_recovery_active) {
                            final_recovery_active = true;
                            final_recovery_start_tick = xTaskGetTickCount();
                            recovery_last_progress_tick = final_recovery_start_tick;
                            live_recovery_start_weight_gn = current_weight;
                            live_recovery_end_weight_gn = current_weight;
                            live_recovery_exit_reason = RECOVERY_EXIT_NONE;
                        }
                        else {
                            live_recovery_end_weight_gn = current_weight;
                        }

                        if (settled_under_weight <= micro_heal_entry_gn) {
                            micro_heal_active = true;
                        }

                        uint32_t recovery_elapsed_ms =
                            (uint32_t)((xTaskGetTickCount() - final_recovery_start_tick) *
                                       portTICK_PERIOD_MS);
                        if (recovery_elapsed_ms < 30000u ||
                            (settled_under_weight > recovery_force_feed_gn &&
                             live_recovery_motor_on_ms < 3000u)) {
                            previous_weight = current_weight;
                            older_weight = current_weight;
                            previous_sample_tick = 0;
                            previous_sample_period_s = 0.0f;
                            charge_mode_set_live_phase(micro_heal_active ? "micro_heal" : "fine_recover",
                                                       settled_under_weight,
                                                       micro_heal_active ? target_tolerance * 0.50f
                                                                         : target_tolerance);
                            continue;
                        }
                        live_recovery_exit_reason = RECOVERY_EXIT_TIMEOUT;
                    }
                    else if (charge_mode_is_valid_scale_measurement(current_weight) &&
                             current_weight - target_weight > target_tolerance) {
                        live_recovery_exit_reason = RECOVERY_EXIT_OVER;
                    }
                    else if (final_recovery_active) {
                        live_recovery_exit_reason = RECOVERY_EXIT_TOLERANCE;
                    }
                    else {
                        live_recovery_exit_reason = RECOVERY_EXIT_GUARD_STOP;
                    }
                    break;
                }

                if (final_recovery_active) {
                    if (recovery_last_progress_tick == 0 ||
                        current_weight > live_recovery_end_weight_gn + 0.004f) {
                        recovery_last_progress_tick = xTaskGetTickCount();
                        live_recovery_end_weight_gn = current_weight;
                    }
                    if (micro_heal_active && high_tail_tube) {
                        float micro_target_margin =
                            fmaxf(target_tolerance * 0.85f,
                                  fminf(0.055f,
                                        fmaxf(learned_micro_tail * 0.45f,
                                              target_tolerance * 0.90f)));
                        if (recovery_must_feed) {
                            micro_target_margin = fminf(micro_target_margin,
                                                        target_tolerance * 0.80f);
                        }
                        float desired_add_gn = remaining_weight - micro_target_margin;
                        if (desired_add_gn <= 0.0f) {
                            desired_add_gn = fmaxf(0.0f, remaining_weight - target_tolerance * 0.70f);
                        }

                        if (desired_add_gn <= 0.003f && !recovery_must_feed) {
                            charge_mode_command_motor(SELECT_FINE_TRICKLER_MOTOR, 0);
                            current_weight = wait_for_settled_measurement(850, current_weight);
                            mark_fine_stop(current_weight, current_weight);
                            if (charge_mode_is_valid_scale_measurement(current_weight)) {
                                charge_mode_update_true_final_measurement(current_weight);
                                live_recovery_end_weight_gn = current_weight;
                            }
                            float after_wait_under = target_weight - current_weight;
                            if (charge_mode_is_valid_scale_measurement(current_weight) &&
                                after_wait_under > target_tolerance &&
                                (uint32_t)((xTaskGetTickCount() - final_recovery_start_tick) *
                                           portTICK_PERIOD_MS) < 30000u) {
                                previous_weight = current_weight;
                                older_weight = current_weight;
                                previous_sample_tick = 0;
                                previous_sample_period_s = 0.0f;
                                continue;
                            }
                            live_recovery_exit_reason =
                                current_weight - target_weight > target_tolerance
                                    ? RECOVERY_EXIT_OVER
                                    : RECOVERY_EXIT_TOLERANCE;
                            break;
                        }

                        float dose_factor = remaining_weight > 0.09f ? 0.58f : 0.45f;
                        uint32_t micro_dose_ms = (uint32_t)lroundf(
                            (desired_add_gn / fmaxf(recovery_micro_flow, 0.012f)) *
                            1000.0f *
                            dose_factor);
                        uint32_t micro_max_ms = remaining_weight < 0.050f
                            ? 260u
                            : (remaining_weight < 0.085f ? 450u : 700u);
                        micro_dose_ms = (uint32_t)fmaxf(70.0f,
                                                        fminf((float)micro_max_ms,
                                                              (float)micro_dose_ms));

                        float fine_stop_weight = current_weight;
                        command_recovery_fine_motor(recovery_micro_speed);
                        if (!wait_with_abort(micro_dose_ms)) {
                            live_recovery_exit_reason = RECOVERY_EXIT_ABORT;
                            return;
                        }
                        command_recovery_fine_motor(0);

                        float stop_weight = get_latest_measurement(120, fine_stop_weight);
                        current_weight = wait_for_settled_measurement(850, stop_weight);
                        mark_fine_stop(stop_weight, current_weight);
                        if (charge_mode_is_valid_scale_measurement(current_weight)) {
                            charge_mode_update_true_final_measurement(current_weight);
                            live_recovery_end_weight_gn = current_weight;
                        }
                        if (charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
                            return;
                        }

                        float settled_under_weight = target_weight - current_weight;
                        if (charge_mode_is_valid_scale_measurement(current_weight) &&
                            current_weight - target_weight > target_tolerance) {
                            live_recovery_exit_reason = RECOVERY_EXIT_OVER;
                            break;
                        }
                        if (charge_mode_is_valid_scale_measurement(current_weight) &&
                            settled_under_weight <= target_tolerance) {
                            live_recovery_exit_reason = RECOVERY_EXIT_TOLERANCE;
                            break;
                        }
                        uint32_t recovery_elapsed_ms =
                            (uint32_t)((xTaskGetTickCount() - final_recovery_start_tick) *
                                       portTICK_PERIOD_MS);
                        if (recovery_elapsed_ms >= 30000u &&
                            live_recovery_motor_on_ms >= 3000u) {
                            live_recovery_exit_reason = RECOVERY_EXIT_TIMEOUT;
                            break;
                        }

                        previous_weight = current_weight;
                        older_weight = current_weight;
                        previous_sample_tick = 0;
                        previous_sample_period_s = 0.0f;
                        continue;
                    }
                    command_recovery_fine_motor(curved_speed);
                }
                else {
                    charge_mode_command_motor(SELECT_FINE_TRICKLER_MOTOR, curved_speed);
                }
            }
            stop_recovery_motor_timer();
        }
    }

    live_runtime_model_active = runtime_model_enabled;

    if (!ai_active && !runtime_model_enabled) {
        float integral = 0.0f;
        float last_error = 0.0f;
        TickType_t last_sample_tick = xTaskGetTickCount();
        bool should_coarse_trickler_move = true;
        int scale_fail_count = 0;

        while (true) {
            if (button_wait_for_input(false) == BUTTON_RST_PRESSED) {
                emergency_exit();
                return;
            }

            float current_weight = 0.0f;
            if (!scale_block_wait_for_next_measurement(200, &current_weight)) {
                scale_fail_count++;
                if (scale_fail_count >= 10) {
                    emergency_exit();
                    return;
                }
                continue;
            }
            if (!charge_mode_is_valid_scale_measurement(current_weight)) {
                scale_fail_count++;
                if (scale_fail_count >= 10) {
                    emergency_exit();
                    return;
                }
                continue;
            }
            scale_fail_count = 0;

            TickType_t current_sample_tick = xTaskGetTickCount();
            float error = charge_mode_config.target_charge_weight - current_weight;
            charge_mode_set_progress_colour(error, charge_mode_config.target_charge_weight);

            if (error <= charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold) {
                stop_all_motors();
                break;
            }

            if (should_coarse_trickler_move &&
                error < charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold) {
                should_coarse_trickler_move = false;
                mark_coarse_stop(current_weight, true);
            }

            float elapsed_ms = (float)((current_sample_tick - last_sample_tick) * portTICK_PERIOD_MS);
            if (elapsed_ms <= 0.0f) {
                elapsed_ms = 1.0f;
            }

            integral += error;
            float derivative = (error - last_error) / elapsed_ms;

            if (should_coarse_trickler_move) {
                float coarse_pid = current_profile->coarse_kp * error +
                                   current_profile->coarse_ki * integral +
                                   current_profile->coarse_kd * derivative;
                float coarse_speed = fmaxf(coarse_trickler_min_speed,
                                           fminf(coarse_pid, coarse_trickler_max_speed));
                charge_mode_command_motor(SELECT_COARSE_TRICKLER_MOTOR, coarse_speed);
            }
            else {
                charge_mode_command_motor(SELECT_COARSE_TRICKLER_MOTOR, 0);
            }

            bool use_pulse = charge_mode_config.eeprom_charge_mode_data.pulse_mode_enabled &&
                             error < charge_mode_config.eeprom_charge_mode_data.pulse_threshold &&
                             error > charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold;

            if (use_pulse) {
                float pulse_speed = fmaxf(fine_trickler_min_speed, fine_trickler_max_speed * 0.3f);
                if (!run_motor_for_duration(SELECT_FINE_TRICKLER_MOTOR,
                                            pulse_speed,
                                            charge_mode_config.eeprom_charge_mode_data.pulse_duration_ms)) {
                    return;
                }
                if (!wait_with_abort(charge_mode_config.eeprom_charge_mode_data.pulse_wait_ms)) {
                    return;
                }
            }
            else {
                float fine_pid = current_profile->fine_kp * error +
                                 current_profile->fine_ki * integral +
                                 current_profile->fine_kd * derivative;
                if (fine_pid <= 0.0f) {
                    charge_mode_command_motor(SELECT_FINE_TRICKLER_MOTOR, 0);
                }
                else {
                    float fine_speed = fmaxf(fine_trickler_min_speed,
                                             fminf(fine_pid, fine_trickler_max_speed));
                    charge_mode_command_motor(SELECT_FINE_TRICKLER_MOTOR, fine_speed);
                }
            }

            last_sample_tick = current_sample_tick;
            last_error = error;
        }
    }

    TickType_t end_tick = xTaskGetTickCount();
    TickType_t elapsed_ticks = end_tick - charge_start_tick;
    last_charge_elapsed_seconds = (float)(elapsed_ticks * portTICK_PERIOD_MS) / 1000.0f;

    float total_ms = (float)(elapsed_ticks * portTICK_PERIOD_MS);
    float coarse_ms = coarse_stop_tick
        ? (float)((coarse_stop_tick - charge_start_tick) * portTICK_PERIOD_MS)
        : 0.0f;
    coarse_ms = fminf(coarse_ms, total_ms);
    float fine_ms = fmaxf(0.0f, total_ms - coarse_ms);

    if (record_ai_drop) {
        ai_pending_telemetry = pending_ai_drop;
        ai_record_pending = true;
    }
    else if (runtime_model_enabled || charge_mode_config.eeprom_charge_mode_data.ml_data_collection_enabled) {
        ml_record_pending = true;
        ml_coarse_time_ms = coarse_ms;
        ml_fine_time_ms = fine_ms;
    }

    if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
        servo_gate_set_ratio(SERVO_GATE_RATIO_CLOSED, true);
    }

    if (!ai_active &&
        !runtime_model_enabled &&
        charge_mode_config.eeprom_charge_mode_data.precharge_enable &&
        servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
        vTaskDelay(pdMS_TO_TICKS(500));
        charge_mode_command_motor(SELECT_COARSE_TRICKLER_MOTOR,
                        charge_mode_config.eeprom_charge_mode_data.precharge_speed_rps);
        vTaskDelay(pdMS_TO_TICKS(charge_mode_config.eeprom_charge_mode_data.precharge_time_ms));
        charge_mode_command_motor(SELECT_COARSE_TRICKLER_MOTOR, 0);
    }
    else {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    charge_mode_set_live_phase("stabilizing", 0.0f, 0.0f);
    charge_mode_config.charge_mode_state = CHARGE_MODE_STABILIZING;
}

void charge_mode_stabilize() {
    snprintf(title_string, sizeof(title_string), "Stabilizing...");
    charge_mode_set_live_phase("stabilizing", 0.0f, 0.0f);

    last_final_weight_valid = false;
    last_final_weight_gn = 0.0f;

    // Wait for scale to stabilize after motors stopped
    if (charge_mode_config.eeprom_charge_mode_data.stabilization_enabled) {
        // Fixed configured wait
        vTaskDelay(pdMS_TO_TICKS(charge_mode_config.eeprom_charge_mode_data.stabilization_time_ms));
        last_final_weight_valid = charge_mode_capture_settled_measurement(
            1200,
            FINAL_WEIGHT_SETTLE_SAMPLE_COUNT,
            &last_final_weight_gn);
    } else {
        // Adaptive: collect samples until SD < set_point_sd_margin or 3s timeout
        FloatRingBuffer stab_buffer(5);
        TickType_t stab_start = xTaskGetTickCount();
        const TickType_t stab_timeout = pdMS_TO_TICKS(3000);
        while ((xTaskGetTickCount() - stab_start) < stab_timeout) {
            float stab_weight;
            if (scale_block_wait_for_next_measurement(200, &stab_weight) &&
                charge_mode_is_valid_scale_measurement(stab_weight)) {
                stab_buffer.enqueue(stab_weight);
                if (stab_buffer.getCounter() >= 5 &&
                    stab_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin) {
                    break;
                }
            }
        }
        if (stab_buffer.getCounter() > 0) {
            charge_mode_update_true_final_measurement(stab_buffer.getMean());
        }
        else {
            float fallback_weight = 0.0f;
            if (charge_mode_try_get_current_measurement(&fallback_weight)) {
                charge_mode_update_true_final_measurement(fallback_weight);
            }
        }
    }

    if (last_final_weight_valid) {
        charge_mode_update_true_final_measurement(last_final_weight_gn);
    }

    if (last_final_weight_valid && !last_charge_was_ai_tuning) {
        charge_mode_salvage_undercharge(&last_final_weight_gn);
        if (charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
            return;
        }
    }

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_CUP_REMOVAL;
}

void charge_mode_wait_for_cup_removal() {
    // Update current status
    snprintf(title_string, sizeof(title_string), "Remove Cup");

    FloatRingBuffer data_buffer(5);
    FloatRingBuffer truth_buffer(5);

    // Use the final settled reading captured before cup handling can disturb the scale.
    float current_measurement = 0.0f;
    bool current_measurement_valid = last_final_weight_valid;
    if (current_measurement_valid) {
        current_measurement = last_final_weight_gn;
    }
    else {
        current_measurement_valid = charge_mode_capture_settled_measurement(
            1200,
            FINAL_WEIGHT_SETTLE_SAMPLE_COUNT,
            &current_measurement);
    }
    bool suppress_charge_result = last_charge_was_ai_tuning;
    float result_tolerance = charge_mode_acceptance_tolerance();
    float live_result_measurement = scale_get_current_measurement();
    float result_measurement = charge_mode_choose_result_measurement(current_measurement,
                                                                     current_measurement_valid,
                                                                     live_result_measurement);
    bool result_measurement_valid = current_measurement_valid ||
                                    charge_mode_is_valid_scale_measurement(live_result_measurement);
    if (result_measurement_valid) {
        charge_mode_update_true_final_measurement(result_measurement);
        current_measurement = last_final_weight_gn;
        current_measurement_valid = last_final_weight_valid;
        result_measurement = last_final_weight_gn;
    }
    charge_mode_set_live_phase("remove_cup",
                               result_measurement_valid
                                   ? (charge_mode_config.target_charge_weight - result_measurement)
                                   : 0.0f,
                               result_tolerance);

    // Deferred AI tuning recording - use settled measurement for accurate weight
    bool ai_characterization_recorded = false;
    if (ai_record_pending) {
        FloatRingBuffer ai_truth_buffer(5);
        const float truth_sd_limit =
            fmaxf(charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin * 1.75f, 0.025f);
        TickType_t truth_start_tick = xTaskGetTickCount();
        while ((xTaskGetTickCount() - truth_start_tick) < pdMS_TO_TICKS(1000)) {
            float truth_weight = 0.0f;
            if (scale_block_wait_for_next_measurement(160, &truth_weight) &&
                charge_mode_is_valid_scale_measurement(truth_weight) &&
                !charge_mode_is_cup_removed_measurement(truth_weight)) {
                ai_truth_buffer.enqueue(truth_weight);
                if (ai_truth_buffer.getCounter() >= 3 &&
                    ai_truth_buffer.getSd() < truth_sd_limit) {
                    charge_mode_update_true_final_measurement(ai_truth_buffer.getMean());
                    current_measurement = last_final_weight_gn;
                    current_measurement_valid = last_final_weight_valid;
                }
            }
        }
    }
    if (ai_record_pending) {
        ai_record_pending = false;
        if (current_measurement_valid) {
            float ai_overthrow = current_measurement - ai_pending_telemetry.target_weight;
            ai_pending_telemetry.final_weight = current_measurement;
            ai_pending_telemetry.delivered_weight =
                fmaxf(0.0f, ai_pending_telemetry.final_weight - ai_pending_telemetry.start_weight);
            ai_pending_telemetry.tail_weight =
                fmaxf(0.0f, ai_pending_telemetry.final_weight - ai_pending_telemetry.stop_weight);
            ai_pending_telemetry.overthrow = ai_overthrow;
            ai_characterization_recorded = ai_tuning_record_drop(&ai_pending_telemetry);
        }
    }

    if (last_charge_was_ai_tuning && ai_characterization_recorded) {
        if (ai_tuning_is_active()) {
            bool ai_tare_ready = charge_mode_force_zero_and_wait_for_ai();
            if (charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
                return;
            }
            if (!ai_tare_ready) {
                snprintf(title_string, sizeof(title_string), "AI Tare Timeout");
                charge_mode_set_live_phase("ai_tare_timeout", 0.0f, 0.0f);
                vTaskDelay(pdMS_TO_TICKS(250));
            }
            charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_COMPLETE;
            return;
        }

        if (ai_tuning_is_complete()) {
            charge_mode_set_live_phase("ai_done", 0.0f, 0.0f);
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }
    }

    charge_mode_apply_result_state(suppress_charge_result,
                                   result_measurement_valid,
                                   result_measurement,
                                   result_tolerance);

    // Stop condition: 5 stable measurements in 300ms apart (1.5 seconds minimum)
    int underload_count = 0;
    while (true) {
        TickType_t last_sample_tick = xTaskGetTickCount();

        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }

        // Perform measurement
        float current_weight;
        if (!scale_block_wait_for_next_measurement(200, &current_weight)) {
            // If no measurement within 200ms then poll the button and retry
            continue;
        }

        if (!charge_mode_is_valid_scale_measurement(current_weight)) {
            if (charge_mode_is_underload_measurement(current_weight)) {
                underload_count++;
                if (underload_count >= 3) {
                    break;
                }
            }
            continue;
        }

        underload_count = 0;
        data_buffer.enqueue(current_weight);

        bool cup_removed_or_moving =
            charge_mode_is_cup_removed_measurement(current_weight) ||
            (current_measurement_valid &&
             current_measurement > charge_mode_config.target_charge_weight * 0.50f &&
             current_weight < charge_mode_config.target_charge_weight * 0.50f);
        if (!cup_removed_or_moving) {
            const float truth_sd_limit =
                fmaxf(charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin * 1.75f, 0.025f);
            truth_buffer.enqueue(current_weight);
            if (truth_buffer.getCounter() >= 3 &&
                truth_buffer.getSd() < truth_sd_limit) {
                charge_mode_update_true_final_measurement(truth_buffer.getMean());
            }
            float dynamic_result = current_weight;
            dynamic_result = last_final_weight_valid ? last_final_weight_gn : current_weight;
            charge_mode_set_live_phase("remove_cup",
                                       charge_mode_config.target_charge_weight - dynamic_result,
                                       result_tolerance);
            charge_mode_apply_result_state(suppress_charge_result,
                                           true,
                                           dynamic_result,
                                           result_tolerance);
        }

        // Generate stop condition
        if (data_buffer.getCounter() >= 5) {
            if (data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin && 
                data_buffer.getMean() + 10 < charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin){
                break;
            }
        }

        // Wait for next measurement
        vTaskDelayUntil(&last_sample_tick, pdMS_TO_TICKS(300));
    }

    // Deferred ML recording uses the truthful post-finish result, including delayed
    // powder that appeared while the cup was still on the scale.
    if (ml_record_pending) {
        ml_record_pending = false;
        if (last_final_weight_valid) {
            float ml_error = last_final_weight_gn - charge_mode_config.target_charge_weight;
            ai_tuning_record_charge(
                (uint8_t)profile_get_selected_idx(),
                charge_mode_config.target_charge_weight,
                ml_error,
                ml_coarse_time_ms + ml_fine_time_ms,
                live_coarse_stop_weight_gn,
                live_after_coarse_settle_weight_gn,
                live_observed_coarse_tail_gn,
                live_fine_stop_weight_gn,
                live_after_fine_settle_weight_gn,
                live_observed_fine_tail_gn,
                live_post_finish_peak_weight_gn,
                live_recovery_start_weight_gn,
                live_recovery_end_weight_gn,
                (float)live_recovery_motor_on_ms,
                (uint16_t)fminf((float)live_recovery_stall_count, 65535.0f),
                live_recovery_exit_reason
            );
        }
    }

    // Reset LED to default colour
    neopixel_led_set_colour(neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led1_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led2_colour,
                            true);

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_CUP_RETURN;
}

void charge_mode_wait_for_cup_return() { 
    charge_mode_set_live_phase("wait_return", 0.0f, 0.0f);
    // Set colour to not ready
    neopixel_led_set_colour(
        neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        true
    );

    snprintf(title_string, sizeof(title_string), "Return Cup");


    FloatRingBuffer data_buffer(5);

    while (true) {
        TickType_t last_sample_tick = xTaskGetTickCount();

        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }
        else if (button_encoder_event == BUTTON_ENCODER_PRESSED) {
            scale_config.scale_handle->force_zero();
        }

        // Perform measurement
        float current_weight;
        if (!scale_block_wait_for_next_measurement(200, &current_weight) ||
            !charge_mode_is_valid_scale_measurement(current_weight)) {
            // If no measurement within 200ms then poll the button and retry
            continue;
        }

        if (current_weight >= 0) {
            break;
        }

        // Wait for next measurement
        vTaskDelayUntil(&last_sample_tick, pdMS_TO_TICKS(20));
    }

    // Auto zero scale if enabled
    if (charge_mode_config.eeprom_charge_mode_data.auto_zero_on_cup_return) {
        scale_config.scale_handle->force_zero();
    }

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_ZERO;
}


uint8_t charge_mode_menu(bool charge_mode_skip_user_input) {
    charge_mode_menu_active = true;

    // Create target weight, if the charge mode weight is built by charge_weight_digits
    if (!charge_mode_skip_user_input) {
        switch (charge_mode_config.eeprom_charge_mode_data.decimal_places) {
            case DP_2:
                charge_mode_config.target_charge_weight = charge_weight_digits[4] * 100 + \
                                                charge_weight_digits[3] * 10 + \
                                                charge_weight_digits[2] * 1 + \
                                                charge_weight_digits[1] * 0.1 + \
                                                charge_weight_digits[0] * 0.01;
                break;
            case DP_3:
                charge_mode_config.target_charge_weight = charge_weight_digits[4] * 10 + \
                                                charge_weight_digits[3] * 1 + \
                                                charge_weight_digits[2] * 0.1 + \
                                                charge_weight_digits[1] * 0.01 + \
                                                charge_weight_digits[0] * 0.001;
                break;
            default:
                charge_mode_config.target_charge_weight = 0;
                break;
        }
    }

    // If the display task is never created then we shall create one, otherwise we shall resume the task
    if (scale_measurement_render_task_handler == NULL) {
        // The render task shall have lower priority than the current one
        UBaseType_t current_task_priority = uxTaskPriorityGet(xTaskGetCurrentTaskHandle());
        xTaskCreate(scale_measurement_render_task, "Scale Measurement Render Task", configMINIMAL_STACK_SIZE, NULL, current_task_priority - 1, &scale_measurement_render_task_handler);
    }
    else {
        vTaskResume(scale_measurement_render_task_handler);
    }

    // Enable motor on entering the charge mode
    motor_enable(SELECT_COARSE_TRICKLER_MOTOR, true);
    motor_enable(SELECT_FINE_TRICKLER_MOTOR, true);
    
    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_ZERO;

    bool quit = false;
    while (quit == false) {
        switch (charge_mode_config.charge_mode_state) {
            case CHARGE_MODE_WAIT_FOR_ZERO:
                charge_mode_wait_for_zero();
                break;
            case CHARGE_MODE_WAIT_FOR_COMPLETE:
                charge_mode_wait_for_complete();
                break;
            case CHARGE_MODE_STABILIZING:
                charge_mode_stabilize();
                break;
            case CHARGE_MODE_WAIT_FOR_CUP_REMOVAL:
                charge_mode_wait_for_cup_removal();
                // If AI tuning just completed, exit after the user removes the cup
                if (last_charge_was_ai_tuning && ai_tuning_is_complete()) {
                    charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
                }
                last_charge_was_ai_tuning = false;
                break;
            case CHARGE_MODE_WAIT_FOR_CUP_RETURN:
                charge_mode_wait_for_cup_return();
                break;
            case CHARGE_MODE_EXIT:
            default:
                quit = true;
                break;
        }
    }

    // Reset LED to default colour
    neopixel_led_set_colour(neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led1_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led2_colour,
                            true);

    // vTaskDelete(scale_measurement_render_handler);
    vTaskSuspend(scale_measurement_render_task_handler);

    // Diable motors on exiting the mode
    motor_enable(SELECT_COARSE_TRICKLER_MOTOR, false);
    motor_enable(SELECT_FINE_TRICKLER_MOTOR, false);

    charge_mode_menu_active = false;
    return 1;  // return back to main menu
}


bool charge_mode_config_init(void) {
    bool is_ok = false;

    // Read charge mode config from EEPROM
    is_ok = load_config(EEPROM_CHARGE_MODE_BASE_ADDR, &charge_mode_config.eeprom_charge_mode_data, &default_charge_mode_data, sizeof(charge_mode_config.eeprom_charge_mode_data), EEPROM_CHARGE_MODE_DATA_REV);
    if (!is_ok) {
        printf("Unable to read charge mode configuration\n");
        return is_ok;
    }

    // Register to eeprom save all
    eeprom_register_handler(charge_mode_config_save);

    return true;
}


bool charge_mode_config_save(void) {
    bool is_ok = save_config(EEPROM_CHARGE_MODE_BASE_ADDR, &charge_mode_config.eeprom_charge_mode_data, sizeof(eeprom_charge_mode_data_t));
    return is_ok;
}

bool charge_mode_is_menu_active(void) {
    return charge_mode_menu_active;
}



bool http_rest_charge_mode_config(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // c1 (str): neopixel_normal_charge_colour
    // c2 (str): neopixel_under_charge_colour
    // c3 (str): neopixel_over_charge_colour
    // c4 (str): neopixel_not_ready_colour

    // c5 (float): coarse_stop_threshold
    // c6 (float): fine_stop_threshold
    // c7 (float): set_point_sd_margin
    // c8 (float): set_point_mean_margin
    // c9 (int): decimal point enum
    // c10 (bool): precharge_enable
    // c11 (int): precharge_time_ms
    // c12 (float): precharge_speed_rps
    // c13 (float): coarse_stop_gate_ratio
    // c14 (uint32): coarse_time_target_ms (AI tuning coarse pre-charge duration)
    // c15 (uint32): total_time_target_ms  (AI tuning total time target)
    // c16 (bool): ml_data_collection_enabled
    // ee (bool): save to eeprom

    static char charge_mode_json_buffer[700];
    bool save_to_eeprom = false;

    // Control
    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "c5") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c6") == 0) {
            charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c7") == 0) {
            charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c8") == 0) {
            charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c9") == 0) {
            charge_mode_config.eeprom_charge_mode_data.decimal_places = (decimal_places_t) atoi(values[idx]);
        }
        
        // Pre charge related settings
        else if (strcmp(params[idx], "c10") == 0) {
            charge_mode_config.eeprom_charge_mode_data.precharge_enable = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "c11") == 0) {
            charge_mode_config.eeprom_charge_mode_data.precharge_time_ms = strtol(values[idx], NULL, 10);
        }
        else if (strcmp(params[idx], "c12") == 0) {
            charge_mode_config.eeprom_charge_mode_data.precharge_speed_rps = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c13") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_stop_gate_ratio = strtof(values[idx], NULL);
        }

        // AI tuning related settings
        else if (strcmp(params[idx], "c14") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_time_target_ms = (uint32_t) strtol(values[idx], NULL, 10);
        }
        else if (strcmp(params[idx], "c15") == 0) {
            charge_mode_config.eeprom_charge_mode_data.total_time_target_ms = (uint32_t) strtol(values[idx], NULL, 10);
        }
        else if (strcmp(params[idx], "c16") == 0) {
            charge_mode_config.eeprom_charge_mode_data.ml_data_collection_enabled = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "c17") == 0) {
            charge_mode_config.eeprom_charge_mode_data.auto_zero_on_cup_return = string_to_boolean(values[idx]);
        }

        // Pulse mode settings
        else if (strcmp(params[idx], "c18") == 0) {
            charge_mode_config.eeprom_charge_mode_data.pulse_mode_enabled = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "c19") == 0) {
            float val = strtof(values[idx], NULL);
            charge_mode_config.eeprom_charge_mode_data.pulse_threshold = fmaxf(0.3f, fminf(1.0f, val));
        }
        else if (strcmp(params[idx], "c20") == 0) {
            charge_mode_config.eeprom_charge_mode_data.pulse_duration_ms = (uint32_t) strtol(values[idx], NULL, 10);
        }
        else if (strcmp(params[idx], "c21") == 0) {
            charge_mode_config.eeprom_charge_mode_data.pulse_wait_ms = (uint32_t) strtol(values[idx], NULL, 10);
        }

        // Scale stabilization settings
        else if (strcmp(params[idx], "c22") == 0) {
            charge_mode_config.eeprom_charge_mode_data.stabilization_enabled = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "c23") == 0) {
            charge_mode_config.eeprom_charge_mode_data.stabilization_time_ms = (uint32_t) strtol(values[idx], NULL, 10);
        }

        // LED related settings
        else if (strcmp(params[idx], "c1") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c2") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c3") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c4") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "ee") == 0) {
            save_to_eeprom = string_to_boolean(values[idx]);
        }
    }
    
    // Perform action
    if (save_to_eeprom) {
        charge_mode_config_save();
    }

    // Response
    snprintf(charge_mode_json_buffer, 
             sizeof(charge_mode_json_buffer),
             "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n"
             "{\"c1\":\"#%06lx\",\"c2\":\"#%06lx\",\"c3\":\"#%06lx\",\"c4\":\"#%06lx\","
             "\"c5\":%.3f,\"c6\":%.3f,\"c7\":%.3f,\"c8\":%.3f,\"c9\":%d,\"c10\":%s,\"c11\":%ld,\"c12\":%0.3f,\"c13\":%0.3f,"
             "\"c14\":%lu,\"c15\":%lu,\"c16\":%s,\"c17\":%s,"
             "\"c18\":%s,\"c19\":%.3f,\"c20\":%lu,\"c21\":%lu,"
             "\"c22\":%s,\"c23\":%lu}",
             charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold,
             charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold,
             charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin,
             charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin,
             charge_mode_config.eeprom_charge_mode_data.decimal_places,
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.precharge_enable),
             charge_mode_config.eeprom_charge_mode_data.precharge_time_ms,
             charge_mode_config.eeprom_charge_mode_data.precharge_speed_rps,
             charge_mode_config.eeprom_charge_mode_data.coarse_stop_gate_ratio,
             (unsigned long) charge_mode_config.eeprom_charge_mode_data.coarse_time_target_ms,
             (unsigned long) charge_mode_config.eeprom_charge_mode_data.total_time_target_ms,
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.ml_data_collection_enabled),
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.auto_zero_on_cup_return),
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.pulse_mode_enabled),
             charge_mode_config.eeprom_charge_mode_data.pulse_threshold,
             (unsigned long) charge_mode_config.eeprom_charge_mode_data.pulse_duration_ms,
             (unsigned long) charge_mode_config.eeprom_charge_mode_data.pulse_wait_ms,
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.stabilization_enabled),
             (unsigned long) charge_mode_config.eeprom_charge_mode_data.stabilization_time_ms);

    size_t data_length = strlen(charge_mode_json_buffer);
    file->data = charge_mode_json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}


bool http_rest_charge_mode_state(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // s0 (float): Charge weight set point (unitless)
    // s1 (float): Current weight (unitless)
    // s2 (charge_mode_state_t | int): Charge mode state
    // s3 (uint32_t): Charge mode event
    // s4 (string): Profile Name
    // s5 (string): Elapsed time in seconds, live during charging
    // s6 (float|string): Last settled final charge weight, or "nan" when unavailable
    // s7 (bool): Runtime-model controller active
    // s8 (string): Live charge phase
    // s9 (float): Live final target used by the runtime controller
    // s10 (float): Live bulk handoff margin
    // s11 (float): Live trim stop margin
    // s12 (float): Live fine window
    // s13 (float): Live remaining weight relative to the active target
    // s14 (float): Live stop margin for the active phase
    // s15 (uint32_t): Salvage cycles used after stabilization
    // s16 (float): Final acceptance tolerance used for pass/fail
    // s17 (float|string): Coarse stop weight captured by controller
    // s18 (float|string): Settled weight after coarse stopped
    // s19 (float|string): Observed coarse tail after stop
    // s20 (uint32_t): Open-loop bulk deadline in milliseconds
    // s21 (float): Last coarse motor command in rps
    // s22 (float): Last fine motor command in rps
    // s23 (float/null): Fine motor stop weight
    // s24 (float/null): Weight after fine settle
    // s25 (float/null): Observed fine tail
    // s26 (float/null): Highest valid post-finish weight while cup remained on scale
    // s27 (float/null): Fine recovery start weight
    // s28 (float/null): Fine recovery end/latest weight
    // s29 (uint32_t): Fine recovery motor-on time in milliseconds
    // s30 (uint32_t): Fine recovery stall count
    // s31 (uint8_t): Fine recovery exit reason
    // s32 (string): AI planner decision/explanation
    // s33 (float): Selected model bulk/coarse speed in rps
    // s34 (float): Selected model bulk/coarse flow in gn/s
    // s35 (float): Selected model bulk/coarse tail in gn
    // s36 (float): Selected model fine speed in rps
    // s37 (float): Selected model fine flow in gn/s
    // s38 (float): Selected model fine tail in gn

    static char charge_mode_json_buffer[1800];
    char elapsed_time_buffer[16] = {0};

    // Control
    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "s0") == 0) {
            charge_mode_config.target_charge_weight = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "s2") == 0) {
            charge_mode_state_t new_state = (charge_mode_state_t) atoi(values[idx]);

            // Exit
            if (new_state == CHARGE_MODE_EXIT && charge_mode_config.charge_mode_state != CHARGE_MODE_EXIT) {
                if (charge_mode_menu_active && encoder_event_queue != NULL) {
                    ButtonEncoderEvent_t button_event = BUTTON_RST_PRESSED;
                    (void)xQueueSend(encoder_event_queue, &button_event, pdMS_TO_TICKS(250));
                }
                charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            }
            // Enter
            else if (new_state == CHARGE_MODE_WAIT_FOR_ZERO) {
                bool needs_menu_entry = !charge_mode_menu_active ||
                                        charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT;
                charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_ZERO;

                if (needs_menu_entry) {
                    exit_state = APP_STATE_ENTER_CHARGE_MODE_FROM_REST;
                    ButtonEncoderEvent_t button_event = OVERRIDE_FROM_REST;
                    if (encoder_event_queue != NULL) {
                        (void)xQueueSend(encoder_event_queue, &button_event, pdMS_TO_TICKS(250));
                    }
                }
            }
            else {
                charge_mode_config.charge_mode_state = new_state;
            }
        }
    }

    // Handle the special case
    float current_measurement = scale_get_current_measurement();
    char weight_string[16];
    charge_mode_format_json_weight(weight_string, sizeof(weight_string), current_measurement);

    char final_weight_string[16];
    charge_mode_format_json_weight(final_weight_string,
                                   sizeof(final_weight_string),
                                   last_final_weight_valid ? last_final_weight_gn : NAN);
    char coarse_stop_weight_string[16];
    charge_mode_format_json_weight(coarse_stop_weight_string,
                                   sizeof(coarse_stop_weight_string),
                                   live_coarse_stop_weight_gn);
    char after_coarse_weight_string[16];
    charge_mode_format_json_weight(after_coarse_weight_string,
                                   sizeof(after_coarse_weight_string),
                                   live_after_coarse_settle_weight_gn);
    char observed_tail_string[16];
    charge_mode_format_json_weight(observed_tail_string,
                                   sizeof(observed_tail_string),
                                   live_observed_coarse_tail_gn);
    char fine_stop_weight_string[16];
    charge_mode_format_json_weight(fine_stop_weight_string,
                                   sizeof(fine_stop_weight_string),
                                   live_fine_stop_weight_gn);
    char after_fine_weight_string[16];
    charge_mode_format_json_weight(after_fine_weight_string,
                                   sizeof(after_fine_weight_string),
                                   live_after_fine_settle_weight_gn);
    char observed_fine_tail_string[16];
    charge_mode_format_json_weight(observed_fine_tail_string,
                                   sizeof(observed_fine_tail_string),
                                   live_observed_fine_tail_gn);
    char post_finish_peak_string[16];
    charge_mode_format_json_weight(post_finish_peak_string,
                                   sizeof(post_finish_peak_string),
                                   live_post_finish_peak_weight_gn);
    char recovery_start_weight_string[16];
    charge_mode_format_json_weight(recovery_start_weight_string,
                                   sizeof(recovery_start_weight_string),
                                   live_recovery_start_weight_gn);
    char recovery_end_weight_string[16];
    charge_mode_format_json_weight(recovery_end_weight_string,
                                   sizeof(recovery_end_weight_string),
                                   live_recovery_end_weight_gn);

    // Format elapsed time
    if (charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_COMPLETE) {
        TickType_t now = xTaskGetTickCount();
        float elapsed_seconds = (float)((now - charge_start_tick) * portTICK_PERIOD_MS) / 1000.0f;
        snprintf(elapsed_time_buffer, sizeof(elapsed_time_buffer), "%.2f", elapsed_seconds);
    } else {
        snprintf(elapsed_time_buffer, sizeof(elapsed_time_buffer), "%.2f", last_charge_elapsed_seconds);
    }

    const char* response_phase = live_charge_phase;
    if (charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
        response_phase = "idle";
    }

    // Response
    snprintf(charge_mode_json_buffer, 
             sizeof(charge_mode_json_buffer),
             "%s"
             "{\"s0\":%0.3f,\"s1\":%s,\"s2\":%d,\"s3\":%lu,\"s4\":\"%s\",\"s5\":\"%s\",\"s6\":%s,"
             "\"s7\":%s,\"s8\":\"%s\",\"s9\":%0.3f,\"s10\":%0.3f,\"s11\":%0.3f,\"s12\":%0.3f,"
             "\"s13\":%0.3f,\"s14\":%0.3f,\"s15\":%lu,\"s16\":%0.4f,"
             "\"s17\":%s,\"s18\":%s,\"s19\":%s,\"s20\":%lu,"
             "\"s21\":%0.3f,\"s22\":%0.3f,\"s23\":%s,\"s24\":%s,\"s25\":%s,"
             "\"s26\":%s,\"s27\":%s,\"s28\":%s,\"s29\":%lu,\"s30\":%lu,\"s31\":%u,"
             "\"s32\":\"%s\",\"s33\":%0.3f,\"s34\":%0.3f,\"s35\":%0.3f,"
             "\"s36\":%0.3f,\"s37\":%0.3f,\"s38\":%0.3f}",
             http_json_header,
             charge_mode_config.target_charge_weight,
             weight_string,
             (int) charge_mode_config.charge_mode_state,
             charge_mode_config.charge_mode_event,
             profile_get_selected()->name,
             elapsed_time_buffer,
             final_weight_string,
             live_runtime_model_active ? "true" : "false",
             response_phase,
             live_final_target_gn,
             live_bulk_handoff_margin_gn,
             live_trim_stop_margin_gn,
             live_fine_window_gn,
             live_remaining_gn,
             live_stop_margin_gn,
             (unsigned long)live_salvage_cycles,
             charge_mode_acceptance_tolerance(),
              coarse_stop_weight_string,
              after_coarse_weight_string,
              observed_tail_string,
              (unsigned long)live_bulk_deadline_ms,
              live_coarse_command_rps,
              live_fine_command_rps,
              fine_stop_weight_string,
              after_fine_weight_string,
              observed_fine_tail_string,
              post_finish_peak_string,
              recovery_start_weight_string,
              recovery_end_weight_string,
              (unsigned long)live_recovery_motor_on_ms,
              (unsigned long)live_recovery_stall_count,
              (unsigned int)live_recovery_exit_reason,
              live_ai_decision,
              live_model_bulk_speed_rps,
              live_model_bulk_flow_gps,
              live_model_bulk_tail_gn,
              live_model_fine_speed_rps,
              live_model_fine_flow_gps,
              live_model_fine_tail_gn);

    // Clear events
    charge_mode_config.charge_mode_event = 0;

    size_t data_length = strlen(charge_mode_json_buffer);
    file->data = charge_mode_json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}
