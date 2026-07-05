/**
 * @file ai_tuning.c
 * @brief Adaptive flow characterization and model-based charge planning.
 */

#include "ai_tuning.h"
#include "charge_mode.h"
#include "flash_storage.h"
#include "eeprom.h"
#include "profile.h"
#include "motors.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "mini_12864_module.h"

extern QueueHandle_t encoder_event_queue;
extern charge_mode_config_t charge_mode_config;

static SemaphoreHandle_t g_ai_tuning_mutex = NULL;

static ai_tuning_session_t g_session;
static ai_tuning_config_t g_config;
static ai_tuning_config_eeprom_t g_config_eeprom;
static ai_tuning_history_t g_history;
static bool g_initialized = false;
static bool g_history_loaded = false;
static bool g_config_loaded = false;

static bool ai_tuning_lock(TickType_t timeout_ticks);
static void ai_tuning_unlock(void);
static void load_history_from_flash(void);
static void save_history_to_flash(void);
static void load_config_from_eeprom(void);
static bool save_config_to_eeprom(void);
static void ai_tuning_reset_session_unlocked(void);
static bool ai_tuning_is_active_unlocked(void);
static bool ai_tuning_is_complete_unlocked(void);
static ai_motor_mode_t ai_tuning_get_motor_mode_unlocked(void);
static uint8_t ai_tuning_get_progress_percent_unlocked(void);
static uint8_t ai_tuning_get_profile_index_for_pointer_unlocked(profile_t* profile);
static void ai_tuning_update_session_stats_unlocked(void);
static uint8_t ai_tuning_get_stage_sample_limit_unlocked(ai_tuning_state_t state);
static float ai_tuning_get_stage_budget_limit_unlocked(ai_tuning_state_t state);
static float ai_tuning_interpolate_stage_speed_unlocked(ai_tuning_state_t state, uint8_t sample_index);
static bool ai_tuning_is_fine_recovery_sample_unlocked(ai_tuning_state_t state, uint8_t sample_index);
static float ai_tuning_fine_recovery_speed_for_sample(uint8_t sample_index);
static float ai_tuning_fine_recovery_on_time_ms_for_sample(uint8_t sample_index);
static void ai_tuning_fit_stage_samples(const ai_flow_sample_t* samples, uint8_t count,
                                        float* slope, float* intercept, float* avg_tail);
static float ai_tuning_predict_flow_from_model(const ai_profile_model_t* model,
                                               ai_motor_mode_t motor_mode,
                                               float speed_rps);
static float ai_tuning_estimate_flow_unlocked(ai_tuning_state_t state, float speed_rps);
static float ai_tuning_estimate_on_time_ms_unlocked(ai_tuning_state_t state,
                                                    float speed_rps,
                                                    float target_weight);
static void ai_tuning_rebuild_stage_model_unlocked(ai_tuning_state_t state);
static void ai_tuning_choose_best_stage_sample(const ai_flow_sample_t* samples, uint8_t count,
                                               float tail_penalty,
                                               float* best_speed_rps,
                                               float* best_flow_gps);
static void ai_tuning_choose_bulk_stage_sample(const ai_flow_sample_t* samples, uint8_t count,
                                               float* best_speed_rps,
                                               float* best_flow_gps);
static void ai_tuning_choose_trim_stage_sample(const ai_flow_sample_t* samples, uint8_t count,
                                               float* trim_speed_rps,
                                               float* trim_flow_gps,
                                               float* trim_tail_gn);
static void ai_tuning_choose_recovery_stage_sample(const ai_flow_sample_t* samples, uint8_t count,
                                                   float* recovery_speed_rps,
                                                   float* recovery_flow_gps,
                                                   float* recovery_tail_gn);
static float ai_tuning_default_target_weight_for_profile_unlocked(uint8_t profile_idx);
static float ai_tuning_estimate_coarse_tail_guard(const ai_flow_sample_t* samples,
                                                  uint8_t count,
                                                  float average_tail_gn,
                                                  float target_weight);
static float ai_tuning_estimate_fine_tail_guard(const ai_flow_sample_t* samples,
                                                uint8_t count,
                                                float average_tail_gn,
                                                float target_weight);
static void ai_tuning_filter_runtime_observations_unlocked(void);
static void ai_tuning_reset_machine_calibration(ai_machine_calibration_t* cal);
static void ai_tuning_update_machine_calibration_unlocked(const ai_drop_telemetry_t* telemetry);
static void ai_tuning_sanitize_machine_calibration(ai_profile_model_t* model, float target_weight);
static float ai_tuning_machine_coarse_tail_guard(const ai_machine_calibration_t* cal,
                                                 float target_weight);
static float ai_tuning_machine_bulk_handoff_margin(const ai_profile_model_t* model,
                                                   float target_weight,
                                                   float desired_fine_window);
static void ai_tuning_sanitize_model_unlocked(ai_profile_model_t* model,
                                              float target_weight,
                                              bool preserve_enabled);
static void ai_tuning_update_finish_profile_unlocked(ai_profile_model_t* model, float target_weight);
static void ai_tuning_clamp_steering_unlocked(ai_profile_model_t* model);
static void ai_tuning_finalize_working_model_unlocked(void);
static void ai_tuning_refresh_plan_unlocked(void);

_Static_assert(sizeof(ai_tuning_history_t) <= FLASH_STORAGE_ML_HISTORY_SIZE,
               "AI tuning history no longer fits in reserved flash storage");

static float ai_clampf(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static bool ai_tuning_is_reasonable_weight(float value) {
    return isfinite(value) && fabsf(value) < 10000.0f;
}

static float ai_tuning_machine_coarse_tail_guard(const ai_machine_calibration_t* cal,
                                                 float target_weight) {
    if (cal == NULL || !cal->valid) {
        return 0.0f;
    }

    const float avg_tail = isfinite(cal->coarse_tail_avg_gn)
        ? fmaxf(cal->coarse_tail_avg_gn, 0.0f)
        : 0.0f;
    const float p95_tail = isfinite(cal->coarse_tail_p95_gn)
        ? fmaxf(cal->coarse_tail_p95_gn, avg_tail)
        : avg_tail;
    const float uncertainty = isfinite(cal->coarse_uncertainty_gn)
        ? fmaxf(cal->coarse_uncertainty_gn, 0.0f)
        : 0.0f;

    float guard = fmaxf(p95_tail, avg_tail + uncertainty * 0.65f);
    const float cap = fmaxf(20.0f, target_weight * 0.55f);
    return ai_clampf(guard, 0.0f, cap);
}

static float ai_tuning_machine_bulk_handoff_margin(const ai_profile_model_t* model,
                                                   float target_weight,
                                                   float desired_fine_window) {
    if (model == NULL || !model->machine.valid) {
        return 0.0f;
    }

    const ai_machine_calibration_t* cal = &model->machine;
    const float tail_guard = ai_tuning_machine_coarse_tail_guard(cal, target_weight);
    if (tail_guard <= 0.0f) {
        return 0.0f;
    }

    const float uncertainty = isfinite(cal->coarse_uncertainty_gn)
        ? fmaxf(cal->coarse_uncertainty_gn, 0.0f)
        : 0.0f;
    const float flow_gps = isfinite(cal->coarse_open_loop_flow_gps)
        ? fmaxf(cal->coarse_open_loop_flow_gps, 0.0f)
        : 0.0f;
    const float latency_s =
        (fmaxf(cal->scale_sample_period_ms, 0.0f) +
         fmaxf(cal->coarse_first_response_ms, 0.0f)) / 1000.0f;

    const float uncertainty_margin =
        fminf(fmaxf(2.25f, target_weight * 0.070f), uncertainty * 0.70f);
    const float latency_margin =
        fminf(fmaxf(0.85f, target_weight * 0.040f), flow_gps * latency_s * 0.85f);
    const float margin = desired_fine_window +
                         tail_guard +
                         uncertainty_margin +
                         latency_margin;

    const float max_margin =
        desired_fine_window + fmaxf(18.0f, target_weight * 0.55f);
    return ai_clampf(margin,
                     desired_fine_window + 2.25f,
                     max_margin);
}

const char* ai_tuning_steering_action_to_string(ai_steering_action_t action) {
    switch (action) {
        case AI_STEERING_FASTER:              return "faster";
        case AI_STEERING_SAFER:               return "safer";
        case AI_STEERING_FINE_FINISH_FASTER:  return "fine_finish_faster";
        case AI_STEERING_BULK_CLOSER:         return "bulk_closer";
        case AI_STEERING_UNDO_LAST:           return "undo_last";
        default:                              return "unknown";
    }
}

const char* ai_tuning_fine_tube_profile_to_string(ai_fine_tube_profile_t profile) {
    switch (profile) {
        case AI_FINE_TUBE_PROFILE_LOW_FLOW_LOW_TAIL: return "low-flow/low-tail";
        case AI_FINE_TUBE_PROFILE_BALANCED:          return "balanced";
        case AI_FINE_TUBE_PROFILE_HIGH_TAIL:         return "high-tail";
        default:                                     return "unknown";
    }
}

static bool ai_tuning_is_reasonable_drop(const ai_drop_telemetry_t* telemetry) {
    if (telemetry == NULL ||
        !ai_tuning_is_reasonable_weight(telemetry->start_weight) ||
        !ai_tuning_is_reasonable_weight(telemetry->stop_weight) ||
        !ai_tuning_is_reasonable_weight(telemetry->final_weight) ||
        !isfinite(telemetry->target_weight) ||
        !isfinite(telemetry->speed_rps) ||
        !isfinite(telemetry->motor_on_time_ms) ||
        telemetry->speed_rps <= 0.0f ||
        telemetry->motor_on_time_ms <= 0.0f) {
        return false;
    }

    float delivered = telemetry->final_weight - telemetry->start_weight;
    float tail = telemetry->final_weight - telemetry->stop_weight;
    float max_reasonable_drop = fmaxf(80.0f, fmaxf(telemetry->target_weight, 1.0f) * 5.0f);

    return delivered > -0.25f &&
           delivered <= max_reasonable_drop &&
           tail > -0.25f &&
           tail <= max_reasonable_drop;
}

static bool ai_tuning_has_valid_stop_snapshot(const ai_drop_telemetry_t* telemetry) {
    if (telemetry == NULL ||
        !ai_tuning_is_reasonable_weight(telemetry->start_weight) ||
        !ai_tuning_is_reasonable_weight(telemetry->stop_weight) ||
        !ai_tuning_is_reasonable_weight(telemetry->final_weight)) {
        return false;
    }

    float delivered = telemetry->final_weight - telemetry->start_weight;
    float stopped = telemetry->stop_weight - telemetry->start_weight;
    if (delivered <= g_config.noise_margin * 1.5f) {
        return false;
    }

    float minimum_stop_progress = fmaxf(0.06f, fminf(0.35f, delivered * 0.12f));
    return stopped >= minimum_stop_progress &&
           telemetry->stop_weight <= telemetry->final_weight + fmaxf(0.10f, g_config.noise_margin * 2.0f);
}

static float ai_tuning_min_fine_window_for_model(const ai_profile_model_t* model) {
    if (model == NULL) {
        return 0.55f;
    }

    float noise = fmaxf(g_config.noise_margin, 0.01f);
    float fine_tail_window = fmaxf(0.0f, model->fine_tail_gn) * 1.25f + noise * 1.5f;
    float trim_tail_window = fmaxf(0.0f, model->coarse_trim_tail_gn) * 0.08f + noise;
    return fmaxf(0.55f, fmaxf(fine_tail_window, trim_tail_window));
}

void ai_tuning_init(void) {
    if (g_initialized) {
        return;
    }

    if (g_ai_tuning_mutex == NULL) {
        g_ai_tuning_mutex = xSemaphoreCreateRecursiveMutex();
    }

    g_config.coarse_budget_gn = 120.0f;
    g_config.fine_budget_gn = 36.0f;
    g_config.coarse_sample_count = AI_TUNING_STAGE_SAMPLE_COUNT;
    g_config.fine_sample_count = AI_TUNING_STAGE_SAMPLE_COUNT;
    g_config.coarse_sample_target_gn = 8.0f;
    g_config.fine_sample_target_gn = 1.75f;
    g_config.noise_margin = 0.05f;
    g_config.time_cost_weight = 1.0f;
    g_config.error_cost_weight = 8.0f;

    memset(&g_session, 0, sizeof(g_session));
    g_session.state = AI_TUNING_IDLE;

    load_config_from_eeprom();
    load_history_from_flash();

    eeprom_register_handler(save_config_to_eeprom);
    g_initialized = true;
}

ai_tuning_config_t* ai_tuning_get_config(void) {
    return &g_config;
}

bool ai_tuning_start(profile_t* profile, float target_weight) {
    if (!g_initialized) {
        ai_tuning_init();
    }

    if (profile == NULL || target_weight <= 0.0f) {
        return false;
    }

    if (!ai_tuning_lock(pdMS_TO_TICKS(1000))) {
        return false;
    }

    load_history_from_flash();
    ai_tuning_reset_session_unlocked();

    g_session.state = AI_TUNING_CHARACTERIZING_COARSE;
    g_session.target_profile = profile;
    g_session.target_profile_idx = ai_tuning_get_profile_index_for_pointer_unlocked(profile);
    g_session.requested_target_weight = target_weight;
    g_session.total_samples_planned = (uint8_t)(g_config.coarse_sample_count + g_config.fine_sample_count);
    g_session.stage_budget_limit_gn = g_config.coarse_budget_gn;
    memset(&g_session.working_model, 0, sizeof(g_session.working_model));
    snprintf(g_session.status_message, sizeof(g_session.status_message),
             "Preparing coarse characterization");

    ai_tuning_refresh_plan_unlocked();
    ai_tuning_unlock();
    return true;
}

bool ai_tuning_start_machine_calibration(profile_t* profile, float target_weight) {
    if (!g_initialized) {
        ai_tuning_init();
    }

    if (profile == NULL || target_weight <= 0.0f) {
        return false;
    }

    if (!ai_tuning_lock(pdMS_TO_TICKS(1000))) {
        return false;
    }

    load_history_from_flash();
    uint8_t profile_idx = ai_tuning_get_profile_index_for_pointer_unlocked(profile);
    const ai_profile_model_t* saved_model = &g_history.models[profile_idx];
    if (!saved_model->valid || !saved_model->enabled) {
        ai_tuning_unlock();
        return false;
    }

    ai_tuning_reset_session_unlocked();
    g_session.state = AI_TUNING_CALIBRATING_COARSE;
    g_session.target_profile = profile;
    g_session.target_profile_idx = profile_idx;
    g_session.requested_target_weight = target_weight;
    g_session.total_samples_planned =
        (uint8_t)(AI_MACHINE_CAL_COARSE_SAMPLE_COUNT + AI_MACHINE_CAL_FINE_SAMPLE_COUNT);
    g_session.stage_budget_limit_gn = ai_tuning_get_stage_budget_limit_unlocked(g_session.state);
    memcpy(&g_session.working_model, saved_model, sizeof(g_session.working_model));
    ai_tuning_reset_machine_calibration(&g_session.working_model.machine);
    snprintf(g_session.status_message, sizeof(g_session.status_message),
             "Preparing OpenTrickler machine calibration");

    ai_tuning_refresh_plan_unlocked();
    ai_tuning_unlock();
    return true;
}

bool ai_tuning_record_drop(const ai_drop_telemetry_t* telemetry) {
    if (!ai_tuning_is_reasonable_drop(telemetry)) {
        return false;
    }

    if (!ai_tuning_lock(pdMS_TO_TICKS(100))) {
        return false;
    }

    if (!ai_tuning_is_active_unlocked()) {
        ai_tuning_unlock();
        return false;
    }

    const uint8_t write_idx = g_session.drop_write_idx;
    memcpy(&g_session.drops[write_idx], telemetry, sizeof(ai_drop_telemetry_t));
    g_session.drop_write_idx = (uint8_t)((g_session.drop_write_idx + 1) % AI_TUNING_DROP_BUF_SIZE);
    if (g_session.drops_completed < UINT8_MAX) {
        g_session.drops_completed++;
    }

    ai_flow_sample_t sample = {0};
    sample.speed_rps = telemetry->speed_rps;
    sample.motor_on_time_ms = telemetry->motor_on_time_ms;
    sample.delivered_weight = fmaxf(0.0f, telemetry->final_weight - telemetry->start_weight);
    sample.tail_weight = ai_tuning_has_valid_stop_snapshot(telemetry)
        ? fmaxf(0.0f, telemetry->final_weight - telemetry->stop_weight)
        : 0.0f;
    sample.flow_gps = (telemetry->motor_on_time_ms > 0.0f)
        ? (sample.delivered_weight / (telemetry->motor_on_time_ms / 1000.0f))
        : 0.0f;

    if (g_session.state == AI_TUNING_CALIBRATING_COARSE ||
        g_session.state == AI_TUNING_CALIBRATING_FINE) {
        ai_tuning_update_machine_calibration_unlocked(telemetry);
        g_session.stage_budget_used_gn += sample.delivered_weight;

        uint8_t sample_limit = ai_tuning_get_stage_sample_limit_unlocked(g_session.state);
        uint8_t samples_in_stage = (uint8_t)(g_session.stage_sample_index + 1u);
        bool sample_limit_reached = samples_in_stage >= sample_limit;
        bool budget_guard_reached =
            samples_in_stage >= AI_MACHINE_CAL_MIN_VALID_SAMPLE_COUNT &&
            g_session.stage_budget_used_gn >= ai_tuning_get_stage_budget_limit_unlocked(g_session.state);
        if (sample_limit_reached || budget_guard_reached) {
            if (g_session.state == AI_TUNING_CALIBRATING_COARSE) {
                g_session.state = AI_TUNING_CALIBRATING_FINE;
                g_session.stage_sample_index = 0;
                g_session.stage_budget_used_gn = 0.0f;
                g_session.stage_budget_limit_gn = ai_tuning_get_stage_budget_limit_unlocked(g_session.state);
                snprintf(g_session.status_message, sizeof(g_session.status_message),
                         "Coarse machine calibration captured %u sample%s. Starting fine timing samples.",
                         samples_in_stage,
                         samples_in_stage == 1u ? "" : "s");
            }
            else {
                ai_tuning_sanitize_model_unlocked(&g_session.working_model,
                                                  fmaxf(g_session.requested_target_weight, 40.0f),
                                                  true);
                if (g_session.working_model.machine.valid) {
                    g_session.state = AI_TUNING_READY_TO_SAVE;
                    snprintf(g_session.status_message, sizeof(g_session.status_message),
                             "Machine calibration complete: %u coarse / %u fine samples. Save model.",
                             g_session.working_model.machine.coarse_sample_count,
                             g_session.working_model.machine.fine_sample_count);
                }
                else {
                    g_session.state = AI_TUNING_ERROR;
                    snprintf(g_session.error_message, sizeof(g_session.error_message),
                             "Machine calibration needs at least %u coarse and %u fine samples; captured %u / %u.",
                             AI_MACHINE_CAL_MIN_VALID_SAMPLE_COUNT,
                             AI_MACHINE_CAL_MIN_VALID_SAMPLE_COUNT,
                             g_session.working_model.machine.coarse_sample_count,
                             g_session.working_model.machine.fine_sample_count);
                    snprintf(g_session.status_message, sizeof(g_session.status_message),
                             "%s", g_session.error_message);
                }
            }
        }
        else {
            g_session.stage_sample_index++;
        }
    }
    else if (g_session.state == AI_TUNING_CHARACTERIZING_COARSE) {
        uint8_t idx = g_session.working_model.coarse_sample_count;
        if (idx < AI_TUNING_STAGE_SAMPLE_COUNT) {
            g_session.working_model.coarse_samples[idx] = sample;
            g_session.working_model.coarse_sample_count++;
            g_session.stage_budget_used_gn += sample.delivered_weight;
        }
        ai_tuning_rebuild_stage_model_unlocked(AI_TUNING_CHARACTERIZING_COARSE);

        if (g_session.working_model.coarse_sample_count >= g_config.coarse_sample_count ||
            g_session.stage_budget_used_gn >= g_config.coarse_budget_gn) {
            g_session.state = AI_TUNING_CHARACTERIZING_FINE;
            g_session.stage_sample_index = 0;
            g_session.stage_budget_used_gn = 0.0f;
            g_session.stage_budget_limit_gn = g_config.fine_budget_gn;
            snprintf(g_session.status_message, sizeof(g_session.status_message),
                     "Coarse characterization done. Taring for fine samples.");
        } else {
            g_session.stage_sample_index++;
        }
    }
    else if (g_session.state == AI_TUNING_CHARACTERIZING_FINE) {
        uint8_t idx = g_session.working_model.fine_sample_count;
        if (idx < AI_TUNING_STAGE_SAMPLE_COUNT) {
            g_session.working_model.fine_samples[idx] = sample;
            g_session.working_model.fine_sample_count++;
            g_session.stage_budget_used_gn += sample.delivered_weight;
        }
        if ((telemetry->sample_number <= AI_TUNING_FINE_RECOVERY_SAMPLE_COUNT ||
             telemetry->speed_rps <= 0.42f) &&
            g_session.working_model.fine_recovery_sample_count < AI_TUNING_FINE_RECOVERY_SAMPLE_COUNT) {
            uint8_t recovery_idx = g_session.working_model.fine_recovery_sample_count;
            g_session.working_model.fine_recovery_samples[recovery_idx] = sample;
            g_session.working_model.fine_recovery_sample_count++;
        }
        ai_tuning_rebuild_stage_model_unlocked(AI_TUNING_CHARACTERIZING_FINE);

        if (g_session.working_model.fine_sample_count >= g_config.fine_sample_count ||
            g_session.stage_budget_used_gn >= g_config.fine_budget_gn) {
            ai_tuning_finalize_working_model_unlocked();
            if (g_session.working_model.valid) {
                g_session.state = AI_TUNING_READY_TO_SAVE;
                snprintf(g_session.status_message, sizeof(g_session.status_message),
                         "Characterization complete. Empty the cup and save this model.");
            } else {
                g_session.state = AI_TUNING_ERROR;
                snprintf(g_session.error_message, sizeof(g_session.error_message),
                         "Need at least one valid coarse and fine sample");
            }
        } else {
            g_session.stage_sample_index++;
        }
    }

    ai_tuning_update_session_stats_unlocked();

    if (ai_tuning_is_active_unlocked()) {
        ai_tuning_refresh_plan_unlocked();
    }

    ai_tuning_unlock();
    return true;
}

bool ai_tuning_get_next_params(float* coarse_a, float* coarse_b, float* fine_a, float* fine_b) {
    if (coarse_a == NULL || coarse_b == NULL || fine_a == NULL || fine_b == NULL) {
        return false;
    }

    if (!ai_tuning_lock(pdMS_TO_TICKS(50))) {
        return false;
    }

    const ai_profile_model_t* model = &g_session.working_model;
    bool have_model = model->valid ||
                      model->coarse_sample_count > 0 ||
                      model->fine_sample_count > 0;

    if (have_model) {
        *coarse_a = model->coarse_best_speed_rps;
        *coarse_b = model->coarse_tail_gn;
        *fine_a = model->fine_best_speed_rps;
        *fine_b = model->fine_tail_gn;
    }

    ai_tuning_unlock();
    return have_model;
}

bool ai_tuning_is_complete(void) {
    if (!ai_tuning_lock(pdMS_TO_TICKS(50))) {
        return false;
    }

    bool result = ai_tuning_is_complete_unlocked();
    ai_tuning_unlock();
    return result;
}

bool ai_tuning_get_session_copy(ai_tuning_session_t* out) {
    if (out == NULL) {
        return false;
    }

    if (!ai_tuning_lock(pdMS_TO_TICKS(50))) {
        return false;
    }

    memcpy(out, &g_session, sizeof(ai_tuning_session_t));
    ai_tuning_unlock();
    return true;
}

bool ai_tuning_get_history_copy(ai_tuning_history_t* out) {
    if (out == NULL) {
        return false;
    }

    if (!ai_tuning_lock(pdMS_TO_TICKS(50))) {
        return false;
    }

    load_history_from_flash();
    memcpy(out, &g_history, sizeof(ai_tuning_history_t));
    ai_tuning_unlock();
    return true;
}

bool ai_tuning_get_recommended_params(float* coarse_a, float* coarse_b, float* fine_a, float* fine_b) {
    return ai_tuning_get_next_params(coarse_a, coarse_b, fine_a, fine_b);
}

bool ai_tuning_apply_params(void) {
    if (!ai_tuning_lock(pdMS_TO_TICKS(100))) {
        return false;
    }

    if (!ai_tuning_is_complete_unlocked() ||
        !g_session.working_model.valid ||
        g_session.target_profile_idx >= MAX_PROFILE_CNT) {
        ai_tuning_unlock();
        return false;
    }

    load_history_from_flash();
    ai_tuning_sanitize_model_unlocked(&g_session.working_model,
                                      fmaxf(g_session.requested_target_weight, 40.0f),
                                      false);
    if (!g_session.working_model.valid) {
        ai_tuning_unlock();
        return false;
    }
    g_session.working_model.enabled = true;
    g_history.models[g_session.target_profile_idx] = g_session.working_model;
    save_history_to_flash();

    ai_tuning_reset_session_unlocked();
    ai_tuning_unlock();
    return true;
}

bool ai_tuning_cancel(void) {
    if (!ai_tuning_lock(pdMS_TO_TICKS(100))) {
        return false;
    }

    bool was_running = (g_session.state != AI_TUNING_IDLE);
    ai_tuning_reset_session_unlocked();
    ai_tuning_unlock();

    if (was_running) {
        charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
        ButtonEncoderEvent_t button_event = BUTTON_RST_PRESSED;
        if (encoder_event_queue != NULL &&
            xQueueSend(encoder_event_queue, &button_event, pdMS_TO_TICKS(50)) != pdTRUE) {
            return false;
        }
    }

    return true;
}

bool ai_tuning_is_active(void) {
    if (!ai_tuning_lock(pdMS_TO_TICKS(50))) {
        return false;
    }

    bool result = ai_tuning_is_active_unlocked();
    ai_tuning_unlock();
    return result;
}

ai_motor_mode_t ai_tuning_get_motor_mode(void) {
    if (!ai_tuning_lock(pdMS_TO_TICKS(50))) {
        return AI_MOTOR_MODE_NORMAL;
    }

    ai_motor_mode_t result = ai_tuning_get_motor_mode_unlocked();
    ai_tuning_unlock();
    return result;
}

uint8_t ai_tuning_get_progress_percent(void) {
    if (!ai_tuning_lock(pdMS_TO_TICKS(50))) {
        return 0;
    }

    uint8_t result = ai_tuning_get_progress_percent_unlocked();
    ai_tuning_unlock();
    return result;
}

bool ai_tuning_get_active_plan(ai_tuning_plan_t* out) {
    if (out == NULL) {
        return false;
    }

    if (!ai_tuning_lock(pdMS_TO_TICKS(50))) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    if (!ai_tuning_is_active_unlocked()) {
        ai_tuning_unlock();
        return false;
    }

    out->valid = true;
    out->motor_mode = ai_tuning_get_motor_mode_unlocked();
    out->speed_rps = g_session.current_speed_rps;
    out->motor_on_time_ms = g_session.current_motor_on_time_ms;
    out->target_weight = g_session.current_target_weight;
    out->stage_budget_used_gn = g_session.stage_budget_used_gn;
    out->stage_budget_limit_gn = g_session.stage_budget_limit_gn;
    out->sample_index = (uint8_t)(g_session.stage_sample_index + 1);
    out->total_samples = ai_tuning_get_stage_sample_limit_unlocked(g_session.state);
    snprintf(out->description, sizeof(out->description), "%s", g_session.status_message);
    ai_tuning_unlock();
    return true;
}

bool ai_tuning_get_profile_model_copy(uint8_t profile_idx, ai_profile_model_t* out) {
    if (out == NULL || profile_idx >= MAX_PROFILE_CNT) {
        return false;
    }

    if (!ai_tuning_lock(pdMS_TO_TICKS(50))) {
        return false;
    }

    load_history_from_flash();
    memcpy(out, &g_history.models[profile_idx], sizeof(ai_profile_model_t));
    ai_tuning_unlock();
    return true;
}

bool ai_tuning_get_enabled_model_copy(uint8_t profile_idx, ai_profile_model_t* out) {
    if (!ai_tuning_get_profile_model_copy(profile_idx, out)) {
        return false;
    }
    return out->valid && out->enabled;
}

bool ai_tuning_apply_steering(uint8_t profile_idx, ai_steering_action_t action) {
    if (profile_idx >= MAX_PROFILE_CNT) {
        return false;
    }

    if (!ai_tuning_lock(pdMS_TO_TICKS(100))) {
        return false;
    }

    load_history_from_flash();
    ai_profile_model_t* model = &g_history.models[profile_idx];
    if (!model->valid || !model->enabled) {
        ai_tuning_unlock();
        return false;
    }

    ai_tuning_clamp_steering_unlocked(model);
    if (action == AI_STEERING_UNDO_LAST) {
        if (!model->steering_undo_available) {
            ai_tuning_unlock();
            return false;
        }
        model->steering_bulk_bias_gn = model->steering_last_bulk_bias_gn;
        model->steering_fine_bias_gn = model->steering_last_fine_bias_gn;
        model->steering_recovery_speed_scale = model->steering_last_recovery_speed_scale;
        model->steering_undo_available = 0;
    }
    else {
        model->steering_last_bulk_bias_gn = model->steering_bulk_bias_gn;
        model->steering_last_fine_bias_gn = model->steering_fine_bias_gn;
        model->steering_last_recovery_speed_scale = model->steering_recovery_speed_scale;
        model->steering_undo_available = 1;

        switch (action) {
            case AI_STEERING_FASTER:
                model->steering_bulk_bias_gn -= 0.12f;
                model->steering_fine_bias_gn -= 0.010f;
                model->steering_recovery_speed_scale += 0.06f;
                break;
            case AI_STEERING_SAFER:
                model->steering_bulk_bias_gn += 0.18f;
                model->steering_fine_bias_gn += 0.020f;
                model->steering_recovery_speed_scale -= 0.04f;
                break;
            case AI_STEERING_FINE_FINISH_FASTER:
                model->steering_fine_bias_gn -= 0.015f;
                model->steering_recovery_speed_scale += 0.10f;
                break;
            case AI_STEERING_BULK_CLOSER:
                model->steering_bulk_bias_gn -= 0.18f;
                break;
            default:
                ai_tuning_unlock();
                return false;
        }

        if (model->steering_count < UINT16_MAX) {
            model->steering_count++;
        }
    }

    ai_tuning_clamp_steering_unlocked(model);
    ai_tuning_update_finish_profile_unlocked(model,
                                             ai_tuning_default_target_weight_for_profile_unlocked(profile_idx));
    save_history_to_flash();
    ai_tuning_unlock();
    return true;
}

void ai_tuning_record_charge(uint8_t profile_idx, float target_weight,
                             float final_error_gn, float total_time_ms,
                             float coarse_stop_weight_gn,
                             float after_coarse_settle_gn,
                             float observed_coarse_tail_gn,
                             float fine_stop_weight_gn,
                             float after_fine_settle_gn,
                             float observed_fine_tail_gn,
                             float post_finish_peak_weight_gn,
                             float recovery_start_weight_gn,
                             float recovery_end_weight_gn,
                             float recovery_motor_on_ms,
                             uint16_t recovery_stall_count,
                             uint8_t recovery_exit_reason) {
    if (profile_idx >= MAX_PROFILE_CNT) {
        return;
    }

    if (!isfinite(target_weight) ||
        !isfinite(final_error_gn) ||
        !isfinite(total_time_ms) ||
        target_weight <= 0.0f ||
        total_time_ms < 0.0f ||
        fabsf(final_error_gn) > fmaxf(35.0f, target_weight * 1.25f)) {
        return;
    }

    if (!ai_tuning_lock(pdMS_TO_TICKS(100))) {
        return;
    }

    load_history_from_flash();

    const float controller_failure_band = fmaxf(3.0f, target_weight * 0.08f);
    bool controller_failure = fabsf(final_error_gn) > controller_failure_band;
    bool history_changed = false;

    ai_runtime_observation_t obs = {
        .target_weight = target_weight,
        .final_error_gn = final_error_gn,
        .total_time_ms = total_time_ms,
        .coarse_stop_weight_gn = coarse_stop_weight_gn,
        .after_coarse_settle_gn = after_coarse_settle_gn,
        .observed_coarse_tail_gn = observed_coarse_tail_gn,
        .fine_stop_weight_gn = fine_stop_weight_gn,
        .after_fine_settle_gn = after_fine_settle_gn,
        .observed_fine_tail_gn = observed_fine_tail_gn,
        .post_finish_peak_weight_gn = post_finish_peak_weight_gn,
        .recovery_start_weight_gn = recovery_start_weight_gn,
        .recovery_end_weight_gn = recovery_end_weight_gn,
        .recovery_motor_on_ms = recovery_motor_on_ms,
        .recovery_stall_count = recovery_stall_count,
        .recovery_exit_reason = recovery_exit_reason,
        .profile_idx = profile_idx,
    };

    g_history.observations[g_history.observation_next_idx] = obs;
    g_history.observation_next_idx =
        (uint8_t)((g_history.observation_next_idx + 1) % AI_RUNTIME_OBSERVATION_COUNT);
    if (g_history.observation_count < AI_RUNTIME_OBSERVATION_COUNT) {
        g_history.observation_count++;
    }
    history_changed = true;

    ai_profile_model_t* model = &g_history.models[profile_idx];
    if (model->valid && model->enabled) {
        float positive_bias_limit = fmaxf(0.18f, fminf(0.30f, target_weight * 0.006f));
        float negative_bias_limit = 0.08f;
        const float acceptable_band = 0.0205f;
        float min_fine_window = ai_tuning_min_fine_window_for_model(model);
        float max_fine_window = fmaxf(1.20f, fminf(1.80f, target_weight * 0.05f));
        const float good_time_ms = 9500.0f;
        const float slow_time_ms = 15000.0f;
        const float very_slow_time_ms = 22000.0f;
        bool have_coarse_feedback =
            isfinite(coarse_stop_weight_gn) &&
            isfinite(after_coarse_settle_gn) &&
            isfinite(observed_coarse_tail_gn) &&
            coarse_stop_weight_gn > fmaxf(0.20f, target_weight * 0.03f) &&
            after_coarse_settle_gn >= -0.25f &&
            after_coarse_settle_gn >= coarse_stop_weight_gn - 0.10f &&
            observed_coarse_tail_gn >= 0.0f &&
            observed_coarse_tail_gn <= fmaxf(25.0f, target_weight * 0.75f);
        const float final_weight_gn = target_weight + final_error_gn;
        const bool recovery_motor_was_used =
            isfinite(recovery_motor_on_ms) && recovery_motor_on_ms > 50.0f;
        float production_fine_tail = observed_fine_tail_gn;
        if (!recovery_motor_was_used &&
            isfinite(fine_stop_weight_gn) &&
            isfinite(final_weight_gn) &&
            fine_stop_weight_gn > target_weight * 0.55f &&
            fine_stop_weight_gn < target_weight + 1.0f) {
            production_fine_tail = fmaxf(production_fine_tail,
                                         final_weight_gn - fine_stop_weight_gn);
        }
        if (isfinite(fine_stop_weight_gn) &&
            isfinite(after_fine_settle_gn) &&
            fine_stop_weight_gn > target_weight * 0.55f &&
            after_fine_settle_gn < target_weight + 1.0f) {
            production_fine_tail = fmaxf(production_fine_tail,
                                         after_fine_settle_gn - fine_stop_weight_gn);
        }
        const float fine_tail_cap = fmaxf(0.75f, fminf(2.00f, target_weight * 0.05f));
        bool have_fine_feedback =
            isfinite(fine_stop_weight_gn) &&
            isfinite(production_fine_tail) &&
            fine_stop_weight_gn > target_weight * 0.55f &&
            fine_stop_weight_gn < target_weight + 1.0f &&
            production_fine_tail >= 0.0f &&
            production_fine_tail <= fine_tail_cap;
        bool have_recovery_feedback =
            isfinite(recovery_start_weight_gn) &&
            isfinite(recovery_end_weight_gn) &&
            isfinite(recovery_motor_on_ms) &&
            recovery_start_weight_gn > target_weight * 0.55f &&
            recovery_end_weight_gn > target_weight * 0.55f &&
            recovery_motor_on_ms >= 0.0f;
        bool recovery_bottleneck =
            have_recovery_feedback &&
            (recovery_stall_count > 0 ||
             recovery_motor_on_ms > 5000.0f ||
             (final_error_gn < -acceptable_band && final_error_gn > -0.14f));
        float worst_final_error_gn = final_error_gn;
        if (isfinite(post_finish_peak_weight_gn) &&
            post_finish_peak_weight_gn > target_weight * 0.50f &&
            post_finish_peak_weight_gn < target_weight + fmaxf(15.0f, target_weight)) {
            worst_final_error_gn = fmaxf(worst_final_error_gn,
                                         post_finish_peak_weight_gn - target_weight);
        }
        const float desired_runtime_fine_window =
            fmaxf(0.85f, fminf(1.10f, target_weight * 0.025f));
        const bool coarse_phase_blame =
            have_coarse_feedback &&
            after_coarse_settle_gn >
                target_weight - desired_runtime_fine_window + acceptable_band;
        const bool recovery_phase_blame =
            have_recovery_feedback &&
            recovery_motor_was_used;

        const float runtime_disable_overcharge_band =
            fmaxf(0.28f, target_weight * 0.0075f);
        if (worst_final_error_gn > runtime_disable_overcharge_band) {
            // A serious overcharge is a lost charge. Keep valid telemetry, but fail the
            // next plan closed by immediately raising phase-specific safety margins.
            const float severe_overcharge =
                fminf(worst_final_error_gn, fmaxf(12.0f, target_weight * 0.45f));
            model->runtime_bias_gn = positive_bias_limit;
            model->recommended_fine_window_gn = max_fine_window;
            if (have_coarse_feedback && coarse_phase_blame) {
                const float tail_cap = fmaxf(20.0f, target_weight * 0.55f);
                const float observed_tail = ai_clampf(observed_coarse_tail_gn,
                                                      0.0f,
                                                      tail_cap);
                model->coarse_tail_gn = fmaxf(model->coarse_tail_gn,
                                              observed_tail * 0.92f);
                model->coarse_trim_tail_gn = fmaxf(model->coarse_trim_tail_gn,
                                                   observed_tail * 0.72f);
                if (model->machine.valid) {
                    model->machine.coarse_tail_avg_gn =
                        fmaxf(model->machine.coarse_tail_avg_gn,
                              observed_tail * 0.78f);
                    model->machine.coarse_tail_p95_gn =
                        fmaxf(model->machine.coarse_tail_p95_gn, observed_tail);
                    model->machine.coarse_uncertainty_gn =
                        fmaxf(model->machine.coarse_uncertainty_gn,
                              fminf(5.0f, severe_overcharge * 0.45f));
                }
            }
            if (model->machine.valid) {
                const float desired_fine_window =
                    fmaxf(0.85f, fminf(1.10f, target_weight * 0.025f));
                const float measured_margin =
                    ai_tuning_machine_bulk_handoff_margin(model,
                                                          target_weight,
                                                          desired_fine_window);
                const float emergency_margin =
                    desired_fine_window +
                    fmaxf(0.0f, observed_coarse_tail_gn) +
                    fminf(fmaxf(2.0f, severe_overcharge * 0.80f),
                          fmaxf(8.0f, target_weight * 0.22f));
                model->machine.recommended_bulk_handoff_gn =
                    fmaxf(model->machine.recommended_bulk_handoff_gn,
                          fmaxf(measured_margin, emergency_margin));
                model->machine.recommended_trim_stop_gn =
                    fmaxf(model->machine.recommended_trim_stop_gn,
                          fmaxf(2.5f, target_weight * 0.08f));
            }
            ai_tuning_sanitize_model_unlocked(model, target_weight, true);
            history_changed = true;
            if (history_changed) {
                save_history_to_flash();
            }
            ai_tuning_unlock();
            return;
        }

        if (fabsf(final_error_gn) > controller_failure_band) {
            // Skip runtime adaptation on obviously broken throws so a controller regression
            // does not teach the saved model the wrong behaviour.
        }
        else if (final_error_gn > acceptable_band) {
            float overcharge = fminf(final_error_gn, 0.60f);
            if (!recovery_phase_blame &&
                !have_fine_feedback &&
                coarse_phase_blame &&
                     after_coarse_settle_gn > target_weight + acceptable_band) {
                model->runtime_bias_gn += fmaxf(0.015f, overcharge * 0.16f);
                if (model->machine.valid) {
                    model->machine.recommended_bulk_handoff_gn +=
                        fmaxf(0.04f, overcharge * 0.35f);
                    model->machine.recommended_trim_stop_gn +=
                        fmaxf(0.02f, overcharge * 0.14f);
                }
            }
        }
        else if (final_error_gn < -acceptable_band) {
            float undercharge = fminf(-final_error_gn, 0.60f);
            model->runtime_bias_gn *= 0.94f;
            if (!recovery_bottleneck && undercharge > 0.18f && total_time_ms > slow_time_ms) {
                model->recommended_fine_window_gn -= fminf(0.018f, undercharge * 0.025f);
            }
        }
        else if (total_time_ms > 0.0f && fabsf(final_error_gn) <= acceptable_band) {
            if (fabsf(final_error_gn) <= acceptable_band * 0.50f) {
                model->runtime_bias_gn *= (total_time_ms > slow_time_ms) ? 0.90f : 0.95f;
            }
            else if (final_error_gn <= 0.0f) {
                model->runtime_bias_gn *= 0.93f;
            }
            if (recovery_bottleneck) {
                // Slow near-target recovery is a micro-flow problem, not a coarse/fine handoff problem.
            }
            else if (total_time_ms > very_slow_time_ms) {
                model->recommended_fine_window_gn -= 0.09f;
            }
            else if (total_time_ms > slow_time_ms) {
                model->recommended_fine_window_gn -= 0.06f;
            }
            else if (total_time_ms > good_time_ms) {
                model->recommended_fine_window_gn -= 0.025f;
            }
        }

        model->runtime_bias_gn = ai_clampf(model->runtime_bias_gn,
                                           -negative_bias_limit,
                                           positive_bias_limit);
        model->recommended_fine_window_gn = ai_clampf(
            model->recommended_fine_window_gn,
            min_fine_window,
            max_fine_window);

        if (have_coarse_feedback) {
            // Production throws expose the real coarse settling tail better than short samples.
            const float desired_fine_window =
                fmaxf(0.85f, fminf(1.10f, target_weight * 0.025f));
            const float desired_after_coarse = target_weight - desired_fine_window;
            const float handoff_error = after_coarse_settle_gn - desired_after_coarse;
            const float tail_cap = fmaxf(20.0f, target_weight * 0.55f);
            const float observed_tail = ai_clampf(observed_coarse_tail_gn, 0.0f, tail_cap);
            float coarse_tail_alpha = observed_tail > model->coarse_tail_gn ? 0.24f : 0.08f;
            if (model->coarse_tail_gn > observed_tail * 2.0f + 0.50f) {
                coarse_tail_alpha = 0.18f;
            }
            model->coarse_tail_gn =
                model->coarse_tail_gn * (1.0f - coarse_tail_alpha) +
                observed_tail * coarse_tail_alpha;
            float trim_tail_alpha = observed_tail > model->coarse_trim_tail_gn ? 0.20f : 0.10f;
            model->coarse_trim_tail_gn =
                model->coarse_trim_tail_gn * (1.0f - trim_tail_alpha) +
                observed_tail * trim_tail_alpha;

            if (handoff_error > 0.12f) {
                model->runtime_bias_gn += fminf(0.05f,
                                                fmaxf(0.006f, handoff_error * 0.025f));
            }
            else if (handoff_error < -0.12f) {
                float early_by = fminf(-handoff_error, 6.0f);
                model->runtime_bias_gn -= fminf(0.05f,
                                                fmaxf(0.004f, early_by * 0.018f));
            }

            if (model->machine.valid) {
                float min_bulk_margin = desired_fine_window + 2.25f;
                float max_bulk_margin = desired_fine_window + fmaxf(18.0f, target_weight * 0.55f);
                float min_trim_margin = desired_fine_window + 0.70f;
                float max_trim_margin = desired_fine_window + fmaxf(1.80f, target_weight * 0.055f);
                if (handoff_error > acceptable_band) {
                    float bump = fminf(fmaxf(1.00f, target_weight * 0.030f),
                                       fmaxf(0.03f, handoff_error * 0.35f));
                    model->machine.recommended_bulk_handoff_gn += bump;
                    model->machine.recommended_trim_stop_gn += bump * 0.45f;
                }
                else if (handoff_error < -acceptable_band) {
                    float relief = fminf(0.65f, fmaxf(0.03f, (-handoff_error) * 0.16f));
                    model->machine.recommended_bulk_handoff_gn -= relief;
                    model->machine.recommended_trim_stop_gn -= relief * 0.35f;
                }
                float measured_margin =
                    ai_tuning_machine_bulk_handoff_margin(model,
                                                          target_weight,
                                                          desired_fine_window);
                if (measured_margin > 0.0f) {
                    model->machine.recommended_bulk_handoff_gn =
                        fmaxf(model->machine.recommended_bulk_handoff_gn,
                              measured_margin);
                }
                model->machine.recommended_bulk_handoff_gn =
                    ai_clampf(model->machine.recommended_bulk_handoff_gn,
                              min_bulk_margin,
                              max_bulk_margin);
                model->machine.recommended_trim_stop_gn =
                    ai_clampf(model->machine.recommended_trim_stop_gn,
                              min_trim_margin,
                              max_trim_margin);
            }

            model->coarse_tail_gn = ai_clampf(model->coarse_tail_gn, 0.0f, tail_cap);
            model->coarse_trim_tail_gn =
                ai_clampf(model->coarse_trim_tail_gn, 0.0f, tail_cap);
            model->runtime_bias_gn = ai_clampf(model->runtime_bias_gn,
                                               -negative_bias_limit,
                                               positive_bias_limit);
        }
        if (have_fine_feedback) {
            const float observed_tail = ai_clampf(production_fine_tail, 0.0f, fine_tail_cap);
            float alpha = 0.10f;
            if (final_error_gn > acceptable_band && !recovery_phase_blame) {
                alpha = 0.38f;
            }
            else if (observed_tail > model->fine_tail_gn + 0.05f) {
                alpha = 0.22f;
            }

            if (observed_tail > model->fine_tail_gn ||
                (final_error_gn > acceptable_band && !recovery_phase_blame)) {
                model->fine_tail_gn =
                    model->fine_tail_gn * (1.0f - alpha) + observed_tail * alpha;
            }
            else if (final_error_gn <= 0.0f &&
                     total_time_ms > slow_time_ms &&
                     observed_tail + 0.10f < model->fine_tail_gn) {
                model->fine_tail_gn = model->fine_tail_gn * 0.96f + observed_tail * 0.04f;
            }

            if (final_error_gn > acceptable_band && !recovery_phase_blame) {
                float overcharge = fminf(final_error_gn, 0.60f);
                float fast_tail_alpha = observed_tail > model->fine_fast_tail_gn ? 0.42f : 0.16f;
                model->fine_fast_tail_gn =
                    model->fine_fast_tail_gn * (1.0f - fast_tail_alpha) +
                    observed_tail * fast_tail_alpha;
                model->fine_stop_safety_bias_gn += fmaxf(0.003f, overcharge * 0.025f);
            }
            else if (observed_tail + 0.03f < model->fine_fast_tail_gn) {
                model->fine_fast_tail_gn =
                    model->fine_fast_tail_gn * 0.94f + observed_tail * 0.06f;
            }

            model->fine_tail_gn = ai_clampf(model->fine_tail_gn, 0.0f, fine_tail_cap);
            model->fine_fast_tail_gn = ai_clampf(model->fine_fast_tail_gn, 0.0f, fine_tail_cap);
            model->runtime_bias_gn = ai_clampf(model->runtime_bias_gn,
                                               -negative_bias_limit,
                                               positive_bias_limit);
            min_fine_window = ai_tuning_min_fine_window_for_model(model);
            model->recommended_fine_window_gn = ai_clampf(
                model->recommended_fine_window_gn,
                min_fine_window,
                max_fine_window);
        }
        if (have_recovery_feedback) {
            float min_recovery_speed = 0.08f;
            float max_recovery_speed = model->fine_best_speed_rps > 0.0f
                ? fminf(0.60f, fmaxf(0.35f, model->fine_best_speed_rps * 0.12f))
                : 0.60f;
            if (!isfinite(model->fine_recovery_speed_rps) ||
                model->fine_recovery_speed_rps <= 0.0f) {
                model->fine_recovery_speed_rps = 0.20f;
            }

            bool learned_micro_tail_small =
                model->fine_micro_tail_gn <= 0.035f ||
                model->fine_recovery_tail_gn <= 0.035f;
            if (final_error_gn > acceptable_band) {
                model->fine_recovery_speed_rps *= 0.88f;
                model->fine_recovery_tail_gn =
                    fmaxf(model->fine_recovery_tail_gn, fminf(fine_tail_cap, final_error_gn));
            }
            else if (final_error_gn < -acceptable_band &&
                     final_error_gn > -0.22f &&
                     (recovery_motor_on_ms > 2500.0f ||
                      recovery_stall_count > 0 ||
                      recovery_exit_reason == 3 ||
                      (recovery_motor_was_used && recovery_motor_on_ms < 900.0f))) {
                model->fine_recovery_speed_rps *= learned_micro_tail_small ? 1.28f : 1.18f;
            }
            else if (fabsf(final_error_gn) <= acceptable_band &&
                     (recovery_stall_count >= 2 || recovery_motor_on_ms > 4500.0f)) {
                model->fine_recovery_speed_rps *= learned_micro_tail_small
                    ? (recovery_stall_count >= 3 ? 1.22f : 1.14f)
                    : (recovery_stall_count >= 3 ? 1.12f : 1.08f);
            }
            else if (fabsf(final_error_gn) <= acceptable_band &&
                     recovery_motor_on_ms > 7000.0f) {
                model->fine_recovery_speed_rps *= learned_micro_tail_small ? 1.16f : 1.08f;
            }

            model->fine_recovery_speed_rps = ai_clampf(model->fine_recovery_speed_rps,
                                                       min_recovery_speed,
                                                       max_recovery_speed);
            float observed_recovery_drop =
                fmaxf(0.0f, recovery_end_weight_gn - recovery_start_weight_gn);
            float observed_recovery_flow = (recovery_motor_on_ms > 50.0f)
                ? observed_recovery_drop / (recovery_motor_on_ms / 1000.0f)
                : 0.0f;
            float predicted_recovery_flow =
                ai_tuning_predict_flow_from_model(model,
                                                  AI_MOTOR_MODE_FINE_ONLY,
                                                  model->fine_recovery_speed_rps);
            if (observed_recovery_flow > 0.001f && observed_recovery_flow < 0.20f) {
                float alpha = recovery_motor_on_ms > 1500.0f ? 0.28f : 0.14f;
                float seed_flow = isfinite(model->fine_recovery_flow_gps) &&
                                  model->fine_recovery_flow_gps > 0.0f
                    ? model->fine_recovery_flow_gps
                    : fmaxf(predicted_recovery_flow, observed_recovery_flow);
                model->fine_recovery_flow_gps =
                    seed_flow * (1.0f - alpha) + observed_recovery_flow * alpha;
                model->fine_micro_flow_gps =
                    (isfinite(model->fine_micro_flow_gps) && model->fine_micro_flow_gps > 0.0f)
                        ? model->fine_micro_flow_gps * (1.0f - alpha) + observed_recovery_flow * alpha
                        : observed_recovery_flow;
            }
            else {
                model->fine_recovery_flow_gps = predicted_recovery_flow;
            }
            if (!isfinite(model->fine_recovery_tail_gn)) {
                model->fine_recovery_tail_gn = 0.0f;
            }
            float observed_micro_tail = 0.0f;
            if (isfinite(post_finish_peak_weight_gn) &&
                isfinite(recovery_end_weight_gn) &&
                post_finish_peak_weight_gn > recovery_end_weight_gn &&
                post_finish_peak_weight_gn < target_weight + 1.0f) {
                observed_micro_tail = post_finish_peak_weight_gn - recovery_end_weight_gn;
            }
            if (final_error_gn > acceptable_band) {
                observed_micro_tail = fmaxf(observed_micro_tail, fminf(final_error_gn, 0.18f));
            }
            if (observed_micro_tail > 0.0f) {
                float micro_tail_alpha =
                    observed_micro_tail > model->fine_micro_tail_gn ? 0.32f : 0.12f;
                model->fine_micro_tail_gn =
                    model->fine_micro_tail_gn * (1.0f - micro_tail_alpha) +
                    observed_micro_tail * micro_tail_alpha;
                model->fine_recovery_tail_gn =
                    model->fine_recovery_tail_gn * (1.0f - micro_tail_alpha) +
                    observed_micro_tail * micro_tail_alpha;
            }
            else if (final_error_gn <= 0.0f &&
                     recovery_motor_on_ms > 2500.0f &&
                     model->fine_micro_tail_gn > 0.0f) {
                model->fine_micro_tail_gn *= 0.98f;
                model->fine_recovery_tail_gn *= 0.98f;
            }
            model->fine_recovery_tail_gn = ai_clampf(model->fine_recovery_tail_gn,
                                                     0.0f,
                                                     fminf(0.18f, fine_tail_cap));
            model->fine_micro_tail_gn = ai_clampf(model->fine_micro_tail_gn,
                                                  0.0f,
                                                  fminf(0.18f, fine_tail_cap));
        }
        ai_tuning_update_finish_profile_unlocked(model, target_weight);
        history_changed = history_changed || !controller_failure;
    }

    if (history_changed) {
        save_history_to_flash();
    }
    ai_tuning_unlock();
}

void ai_tuning_calculate_refinements(uint8_t profile_idx) {
    (void)profile_idx;
}

bool ai_tuning_get_refined_params(float* coarse_a, float* coarse_b, float* fine_a, float* fine_b) {
    (void)coarse_a;
    (void)coarse_b;
    (void)fine_a;
    (void)fine_b;
    return false;
}

bool ai_tuning_get_suggestions(uint8_t profile_idx, float* coarse_a, float* coarse_b,
                               float* fine_a, float* fine_b) {
    (void)profile_idx;
    (void)coarse_a;
    (void)coarse_b;
    (void)fine_a;
    (void)fine_b;
    return false;
}

bool ai_tuning_apply_refined_params(uint8_t profile_idx) {
    (void)profile_idx;
    return false;
}

void ai_tuning_clear_history(void) {
    if (!ai_tuning_lock(pdMS_TO_TICKS(100))) {
        return;
    }

    memset(&g_history, 0, sizeof(g_history));
    g_history.revision = AI_TUNING_HISTORY_REV;
    save_history_to_flash();
    ai_tuning_unlock();
}

void ai_tuning_save_config(void) {
    save_config_to_eeprom();
}

float ai_tuning_get_scale_compensation(void) {
    return 0.0f;
}

static void load_history_from_flash(void) {
    if (g_history_loaded) {
        return;
    }

    bool ok = flash_ml_history_read((uint8_t*)&g_history, sizeof(g_history));
    if (!ok || g_history.revision != AI_TUNING_HISTORY_REV) {
        memset(&g_history, 0, sizeof(g_history));
        g_history.revision = AI_TUNING_HISTORY_REV;
    }
    else {
        g_history.observation_count = (uint8_t)ai_clampf((float)g_history.observation_count,
                                                         0.0f,
                                                         AI_RUNTIME_OBSERVATION_COUNT);
        g_history.observation_next_idx %= AI_RUNTIME_OBSERVATION_COUNT;
        ai_tuning_filter_runtime_observations_unlocked();
        for (uint8_t profile_idx = 0; profile_idx < MAX_PROFILE_CNT; profile_idx++) {
            ai_tuning_sanitize_model_unlocked(&g_history.models[profile_idx],
                                              ai_tuning_default_target_weight_for_profile_unlocked(profile_idx),
                                              true);
        }
    }

    g_history_loaded = true;
}

static void save_history_to_flash(void) {
    flash_ml_history_write((const uint8_t*)&g_history, sizeof(g_history));
}

static void load_config_from_eeprom(void) {
    if (g_config_loaded) {
        return;
    }

    bool ok = eeprom_read(EEPROM_AI_TUNING_CONFIG_BASE_ADDR,
                          (uint8_t*)&g_config_eeprom,
                          sizeof(g_config_eeprom));

    if (ok && g_config_eeprom.revision == AI_TUNING_CONFIG_REV) {
        g_config.coarse_budget_gn = g_config_eeprom.coarse_budget_gn;
        g_config.fine_budget_gn = g_config_eeprom.fine_budget_gn;
        g_config.coarse_sample_count = g_config_eeprom.coarse_sample_count;
        g_config.fine_sample_count = g_config_eeprom.fine_sample_count;
        g_config.coarse_sample_target_gn = g_config_eeprom.coarse_sample_target_gn;
        g_config.fine_sample_target_gn = g_config_eeprom.fine_sample_target_gn;
        g_config.noise_margin = g_config_eeprom.noise_margin;
        g_config.time_cost_weight = g_config_eeprom.time_cost_weight;
        g_config.error_cost_weight = g_config_eeprom.error_cost_weight;
    }

    g_config.coarse_budget_gn = ai_clampf(g_config.coarse_budget_gn, 20.0f, 500.0f);
    g_config.fine_budget_gn = ai_clampf(g_config.fine_budget_gn, 5.0f, 150.0f);
    g_config.coarse_sample_count = (uint8_t)ai_clampf((float)g_config.coarse_sample_count, 2.0f, AI_TUNING_STAGE_SAMPLE_COUNT);
    g_config.fine_sample_count = (uint8_t)ai_clampf((float)g_config.fine_sample_count, 2.0f, AI_TUNING_STAGE_SAMPLE_COUNT);
    g_config.coarse_sample_target_gn = ai_clampf(g_config.coarse_sample_target_gn, 2.0f, 50.0f);
    g_config.fine_sample_target_gn = ai_clampf(g_config.fine_sample_target_gn, 0.2f, 10.0f);
    g_config.noise_margin = ai_clampf(g_config.noise_margin, 0.005f, 0.25f);
    g_config.time_cost_weight = ai_clampf(g_config.time_cost_weight, 0.1f, 20.0f);
    g_config.error_cost_weight = ai_clampf(g_config.error_cost_weight, 0.1f, 50.0f);
    g_config_loaded = true;
}

static bool save_config_to_eeprom(void) {
    g_config_eeprom.revision = AI_TUNING_CONFIG_REV;
    g_config_eeprom.coarse_budget_gn = g_config.coarse_budget_gn;
    g_config_eeprom.fine_budget_gn = g_config.fine_budget_gn;
    g_config_eeprom.coarse_sample_count = g_config.coarse_sample_count;
    g_config_eeprom.fine_sample_count = g_config.fine_sample_count;
    g_config_eeprom.coarse_sample_target_gn = g_config.coarse_sample_target_gn;
    g_config_eeprom.fine_sample_target_gn = g_config.fine_sample_target_gn;
    g_config_eeprom.noise_margin = g_config.noise_margin;
    g_config_eeprom.time_cost_weight = g_config.time_cost_weight;
    g_config_eeprom.error_cost_weight = g_config.error_cost_weight;

    return eeprom_write(EEPROM_AI_TUNING_CONFIG_BASE_ADDR,
                        (uint8_t*)&g_config_eeprom,
                        sizeof(g_config_eeprom));
}

static bool ai_tuning_lock(TickType_t timeout_ticks) {
    if (g_ai_tuning_mutex == NULL) {
        return true;
    }
    return xSemaphoreTakeRecursive(g_ai_tuning_mutex, timeout_ticks) == pdTRUE;
}

static void ai_tuning_unlock(void) {
    if (g_ai_tuning_mutex != NULL) {
        xSemaphoreGiveRecursive(g_ai_tuning_mutex);
    }
}

static void ai_tuning_reset_session_unlocked(void) {
    memset(&g_session, 0, sizeof(g_session));
    g_session.state = AI_TUNING_IDLE;
}

static bool ai_tuning_is_active_unlocked(void) {
    return g_session.state == AI_TUNING_CHARACTERIZING_COARSE ||
           g_session.state == AI_TUNING_CHARACTERIZING_FINE ||
           g_session.state == AI_TUNING_CALIBRATING_COARSE ||
           g_session.state == AI_TUNING_CALIBRATING_FINE;
}

static bool ai_tuning_is_complete_unlocked(void) {
    return g_session.state == AI_TUNING_READY_TO_SAVE;
}

static ai_motor_mode_t ai_tuning_get_motor_mode_unlocked(void) {
    switch (g_session.state) {
        case AI_TUNING_CHARACTERIZING_COARSE:
        case AI_TUNING_CALIBRATING_COARSE:
            return AI_MOTOR_MODE_COARSE_ONLY;
        case AI_TUNING_CHARACTERIZING_FINE:
        case AI_TUNING_CALIBRATING_FINE:
            return AI_MOTOR_MODE_FINE_ONLY;
        default:
            return AI_MOTOR_MODE_NORMAL;
    }
}

static uint8_t ai_tuning_get_progress_percent_unlocked(void) {
    if (g_session.total_samples_planned == 0) {
        return 0;
    }

    uint32_t progress = ((uint32_t)g_session.drops_completed * 100u) / g_session.total_samples_planned;
    if (progress > 100u) {
        progress = 100u;
    }
    return (uint8_t)progress;
}

static uint8_t ai_tuning_get_profile_index_for_pointer_unlocked(profile_t* profile) {
    uint8_t idx = 0;
    if (profile_get_idx_for_pointer(profile, &idx)) {
        return idx;
    }

    return (uint8_t)profile_get_selected_idx();
}

static void ai_tuning_update_session_stats_unlocked(void) {
    uint8_t count = (g_session.drops_completed < AI_TUNING_DROP_BUF_SIZE)
        ? g_session.drops_completed
        : AI_TUNING_DROP_BUF_SIZE;

    if (count == 0) {
        g_session.avg_final_error_gn = 0.0f;
        g_session.avg_total_time_ms = 0.0f;
        return;
    }

    float total_error = 0.0f;
    float total_time = 0.0f;
    for (uint8_t idx = 0; idx < count; idx++) {
        total_error += g_session.drops[idx].overthrow;
        total_time += g_session.drops[idx].total_time_ms;
    }

    g_session.avg_final_error_gn = total_error / count;
    g_session.avg_total_time_ms = total_time / count;
}

static uint8_t ai_tuning_get_stage_sample_limit_unlocked(ai_tuning_state_t state) {
    if (state == AI_TUNING_CALIBRATING_COARSE) {
        return AI_MACHINE_CAL_COARSE_SAMPLE_COUNT;
    }
    if (state == AI_TUNING_CALIBRATING_FINE) {
        return AI_MACHINE_CAL_FINE_SAMPLE_COUNT;
    }
    return (state == AI_TUNING_CHARACTERIZING_FINE)
        ? g_config.fine_sample_count
        : g_config.coarse_sample_count;
}

static float ai_tuning_get_stage_budget_limit_unlocked(ai_tuning_state_t state) {
    if (state == AI_TUNING_CALIBRATING_COARSE) {
        return fmaxf(80.0f, g_config.coarse_budget_gn * 0.65f);
    }
    if (state == AI_TUNING_CALIBRATING_FINE) {
        return fmaxf(12.0f, g_config.fine_budget_gn * 0.45f);
    }
    return (state == AI_TUNING_CHARACTERIZING_FINE)
        ? g_config.fine_budget_gn
        : g_config.coarse_budget_gn;
}

static bool ai_tuning_is_fine_recovery_sample_unlocked(ai_tuning_state_t state, uint8_t sample_index) {
    return (state == AI_TUNING_CHARACTERIZING_FINE ||
            state == AI_TUNING_CALIBRATING_FINE) &&
           sample_index < AI_TUNING_FINE_RECOVERY_SAMPLE_COUNT;
}

static float ai_tuning_fine_recovery_speed_for_sample(uint8_t sample_index) {
    static const float recovery_speeds[AI_TUNING_FINE_RECOVERY_SAMPLE_COUNT] = {
        0.08f, 0.12f, 0.20f, 0.35f
    };
    return recovery_speeds[sample_index % AI_TUNING_FINE_RECOVERY_SAMPLE_COUNT];
}

static float ai_tuning_fine_recovery_on_time_ms_for_sample(uint8_t sample_index) {
    static const float recovery_on_times_ms[AI_TUNING_FINE_RECOVERY_SAMPLE_COUNT] = {
        12000.0f, 10000.0f, 8000.0f, 6500.0f
    };
    return recovery_on_times_ms[sample_index % AI_TUNING_FINE_RECOVERY_SAMPLE_COUNT];
}

static float ai_tuning_interpolate_stage_speed_unlocked(ai_tuning_state_t state, uint8_t sample_index) {
    if (g_session.target_profile == NULL) {
        return 0.0f;
    }

    float min_speed = 0.0f;
    float profile_max_speed = 0.0f;
    float motor_min_speed = 0.0f;
    float motor_max_speed = 0.0f;
    float hardware_cap = 0.0f;
    if (state == AI_TUNING_CHARACTERIZING_FINE) {
        min_speed = g_session.target_profile->fine_min_flow_speed_rps;
        profile_max_speed = g_session.target_profile->fine_max_flow_speed_rps;
        motor_min_speed = get_motor_min_speed(SELECT_FINE_TRICKLER_MOTOR);
        motor_max_speed = (float)get_motor_max_speed(SELECT_FINE_TRICKLER_MOTOR);
        hardware_cap = 6.0f;
    } else {
        min_speed = g_session.target_profile->coarse_min_flow_speed_rps;
        profile_max_speed = g_session.target_profile->coarse_max_flow_speed_rps;
        motor_min_speed = get_motor_min_speed(SELECT_COARSE_TRICKLER_MOTOR);
        motor_max_speed = (float)get_motor_max_speed(SELECT_COARSE_TRICKLER_MOTOR);
        hardware_cap = 8.0f;
    }

    min_speed = fmaxf(min_speed, motor_min_speed);
    float max_speed = fminf(motor_max_speed, hardware_cap);
    if (max_speed <= 0.0f) {
        max_speed = fminf(profile_max_speed, hardware_cap);
    }

    if (max_speed < min_speed) {
        float temp = max_speed;
        max_speed = min_speed;
        min_speed = temp;
    }

    uint8_t sample_limit = ai_tuning_get_stage_sample_limit_unlocked(state);
    if (sample_limit <= 1 || fabsf(max_speed - min_speed) < 0.001f) {
        return max_speed;
    }

    if (ai_tuning_is_fine_recovery_sample_unlocked(state, sample_index)) {
        return ai_clampf(ai_tuning_fine_recovery_speed_for_sample(sample_index),
                         min_speed,
                         max_speed);
    }

    static const float coarse_speed_ratios[AI_TUNING_STAGE_SAMPLE_COUNT] = {
        0.70f, 1.00f, 0.45f, 0.85f, 0.25f, 0.60f,
        0.10f, 0.95f, 0.35f, 0.75f, 0.50f, 0.15f
    };
    static const float fine_speed_ratios[AI_TUNING_STAGE_SAMPLE_COUNT] = {
        0.70f, 1.00f, 0.45f, 0.85f, 0.25f, 0.60f,
        0.12f, 0.95f, 0.35f, 0.75f, 0.00f, 0.50f
    };
    const float* speed_ratios = (state == AI_TUNING_CHARACTERIZING_FINE)
        ? fine_speed_ratios
        : coarse_speed_ratios;
    float ratio = speed_ratios[sample_index % AI_TUNING_STAGE_SAMPLE_COUNT];
    return min_speed + (max_speed - min_speed) * ratio;
}

static void ai_tuning_fit_stage_samples(const ai_flow_sample_t* samples, uint8_t count,
                                        float* slope, float* intercept, float* avg_tail) {
    *slope = 0.0f;
    *intercept = 0.0f;
    *avg_tail = 0.0f;

    if (samples == NULL || count == 0) {
        return;
    }

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_xx = 0.0f;
    float sum_xy = 0.0f;
    uint8_t valid_count = 0;

    for (uint8_t idx = 0; idx < count; idx++) {
        if (samples[idx].delivered_weight <= g_config.noise_margin * 2.0f ||
            samples[idx].flow_gps <= 0.001f) {
            continue;
        }

        sum_x += samples[idx].speed_rps;
        sum_y += samples[idx].flow_gps;
        sum_xx += samples[idx].speed_rps * samples[idx].speed_rps;
        sum_xy += samples[idx].speed_rps * samples[idx].flow_gps;
        *avg_tail += samples[idx].tail_weight;
        valid_count++;
    }

    if (valid_count == 0) {
        return;
    }

    *avg_tail /= valid_count;
    if (valid_count == 1) {
        *intercept = sum_y;
        return;
    }

    float denom = (valid_count * sum_xx) - (sum_x * sum_x);
    if (fabsf(denom) < 0.0001f) {
        *intercept = sum_y / valid_count;
        return;
    }

    *slope = ((valid_count * sum_xy) - (sum_x * sum_y)) / denom;
    *intercept = (sum_y - (*slope * sum_x)) / valid_count;
}

static float ai_tuning_predict_flow_from_model(const ai_profile_model_t* model,
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
    } else {
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
            nearest_idx = idx;
            nearest_delta = delta;
        }
    }

    const float sample_speed = fmaxf(samples[nearest_idx].speed_rps, 0.1f);
    return fmaxf(0.0f, samples[nearest_idx].flow_gps * (speed_rps / sample_speed));
}

static float ai_tuning_estimate_flow_unlocked(ai_tuning_state_t state, float speed_rps) {
    const ai_motor_mode_t motor_mode = (state == AI_TUNING_CHARACTERIZING_FINE)
        ? AI_MOTOR_MODE_FINE_ONLY
        : AI_MOTOR_MODE_COARSE_ONLY;

    float flow = ai_tuning_predict_flow_from_model(&g_session.working_model, motor_mode, speed_rps);
    if (flow > 0.0f) {
        return flow;
    }

    if (state == AI_TUNING_CHARACTERIZING_FINE) {
        return g_config.fine_sample_target_gn / 4.0f;
    }
    return g_config.coarse_sample_target_gn / 0.90f;
}

static float ai_tuning_estimate_on_time_ms_unlocked(ai_tuning_state_t state,
                                                    float speed_rps,
                                                    float target_weight) {
    float estimated_flow = ai_tuning_estimate_flow_unlocked(state, speed_rps);
    if (estimated_flow <= 0.0f) {
        return (state == AI_TUNING_CHARACTERIZING_FINE) ? 700.0f : 900.0f;
    }

    float on_time_ms = (target_weight / estimated_flow) * 1000.0f;
    if (state == AI_TUNING_CHARACTERIZING_FINE) {
        return ai_clampf(on_time_ms, 500.0f, 12000.0f);
    }
    return ai_clampf(on_time_ms, 350.0f, 1500.0f);
}

static float ai_tuning_coarse_characterization_max_on_time_ms(uint8_t sample_index) {
    static const float max_on_times_ms[AI_TUNING_STAGE_SAMPLE_COUNT] = {
        120.0f, 160.0f, 220.0f, 300.0f,
        380.0f, 500.0f, 650.0f, 800.0f,
        950.0f, 1100.0f, 1250.0f, 1500.0f
    };
    return max_on_times_ms[sample_index % AI_TUNING_STAGE_SAMPLE_COUNT];
}

static void ai_tuning_rebuild_stage_model_unlocked(ai_tuning_state_t state) {
    float slope = 0.0f;
    float intercept = 0.0f;
    float avg_tail = 0.0f;

    if (state == AI_TUNING_CHARACTERIZING_FINE) {
        ai_tuning_fit_stage_samples(g_session.working_model.fine_samples,
                                    g_session.working_model.fine_sample_count,
                                    &slope, &intercept, &avg_tail);
        g_session.working_model.fine_flow_slope = slope;
        g_session.working_model.fine_flow_intercept = intercept;
        g_session.working_model.fine_tail_gn = avg_tail;
        ai_tuning_choose_recovery_stage_sample(
            g_session.working_model.fine_recovery_samples,
            g_session.working_model.fine_recovery_sample_count,
            &g_session.working_model.fine_recovery_speed_rps,
            &g_session.working_model.fine_recovery_flow_gps,
            &g_session.working_model.fine_recovery_tail_gn);
    } else {
        ai_tuning_fit_stage_samples(g_session.working_model.coarse_samples,
                                    g_session.working_model.coarse_sample_count,
                                    &slope, &intercept, &avg_tail);
        g_session.working_model.coarse_flow_slope = slope;
        g_session.working_model.coarse_flow_intercept = intercept;
        g_session.working_model.coarse_tail_gn = avg_tail;
    }
}

static void ai_tuning_choose_best_stage_sample(const ai_flow_sample_t* samples, uint8_t count,
                                               float tail_penalty,
                                               float* best_speed_rps,
                                               float* best_flow_gps) {
    *best_speed_rps = 0.0f;
    *best_flow_gps = 0.0f;

    if (samples == NULL || count == 0) {
        return;
    }

    float best_score = -1000000.0f;
    for (uint8_t idx = 0; idx < count; idx++) {
        if (samples[idx].delivered_weight <= g_config.noise_margin * 2.0f ||
            samples[idx].flow_gps <= 0.001f) {
            continue;
        }

    float score = samples[idx].flow_gps /
                  (1.0f + tail_penalty * fmaxf(samples[idx].tail_weight, 0.0f));
        if (score > best_score) {
            best_score = score;
            *best_speed_rps = samples[idx].speed_rps;
            *best_flow_gps = samples[idx].flow_gps;
        }
    }
}

static void ai_tuning_choose_bulk_stage_sample(const ai_flow_sample_t* samples, uint8_t count,
                                               float* best_speed_rps,
                                               float* best_flow_gps) {
    if (best_speed_rps == NULL || best_flow_gps == NULL) {
        return;
    }

    *best_speed_rps = 0.0f;
    *best_flow_gps = 0.0f;

    bool found = false;
    float best_score = -1000000.0f;
    for (uint8_t idx = 0; idx < count; idx++) {
        const ai_flow_sample_t* sample = &samples[idx];
        if (sample->delivered_weight < fmaxf(2.0f, g_config.noise_margin * 10.0f) ||
            sample->flow_gps <= 0.001f ||
            sample->speed_rps <= 0.0f) {
            continue;
        }

        float tail = fmaxf(sample->tail_weight, 0.0f);
        if (tail > 4.0f ||
            tail > sample->delivered_weight * 0.55f) {
            continue;
        }

        float score = sample->flow_gps / (1.0f + tail * 0.35f);
        if (!found || score > best_score) {
            found = true;
            best_score = score;
            *best_speed_rps = sample->speed_rps;
            *best_flow_gps = sample->flow_gps;
        }
    }

    if (!found) {
        float safest_score = 1000000.0f;
        for (uint8_t idx = 0; idx < count; idx++) {
            const ai_flow_sample_t* sample = &samples[idx];
            if (sample->delivered_weight < fmaxf(2.0f, g_config.noise_margin * 10.0f) ||
                sample->flow_gps <= 0.001f ||
                sample->speed_rps <= 0.0f) {
                continue;
            }

            float tail = fmaxf(sample->tail_weight, 0.0f);
            float flow = fmaxf(sample->flow_gps, 0.001f);
            float score = tail * 10.0f + (1.0f / flow);
            if (score < safest_score) {
                found = true;
                safest_score = score;
                *best_speed_rps = sample->speed_rps;
                *best_flow_gps = sample->flow_gps;
            }
        }
    }
}

static void ai_tuning_choose_trim_stage_sample(const ai_flow_sample_t* samples, uint8_t count,
                                               float* trim_speed_rps,
                                               float* trim_flow_gps,
                                               float* trim_tail_gn) {
    *trim_speed_rps = 0.0f;
    *trim_flow_gps = 0.0f;
    *trim_tail_gn = 0.0f;

    if (samples == NULL || count == 0) {
        return;
    }

    float best_score = 1000000.0f;
    for (uint8_t idx = 0; idx < count; idx++) {
        if (samples[idx].delivered_weight <= g_config.noise_margin * 2.0f ||
            samples[idx].flow_gps <= 0.001f) {
            continue;
        }

        float tail = fmaxf(samples[idx].tail_weight, 0.0f);
        float flow = fmaxf(samples[idx].flow_gps, 0.001f);
        float score = tail * 10.0f + (1.0f / flow);
        if (score < best_score) {
            best_score = score;
            *trim_speed_rps = samples[idx].speed_rps;
            *trim_flow_gps = samples[idx].flow_gps;
            *trim_tail_gn = tail;
        }
    }
}

static void ai_tuning_choose_recovery_stage_sample(const ai_flow_sample_t* samples, uint8_t count,
                                                   float* recovery_speed_rps,
                                                   float* recovery_flow_gps,
                                                   float* recovery_tail_gn) {
    *recovery_speed_rps = 0.0f;
    *recovery_flow_gps = 0.0f;
    *recovery_tail_gn = 0.0f;

    if (samples == NULL || count == 0) {
        return;
    }

    const float target_micro_flow_gps = 0.050f;
    bool found = false;
    float best_score = 1000000.0f;
    for (uint8_t idx = 0; idx < count; idx++) {
        const ai_flow_sample_t* sample = &samples[idx];
        if (sample->speed_rps <= 0.0f ||
            sample->flow_gps <= 0.001f ||
            sample->delivered_weight <= g_config.noise_margin * 0.40f) {
            continue;
        }

        float flow = fmaxf(sample->flow_gps, 0.001f);
        float tail = fmaxf(sample->tail_weight, 0.0f);
        float flow_error = fabsf(flow - target_micro_flow_gps);
        float overspeed_penalty = flow > 0.110f ? (flow - 0.110f) * 7.0f : 0.0f;
        float score = flow_error * 11.0f + tail * 1.6f + overspeed_penalty;
        if (!found || score < best_score) {
            found = true;
            best_score = score;
            *recovery_speed_rps = sample->speed_rps;
            *recovery_flow_gps = sample->flow_gps;
            *recovery_tail_gn = tail;
        }
    }
}

static float ai_tuning_default_target_weight_for_profile_unlocked(uint8_t profile_idx) {
    for (uint8_t offset = 0; offset < g_history.observation_count; offset++) {
        int idx = (int)g_history.observation_next_idx - 1 - (int)offset;
        if (idx < 0) {
            idx += AI_RUNTIME_OBSERVATION_COUNT;
        }

        const ai_runtime_observation_t* obs = &g_history.observations[idx];
        if (obs->profile_idx == profile_idx &&
            isfinite(obs->target_weight) &&
            obs->target_weight > 0.0f) {
            return obs->target_weight;
        }
    }

    return 40.0f;
}

static float ai_tuning_estimate_coarse_tail_guard(const ai_flow_sample_t* samples,
                                                  uint8_t count,
                                                  float average_tail_gn,
                                                  float target_weight) {
    float max_tail = fmaxf(average_tail_gn, 0.0f);

    for (uint8_t idx = 0; idx < count; idx++) {
        const ai_flow_sample_t* sample = &samples[idx];
        if (sample->delivered_weight <= g_config.noise_margin * 2.0f ||
            sample->flow_gps <= 0.001f ||
            sample->speed_rps <= 0.0f) {
            continue;
        }
        max_tail = fmaxf(max_tail, fmaxf(sample->tail_weight, 0.0f));
    }

    float guard_tail = fmaxf(fmaxf(average_tail_gn, 0.0f) * 1.02f,
                             max_tail * 0.72f);
    float guard_cap = fmaxf(4.50f, target_weight * 0.28f);
    return ai_clampf(guard_tail, 0.0f, guard_cap);
}

static float ai_tuning_estimate_fine_tail_guard(const ai_flow_sample_t* samples,
                                                uint8_t count,
                                                float average_tail_gn,
                                                float target_weight) {
    float max_tail = fmaxf(average_tail_gn, 0.0f);

    for (uint8_t idx = 0; idx < count; idx++) {
        const ai_flow_sample_t* sample = &samples[idx];
        if (sample->delivered_weight <= g_config.noise_margin * 2.0f ||
            sample->flow_gps <= 0.001f ||
            sample->speed_rps <= 0.0f) {
            continue;
        }
        max_tail = fmaxf(max_tail, fmaxf(sample->tail_weight, 0.0f));
    }

    float guard_tail = fmaxf(fmaxf(average_tail_gn, 0.0f) * 1.12f,
                             max_tail * 0.92f);
    float guard_cap = fmaxf(0.75f, fminf(2.00f, target_weight * 0.05f));
    return ai_clampf(guard_tail, 0.0f, guard_cap);
}

static void ai_tuning_reset_machine_calibration(ai_machine_calibration_t* cal) {
    if (cal == NULL) {
        return;
    }
    memset(cal, 0, sizeof(*cal));
}

static void ai_tuning_cal_update_mean(float* mean, uint8_t count, float value) {
    if (mean == NULL || !isfinite(value) || count == 0) {
        return;
    }
    if (count == 1 || !isfinite(*mean) || *mean <= 0.0f) {
        *mean = value;
    }
    else {
        *mean += (value - *mean) / (float)count;
    }
}

static void ai_tuning_cal_update_max(float* high, float value) {
    if (high == NULL || !isfinite(value)) {
        return;
    }
    if (!isfinite(*high) || value > *high) {
        *high = value;
    }
}

static void ai_tuning_sanitize_machine_calibration(ai_profile_model_t* model, float target_weight) {
    if (model == NULL) {
        return;
    }

    ai_machine_calibration_t* cal = &model->machine;
    float desired_fine_window = fmaxf(0.85f, fminf(1.10f, target_weight * 0.025f));
    cal->valid = cal->coarse_sample_count >= 3 && cal->fine_sample_count >= 3;

    if (!isfinite(cal->scale_sample_period_ms) || cal->scale_sample_period_ms <= 0.0f) {
        cal->scale_sample_period_ms = 750.0f;
    }
    cal->scale_sample_period_ms = ai_clampf(cal->scale_sample_period_ms, 80.0f, 1400.0f);

    cal->coarse_first_response_ms = ai_clampf(isfinite(cal->coarse_first_response_ms)
                                                  ? cal->coarse_first_response_ms : cal->scale_sample_period_ms,
                                              0.0f,
                                              2500.0f);
    cal->fine_first_response_ms = ai_clampf(isfinite(cal->fine_first_response_ms)
                                                ? cal->fine_first_response_ms : cal->scale_sample_period_ms,
                                            0.0f,
                                            2500.0f);
    cal->coarse_settle_ms = ai_clampf(isfinite(cal->coarse_settle_ms)
                                          ? cal->coarse_settle_ms : cal->scale_sample_period_ms * 2.0f,
                                      150.0f,
                                      4000.0f);
    cal->fine_settle_ms = ai_clampf(isfinite(cal->fine_settle_ms)
                                        ? cal->fine_settle_ms : cal->scale_sample_period_ms * 2.0f,
                                    150.0f,
                                    4000.0f);

    cal->coarse_tail_avg_gn = ai_clampf(isfinite(cal->coarse_tail_avg_gn)
                                            ? cal->coarse_tail_avg_gn : 0.0f,
                                        0.0f,
                                        fmaxf(20.0f, target_weight * 0.55f));
    cal->coarse_tail_p95_gn = ai_clampf(isfinite(cal->coarse_tail_p95_gn)
                                            ? fmaxf(cal->coarse_tail_p95_gn, cal->coarse_tail_avg_gn)
                                            : cal->coarse_tail_avg_gn,
                                        0.0f,
                                        fmaxf(20.0f, target_weight * 0.55f));
    cal->fine_tail_avg_gn = ai_clampf(isfinite(cal->fine_tail_avg_gn)
                                          ? cal->fine_tail_avg_gn : 0.0f,
                                      0.0f,
                                      2.5f);
    cal->fine_tail_p95_gn = ai_clampf(isfinite(cal->fine_tail_p95_gn)
                                          ? fmaxf(cal->fine_tail_p95_gn, cal->fine_tail_avg_gn)
                                          : cal->fine_tail_avg_gn,
                                      0.0f,
                                      3.0f);

    cal->coarse_uncertainty_gn = ai_clampf(isfinite(cal->coarse_uncertainty_gn)
                                                ? cal->coarse_uncertainty_gn : 0.0f,
                                            0.0f,
                                            fmaxf(5.0f, target_weight * 0.14f));
    cal->fine_uncertainty_gn = ai_clampf(isfinite(cal->fine_uncertainty_gn)
                                             ? cal->fine_uncertainty_gn : 0.0f,
                                         0.0f,
                                         0.50f);
    float coarse_flow_fallback = isfinite(model->coarse_best_flow_gps) ? model->coarse_best_flow_gps : 0.0f;
    float trim_flow_fallback = isfinite(model->coarse_trim_flow_gps) ? model->coarse_trim_flow_gps : 0.0f;
    float fine_flow_fallback = isfinite(model->fine_best_flow_gps) ? model->fine_best_flow_gps : 0.0f;
    float micro_flow_fallback = isfinite(model->fine_recovery_flow_gps) ? model->fine_recovery_flow_gps : 0.0f;
    cal->coarse_open_loop_flow_gps =
        ai_clampf((isfinite(cal->coarse_open_loop_flow_gps) && cal->coarse_open_loop_flow_gps > 0.05f)
                      ? cal->coarse_open_loop_flow_gps : coarse_flow_fallback,
                  0.0f,
                  220.0f);
    cal->trim_open_loop_flow_gps =
        ai_clampf((isfinite(cal->trim_open_loop_flow_gps) && cal->trim_open_loop_flow_gps > 0.05f)
                      ? cal->trim_open_loop_flow_gps : trim_flow_fallback,
                  0.0f,
                  120.0f);
    cal->fine_open_loop_flow_gps =
        ai_clampf((isfinite(cal->fine_open_loop_flow_gps) && cal->fine_open_loop_flow_gps > 0.005f)
                      ? cal->fine_open_loop_flow_gps : fine_flow_fallback,
                  0.0f,
                  8.0f);
    cal->micro_open_loop_flow_gps =
        ai_clampf((isfinite(cal->micro_open_loop_flow_gps) && cal->micro_open_loop_flow_gps > 0.0005f)
                      ? cal->micro_open_loop_flow_gps : micro_flow_fallback,
                  0.0f,
                  1.0f);
    if (coarse_flow_fallback > 0.05f &&
        cal->coarse_open_loop_flow_gps < coarse_flow_fallback * 0.70f) {
        cal->coarse_open_loop_flow_gps = coarse_flow_fallback * 0.92f;
    }
    if (trim_flow_fallback > 0.05f &&
        cal->trim_open_loop_flow_gps < trim_flow_fallback * 0.60f) {
        cal->trim_open_loop_flow_gps = trim_flow_fallback * 0.85f;
    }
    if (fine_flow_fallback > 0.005f &&
        cal->fine_open_loop_flow_gps < fine_flow_fallback * 0.60f) {
        cal->fine_open_loop_flow_gps = fine_flow_fallback * 0.85f;
    }
    if (cal->coarse_open_loop_flow_gps <= 0.05f ||
        cal->fine_open_loop_flow_gps <= 0.005f) {
        cal->valid = false;
    }

    float coarse_timing_guard =
        ai_tuning_machine_coarse_tail_guard(cal, target_weight) +
        fminf(fmaxf(2.25f, target_weight * 0.070f),
              cal->coarse_uncertainty_gn * 0.70f) +
        fminf(fmaxf(0.85f, target_weight * 0.040f),
              cal->coarse_open_loop_flow_gps *
                  ((cal->scale_sample_period_ms + cal->coarse_first_response_ms) / 1000.0f) *
                  0.85f);
    float fine_timing_guard =
        cal->fine_tail_p95_gn * 0.55f +
        cal->fine_uncertainty_gn * 0.80f +
        fmaxf(0.12f, cal->scale_sample_period_ms / 1000.0f * 0.18f);

    float default_bulk_margin =
        ai_clampf(desired_fine_window + coarse_timing_guard,
                  desired_fine_window + 2.25f,
                  desired_fine_window + fmaxf(18.0f, target_weight * 0.55f));
    float default_trim_margin =
        desired_fine_window + ai_clampf(fine_timing_guard,
                                        0.70f,
                                        fmaxf(1.80f, target_weight * 0.055f));

    if (!isfinite(cal->recommended_bulk_handoff_gn) ||
        cal->recommended_bulk_handoff_gn <= 0.0f) {
        cal->recommended_bulk_handoff_gn = default_bulk_margin;
    }
    else {
        cal->recommended_bulk_handoff_gn =
            fmaxf(cal->recommended_bulk_handoff_gn, default_bulk_margin);
    }

    if (!isfinite(cal->recommended_trim_stop_gn) ||
        cal->recommended_trim_stop_gn <= 0.0f) {
        cal->recommended_trim_stop_gn = default_trim_margin;
    }
    else {
        cal->recommended_trim_stop_gn =
            cal->recommended_trim_stop_gn * 0.70f + default_trim_margin * 0.30f;
    }

    cal->recommended_bulk_handoff_gn =
        ai_clampf(cal->recommended_bulk_handoff_gn,
                  desired_fine_window + 2.25f,
                  desired_fine_window + fmaxf(18.0f, target_weight * 0.55f));
    cal->recommended_trim_stop_gn =
        ai_clampf(cal->recommended_trim_stop_gn,
                  desired_fine_window + 0.70f,
                  desired_fine_window + fmaxf(1.80f, target_weight * 0.055f));
    cal->post_finish_watch_ms =
        ai_clampf(fmaxf(cal->fine_settle_ms, cal->scale_sample_period_ms * 3.0f),
                  900.0f,
                  4500.0f);
}

static void ai_tuning_clamp_steering_unlocked(ai_profile_model_t* model) {
    if (model == NULL) {
        return;
    }

    model->steering_bulk_bias_gn = ai_clampf(isfinite(model->steering_bulk_bias_gn)
                                                 ? model->steering_bulk_bias_gn : 0.0f,
                                             -0.75f,
                                             1.50f);
    model->steering_fine_bias_gn = ai_clampf(isfinite(model->steering_fine_bias_gn)
                                                 ? model->steering_fine_bias_gn : 0.0f,
                                             -0.08f,
                                             0.25f);
    model->steering_recovery_speed_scale =
        ai_clampf(isfinite(model->steering_recovery_speed_scale)
                      ? model->steering_recovery_speed_scale : 0.0f,
                  -0.35f,
                  0.60f);
    if (!isfinite(model->steering_last_bulk_bias_gn)) {
        model->steering_last_bulk_bias_gn = model->steering_bulk_bias_gn;
    }
    if (!isfinite(model->steering_last_fine_bias_gn)) {
        model->steering_last_fine_bias_gn = model->steering_fine_bias_gn;
    }
    if (!isfinite(model->steering_last_recovery_speed_scale)) {
        model->steering_last_recovery_speed_scale = model->steering_recovery_speed_scale;
    }
    model->steering_undo_available = model->steering_undo_available ? 1 : 0;
}

static void ai_tuning_update_finish_profile_unlocked(ai_profile_model_t* model, float target_weight) {
    if (model == NULL) {
        return;
    }

    const float fine_tail_cap = fmaxf(0.75f, fminf(2.00f, target_weight * 0.05f));
    float best_fine_flow = isfinite(model->fine_best_flow_gps) ? model->fine_best_flow_gps : 0.0f;
    float best_fine_tail = isfinite(model->fine_tail_gn) ? model->fine_tail_gn : 0.0f;
    float micro_flow = isfinite(model->fine_recovery_flow_gps) ? model->fine_recovery_flow_gps : 0.0f;
    float micro_tail = isfinite(model->fine_recovery_tail_gn) ? model->fine_recovery_tail_gn : 0.0f;

    if (!isfinite(model->fine_fast_flow_gps) || model->fine_fast_flow_gps <= 0.0f) {
        model->fine_fast_flow_gps = best_fine_flow;
    }
    else if (best_fine_flow > 0.005f) {
        model->fine_fast_flow_gps = model->fine_fast_flow_gps * 0.85f + best_fine_flow * 0.15f;
    }

    if (!isfinite(model->fine_fast_tail_gn) || model->fine_fast_tail_gn <= 0.0f) {
        model->fine_fast_tail_gn = best_fine_tail;
    }
    else if (best_fine_tail > model->fine_fast_tail_gn) {
        model->fine_fast_tail_gn = model->fine_fast_tail_gn * 0.75f + best_fine_tail * 0.25f;
    }

    if (!isfinite(model->fine_micro_flow_gps) || model->fine_micro_flow_gps <= 0.0f) {
        model->fine_micro_flow_gps = micro_flow;
    }
    else if (micro_flow > 0.001f) {
        model->fine_micro_flow_gps = model->fine_micro_flow_gps * 0.80f + micro_flow * 0.20f;
    }

    if (!isfinite(model->fine_micro_tail_gn) || model->fine_micro_tail_gn <= 0.0f) {
        model->fine_micro_tail_gn = micro_tail;
    }
    else if (micro_tail > model->fine_micro_tail_gn) {
        model->fine_micro_tail_gn = model->fine_micro_tail_gn * 0.72f + micro_tail * 0.28f;
    }

    model->fine_fast_flow_gps = ai_clampf(model->fine_fast_flow_gps, 0.0f, 8.0f);
    model->fine_fast_tail_gn = ai_clampf(model->fine_fast_tail_gn, 0.0f, fine_tail_cap);
    model->fine_micro_flow_gps = ai_clampf(model->fine_micro_flow_gps, 0.0f, 1.0f);
    model->fine_micro_tail_gn = ai_clampf(model->fine_micro_tail_gn, 0.0f, fminf(0.25f, fine_tail_cap));
    if (best_fine_tail > 0.0f) {
        float fast_tail_ceiling = fmaxf(best_fine_tail + 0.35f, best_fine_tail * 1.50f);
        model->fine_fast_tail_gn = fminf(model->fine_fast_tail_gn, fast_tail_ceiling);
    }
    if (micro_tail > 0.0f) {
        float micro_tail_ceiling = fmaxf(0.12f, micro_tail * 2.50f);
        model->fine_micro_tail_gn = fminf(model->fine_micro_tail_gn, micro_tail_ceiling);
    }

    float fine_sample_conf = (float)model->fine_sample_count /
                             fmaxf(1.0f, (float)AI_TUNING_STAGE_SAMPLE_COUNT);
    float micro_sample_conf = (float)model->fine_recovery_sample_count /
                              fmaxf(1.0f, (float)AI_TUNING_FINE_RECOVERY_SAMPLE_COUNT);
    float machine_conf = model->machine.valid
        ? (float)model->machine.fine_sample_count /
              fmaxf(1.0f, (float)AI_MACHINE_CAL_FINE_SAMPLE_COUNT)
        : 0.0f;
    float uncertainty = model->machine.valid ? model->machine.fine_uncertainty_gn : 0.0f;
    float fast_noise_penalty =
        ai_clampf(uncertainty / fmaxf(model->fine_fast_tail_gn + 0.050f, 0.050f), 0.0f, 1.0f);
    float micro_noise_penalty =
        ai_clampf(uncertainty / fmaxf(model->fine_micro_tail_gn + 0.030f, 0.030f), 0.0f, 1.0f);

    model->fine_fast_tail_confidence =
        ai_clampf(0.12f + fine_sample_conf * 0.48f + machine_conf * 0.25f -
                      fast_noise_penalty * 0.20f,
                  0.0f,
                  1.0f);
    model->fine_micro_tail_confidence =
        ai_clampf(0.10f + micro_sample_conf * 0.55f + machine_conf * 0.20f -
                      micro_noise_penalty * 0.18f,
                  0.0f,
                  1.0f);

    if (model->fine_micro_tail_gn <= 0.035f &&
        model->fine_micro_flow_gps <= 0.045f &&
        model->fine_micro_tail_confidence >= 0.35f) {
        model->fine_tube_profile = AI_FINE_TUBE_PROFILE_LOW_FLOW_LOW_TAIL;
    }
    else if (model->fine_micro_tail_gn >= 0.140f &&
             model->fine_micro_tail_confidence >= 0.35f) {
        model->fine_tube_profile = AI_FINE_TUBE_PROFILE_HIGH_TAIL;
    }
    else if (model->fine_sample_count > 0 || model->fine_recovery_sample_count > 0) {
        model->fine_tube_profile = AI_FINE_TUBE_PROFILE_BALANCED;
    }
    else {
        model->fine_tube_profile = AI_FINE_TUBE_PROFILE_UNKNOWN;
    }

    float confidence = fmaxf(model->fine_fast_tail_confidence,
                             model->fine_micro_tail_confidence);
    float learned_safety_bias =
        ai_clampf((1.0f - confidence) * 0.055f + 0.010f,
                  0.005f,
                  0.090f);
    if (model->fine_tube_profile == AI_FINE_TUBE_PROFILE_HIGH_TAIL) {
        learned_safety_bias = ai_clampf(learned_safety_bias + 0.020f, 0.005f, 0.110f);
    }
    else if (model->fine_tube_profile == AI_FINE_TUBE_PROFILE_LOW_FLOW_LOW_TAIL) {
        learned_safety_bias = ai_clampf(learned_safety_bias - 0.010f, 0.004f, 0.080f);
    }
    if (!isfinite(model->fine_stop_safety_bias_gn) ||
        model->fine_stop_safety_bias_gn <= 0.0f) {
        model->fine_stop_safety_bias_gn = learned_safety_bias;
    }
    else if (learned_safety_bias > model->fine_stop_safety_bias_gn) {
        model->fine_stop_safety_bias_gn =
            model->fine_stop_safety_bias_gn * 0.85f + learned_safety_bias * 0.15f;
    }
    else if (model->fine_stop_safety_bias_gn > learned_safety_bias + 0.015f) {
        model->fine_stop_safety_bias_gn =
            model->fine_stop_safety_bias_gn * 0.75f + learned_safety_bias * 0.25f;
    }
    else {
        model->fine_stop_safety_bias_gn =
            model->fine_stop_safety_bias_gn * 0.98f + learned_safety_bias * 0.02f;
    }
    model->fine_stop_safety_bias_gn =
        ai_clampf(model->fine_stop_safety_bias_gn, 0.004f, 0.110f);

    ai_tuning_clamp_steering_unlocked(model);
}

static void ai_tuning_update_machine_calibration_unlocked(const ai_drop_telemetry_t* telemetry) {
    if (telemetry == NULL) {
        return;
    }

    ai_machine_calibration_t* cal = &g_session.working_model.machine;
    const bool coarse = telemetry->motor_mode == AI_MOTOR_MODE_COARSE_ONLY;
    uint8_t count = coarse ? (uint8_t)(cal->coarse_sample_count + 1u)
                           : (uint8_t)(cal->fine_sample_count + 1u);
    float delivered = fmaxf(0.0f, telemetry->final_weight - telemetry->start_weight);
    bool stop_snapshot_valid = ai_tuning_has_valid_stop_snapshot(telemetry);
    float tail = stop_snapshot_valid
        ? fmaxf(0.0f, telemetry->final_weight - telemetry->stop_weight)
        : 0.0f;
    float flow = telemetry->motor_on_time_ms > 50.0f
        ? delivered / (telemetry->motor_on_time_ms / 1000.0f)
        : 0.0f;

    if (isfinite(telemetry->scale_sample_period_ms) &&
        telemetry->scale_sample_period_ms > 20.0f &&
        telemetry->scale_sample_period_ms < 1500.0f) {
        uint8_t total_count = (uint8_t)(cal->coarse_sample_count + cal->fine_sample_count + 1u);
        ai_tuning_cal_update_mean(&cal->scale_sample_period_ms,
                                  total_count,
                                  telemetry->scale_sample_period_ms);
    }

    if (coarse) {
        cal->coarse_sample_count = count;
        ai_tuning_cal_update_mean(&cal->coarse_first_response_ms,
                                  count,
                                  telemetry->first_response_time_ms);
        ai_tuning_cal_update_mean(&cal->coarse_settle_ms,
                                  count,
                                  telemetry->settle_time_ms);
        if (stop_snapshot_valid) {
            ai_tuning_cal_update_mean(&cal->coarse_tail_avg_gn, count, tail);
            ai_tuning_cal_update_max(&cal->coarse_tail_p95_gn, tail);
        }

        bool looks_like_trim =
            g_session.working_model.coarse_trim_speed_rps > 0.0f &&
            telemetry->speed_rps <= g_session.working_model.coarse_trim_speed_rps + 0.05f;
        if (flow > 0.05f && flow < 220.0f) {
            if (looks_like_trim) {
                ai_tuning_cal_update_mean(&cal->trim_open_loop_flow_gps, count, flow);
            }
            else {
                ai_tuning_cal_update_mean(&cal->coarse_open_loop_flow_gps, count, flow);
            }
        }
        if (stop_snapshot_valid) {
            cal->coarse_uncertainty_gn =
                fmaxf(cal->coarse_uncertainty_gn * 0.80f,
                      fabsf(tail - cal->coarse_tail_avg_gn));
        }
    }
    else {
        cal->fine_sample_count = count;
        ai_tuning_cal_update_mean(&cal->fine_first_response_ms,
                                  count,
                                  telemetry->first_response_time_ms);
        ai_tuning_cal_update_mean(&cal->fine_settle_ms,
                                  count,
                                  telemetry->settle_time_ms);
        if (stop_snapshot_valid) {
            ai_tuning_cal_update_mean(&cal->fine_tail_avg_gn, count, tail);
            ai_tuning_cal_update_max(&cal->fine_tail_p95_gn, tail);
        }

        bool looks_like_micro =
            g_session.working_model.fine_recovery_speed_rps > 0.0f &&
            telemetry->speed_rps <= g_session.working_model.fine_recovery_speed_rps + 0.04f;
        if (flow > 0.001f && flow < 8.0f) {
            if (looks_like_micro) {
                ai_tuning_cal_update_mean(&cal->micro_open_loop_flow_gps, count, flow);
            }
            else {
                ai_tuning_cal_update_mean(&cal->fine_open_loop_flow_gps, count, flow);
            }
        }
        if (stop_snapshot_valid) {
            cal->fine_uncertainty_gn =
                fmaxf(cal->fine_uncertainty_gn * 0.80f,
                      fabsf(tail - cal->fine_tail_avg_gn));
        }
    }

    ai_tuning_sanitize_machine_calibration(&g_session.working_model,
                                           fmaxf(g_session.requested_target_weight, 40.0f));
}

static void ai_tuning_filter_runtime_observations_unlocked(void) {
    ai_runtime_observation_t filtered[AI_RUNTIME_OBSERVATION_COUNT];
    memset(filtered, 0, sizeof(filtered));

    uint8_t filtered_count = 0;
    int oldest_idx = (g_history.observation_count >= AI_RUNTIME_OBSERVATION_COUNT)
        ? g_history.observation_next_idx
        : 0;

    for (uint8_t offset = 0; offset < g_history.observation_count; offset++) {
        int idx = (oldest_idx + offset) % AI_RUNTIME_OBSERVATION_COUNT;
        const ai_runtime_observation_t* obs = &g_history.observations[idx];
        if (obs->profile_idx >= MAX_PROFILE_CNT ||
            !isfinite(obs->target_weight) ||
            !isfinite(obs->final_error_gn) ||
            !isfinite(obs->total_time_ms) ||
            obs->target_weight <= 0.0f ||
            obs->total_time_ms < 0.0f) {
            continue;
        }

        float telemetry_sanity_band = fmaxf(35.0f, obs->target_weight * 1.25f);
        if (fabsf(obs->final_error_gn) > telemetry_sanity_band) {
            continue;
        }

        filtered[filtered_count++] = *obs;
        if (filtered_count >= AI_RUNTIME_OBSERVATION_COUNT) {
            break;
        }
    }

    memset(g_history.observations, 0, sizeof(g_history.observations));
    memcpy(g_history.observations, filtered, sizeof(filtered));
    g_history.observation_count = filtered_count;
    g_history.observation_next_idx = filtered_count % AI_RUNTIME_OBSERVATION_COUNT;
}

static void ai_tuning_sanitize_model_unlocked(ai_profile_model_t* model,
                                              float target_weight,
                                              bool preserve_enabled) {
    if (model == NULL) {
        return;
    }

    bool keep_enabled = preserve_enabled && model->enabled;
    model->coarse_sample_count = (uint8_t)ai_clampf((float)model->coarse_sample_count,
                                                    0.0f,
                                                    AI_TUNING_STAGE_SAMPLE_COUNT);
    model->fine_sample_count = (uint8_t)ai_clampf((float)model->fine_sample_count,
                                                  0.0f,
                                                  AI_TUNING_STAGE_SAMPLE_COUNT);
    model->fine_recovery_sample_count =
        (uint8_t)ai_clampf((float)model->fine_recovery_sample_count,
                           0.0f,
                           AI_TUNING_FINE_RECOVERY_SAMPLE_COUNT);

    float production_coarse_tail = isfinite(model->coarse_tail_gn) &&
                                   model->coarse_tail_gn > 0.0f
        ? model->coarse_tail_gn
        : 0.0f;
    float production_trim_tail = isfinite(model->coarse_trim_tail_gn) &&
                                 model->coarse_trim_tail_gn > 0.0f
        ? model->coarse_trim_tail_gn
        : 0.0f;
    float production_fine_tail = isfinite(model->fine_tail_gn) &&
                                 model->fine_tail_gn > 0.0f
        ? model->fine_tail_gn
        : 0.0f;
    float production_recovery_speed = isfinite(model->fine_recovery_speed_rps) &&
                                      model->fine_recovery_speed_rps > 0.0f
        ? model->fine_recovery_speed_rps
        : 0.0f;
    float production_recovery_flow = isfinite(model->fine_recovery_flow_gps) &&
                                     model->fine_recovery_flow_gps > 0.0f
        ? model->fine_recovery_flow_gps
        : 0.0f;
    float production_recovery_tail = isfinite(model->fine_recovery_tail_gn) &&
                                     model->fine_recovery_tail_gn >= 0.0f
        ? model->fine_recovery_tail_gn
        : 0.0f;

    if (model->coarse_sample_count > 0) {
        ai_tuning_fit_stage_samples(model->coarse_samples,
                                    model->coarse_sample_count,
                                    &model->coarse_flow_slope,
                                    &model->coarse_flow_intercept,
                                    &model->coarse_tail_gn);
        model->coarse_tail_gn = ai_tuning_estimate_coarse_tail_guard(model->coarse_samples,
                                                                     model->coarse_sample_count,
                                                                     model->coarse_tail_gn,
                                                                     target_weight);
        if (production_coarse_tail > model->coarse_tail_gn) {
            float tail_cap = fmaxf(8.0f, target_weight * 0.32f);
            model->coarse_tail_gn = ai_clampf(production_coarse_tail,
                                             model->coarse_tail_gn,
                                             tail_cap);
        }
        else if (production_coarse_tail > 0.0f &&
                 production_coarse_tail < model->coarse_tail_gn) {
            model->coarse_tail_gn = ai_clampf(production_coarse_tail,
                                             model->coarse_tail_gn * 0.55f,
                                             model->coarse_tail_gn);
        }
    }
    else {
        model->coarse_flow_slope = 0.0f;
        model->coarse_flow_intercept = 0.0f;
        model->coarse_tail_gn = 0.0f;
    }

    if (model->fine_sample_count > 0) {
        ai_tuning_fit_stage_samples(model->fine_samples,
                                    model->fine_sample_count,
                                    &model->fine_flow_slope,
                                    &model->fine_flow_intercept,
                                    &model->fine_tail_gn);
        model->fine_tail_gn = ai_tuning_estimate_fine_tail_guard(model->fine_samples,
                                                                 model->fine_sample_count,
                                                                 model->fine_tail_gn,
                                                                 target_weight);
        if (production_fine_tail > model->fine_tail_gn) {
            float tail_cap = fmaxf(0.75f, fminf(2.00f, target_weight * 0.05f));
            model->fine_tail_gn = ai_clampf(production_fine_tail,
                                            model->fine_tail_gn,
                                            tail_cap);
        }
    }
    else {
        model->fine_flow_slope = 0.0f;
        model->fine_flow_intercept = 0.0f;
        model->fine_tail_gn = 0.0f;
    }

    ai_tuning_choose_bulk_stage_sample(model->coarse_samples,
                                       model->coarse_sample_count,
                                       &model->coarse_best_speed_rps,
                                       &model->coarse_best_flow_gps);
    ai_tuning_choose_trim_stage_sample(model->coarse_samples,
                                       model->coarse_sample_count,
                                       &model->coarse_trim_speed_rps,
                                       &model->coarse_trim_flow_gps,
                                       &model->coarse_trim_tail_gn);
    if (production_trim_tail > 0.0f &&
        production_trim_tail < model->coarse_trim_tail_gn) {
        model->coarse_trim_tail_gn = ai_clampf(production_trim_tail,
                                               model->coarse_trim_tail_gn * 0.50f,
                                               model->coarse_trim_tail_gn);
    }
    ai_tuning_choose_best_stage_sample(model->fine_samples,
                                       model->fine_sample_count,
                                       fminf(g_config.error_cost_weight, 0.25f),
                                       &model->fine_best_speed_rps,
                                       &model->fine_best_flow_gps);
    ai_tuning_choose_recovery_stage_sample(model->fine_recovery_samples,
                                           model->fine_recovery_sample_count,
                                           &model->fine_recovery_speed_rps,
                                           &model->fine_recovery_flow_gps,
                                           &model->fine_recovery_tail_gn);
    if (production_recovery_speed > 0.0f) {
        float max_recovery_speed = model->fine_best_speed_rps > 0.0f
            ? fminf(0.60f, fmaxf(0.35f, model->fine_best_speed_rps * 0.12f))
            : 0.60f;
        model->fine_recovery_speed_rps =
            ai_clampf(production_recovery_speed, 0.08f, max_recovery_speed);
    }
    if (production_recovery_flow > 0.001f && production_recovery_flow < 0.25f) {
        model->fine_recovery_flow_gps = production_recovery_flow;
    }
    if (production_recovery_tail > 0.0f) {
        model->fine_recovery_tail_gn =
            ai_clampf(production_recovery_tail, 0.0f, 0.18f);
    }
    if (model->fine_recovery_speed_rps <= 0.0f && model->fine_best_speed_rps > 0.0f) {
        model->fine_recovery_speed_rps = ai_clampf(model->fine_best_speed_rps * 0.035f,
                                                   0.08f,
                                                   0.35f);
        model->fine_recovery_flow_gps =
            ai_tuning_predict_flow_from_model(model,
                                              AI_MOTOR_MODE_FINE_ONLY,
                                              model->fine_recovery_speed_rps);
        model->fine_recovery_tail_gn = 0.0f;
    }

    float min_window = ai_tuning_min_fine_window_for_model(model);
    float base_window = fmaxf(min_window,
                              fmaxf(model->fine_best_flow_gps * 0.60f +
                                         fmaxf(model->fine_tail_gn, 0.0f) * 2.0f,
                                    fmaxf(model->coarse_trim_tail_gn, 0.0f) * 0.35f +
                                        g_config.noise_margin * 3.0f));
    if (!isfinite(model->recommended_fine_window_gn) ||
        model->recommended_fine_window_gn <= 0.0f) {
        model->recommended_fine_window_gn = base_window;
    }
    float max_window = fmaxf(1.20f, fminf(1.80f, target_weight * 0.05f));
    model->recommended_fine_window_gn = ai_clampf(model->recommended_fine_window_gn,
                                                  min_window,
                                                  max_window);

    if (!isfinite(model->runtime_bias_gn)) {
        model->runtime_bias_gn = 0.0f;
    }
    float positive_bias_limit = fmaxf(0.18f, fminf(0.30f, target_weight * 0.006f));
    model->runtime_bias_gn = ai_clampf(model->runtime_bias_gn,
                                       -0.08f,
                                       positive_bias_limit);

    ai_tuning_sanitize_machine_calibration(model, target_weight);
    if (model->machine.valid) {
        float machine_tail_guard =
            ai_tuning_machine_coarse_tail_guard(&model->machine, target_weight);
        if (machine_tail_guard > 0.0f) {
            model->coarse_tail_gn = fmaxf(model->coarse_tail_gn,
                                          machine_tail_guard * 0.92f);
            model->coarse_trim_tail_gn = fmaxf(model->coarse_trim_tail_gn,
                                               machine_tail_guard * 0.55f);
        }
    }
    ai_tuning_update_finish_profile_unlocked(model, target_weight);

    model->valid =
        model->coarse_sample_count > 0 &&
        model->fine_sample_count > 0 &&
        model->coarse_best_speed_rps > 0.0f &&
        model->coarse_trim_speed_rps > 0.0f &&
        model->coarse_trim_flow_gps > g_config.noise_margin * 0.25f &&
        model->fine_best_speed_rps > 0.0f &&
        model->fine_best_flow_gps > g_config.noise_margin * 0.25f;
    model->enabled = model->valid ? keep_enabled : false;
}

static void ai_tuning_finalize_working_model_unlocked(void) {
    g_session.working_model.runtime_bias_gn = 0.0f;
    ai_tuning_sanitize_model_unlocked(&g_session.working_model,
                                      fmaxf(g_session.requested_target_weight, 40.0f),
                                      false);
}

static void ai_tuning_refresh_plan_unlocked(void) {
    if (!ai_tuning_is_active_unlocked() || g_session.target_profile == NULL) {
        return;
    }

    uint8_t sample_limit = ai_tuning_get_stage_sample_limit_unlocked(g_session.state);
    g_session.stage_budget_limit_gn = ai_tuning_get_stage_budget_limit_unlocked(g_session.state);
    if (g_session.stage_sample_index >= sample_limit) {
        return;
    }

    if (g_session.state == AI_TUNING_CALIBRATING_COARSE ||
        g_session.state == AI_TUNING_CALIBRATING_FINE) {
        static const float coarse_on_times_ms[AI_MACHINE_CAL_COARSE_SAMPLE_COUNT] = {
            250.0f, 450.0f, 650.0f, 850.0f,
            250.0f, 450.0f, 650.0f, 850.0f
        };
        static const float fine_on_times_ms[AI_MACHINE_CAL_FINE_SAMPLE_COUNT] = {
            500.0f, 800.0f, 1100.0f, 1400.0f,
            2500.0f, 3500.0f, 5000.0f, 7000.0f
        };

        bool coarse = g_session.state == AI_TUNING_CALIBRATING_COARSE;
        bool low_speed_sample = g_session.stage_sample_index >= 4u;
        float speed_rps = 0.0f;
        float expected_flow = 0.0f;
        float on_time_ms = coarse
            ? coarse_on_times_ms[g_session.stage_sample_index % AI_MACHINE_CAL_COARSE_SAMPLE_COUNT]
            : fine_on_times_ms[g_session.stage_sample_index % AI_MACHINE_CAL_FINE_SAMPLE_COUNT];

        if (coarse) {
            speed_rps = low_speed_sample
                ? g_session.working_model.coarse_trim_speed_rps
                : g_session.working_model.coarse_best_speed_rps;
            expected_flow = low_speed_sample
                ? g_session.working_model.coarse_trim_flow_gps
                : g_session.working_model.coarse_best_flow_gps;
        }
        else {
            speed_rps = low_speed_sample
                ? g_session.working_model.fine_recovery_speed_rps
                : g_session.working_model.fine_best_speed_rps;
            expected_flow = low_speed_sample
                ? g_session.working_model.fine_recovery_flow_gps
                : g_session.working_model.fine_best_flow_gps;
        }

        if (speed_rps <= 0.0f) {
            speed_rps = ai_tuning_interpolate_stage_speed_unlocked(
                coarse ? AI_TUNING_CHARACTERIZING_COARSE : AI_TUNING_CHARACTERIZING_FINE,
                g_session.stage_sample_index);
        }
        if (expected_flow <= 0.0f) {
            expected_flow = ai_tuning_predict_flow_from_model(&g_session.working_model,
                                                              coarse ? AI_MOTOR_MODE_COARSE_ONLY
                                                                     : AI_MOTOR_MODE_FINE_ONLY,
                                                              speed_rps);
        }

        if (expected_flow > 0.0f) {
            float safe_sample_target = coarse
                ? (low_speed_sample ? 3.50f : 6.00f)
                : (low_speed_sample ? 0.12f : 0.75f);
            float safe_on_time_ms = (safe_sample_target / expected_flow) * 1000.0f;
            float min_on_time_ms = coarse
                ? 120.0f
                : (low_speed_sample ? 1200.0f : 250.0f);
            on_time_ms = fminf(on_time_ms,
                               ai_clampf(safe_on_time_ms, min_on_time_ms, on_time_ms));
        }

        float target_weight = expected_flow > 0.0f
            ? expected_flow * (on_time_ms / 1000.0f)
            : (coarse ? 8.0f : 0.60f);
        target_weight = coarse
            ? ai_clampf(target_weight, 2.0f, 35.0f)
            : ai_clampf(target_weight, low_speed_sample ? 0.03f : 0.20f,
                        low_speed_sample ? 0.25f : 2.50f);

        g_session.current_speed_rps = speed_rps;
        g_session.current_motor_on_time_ms = on_time_ms;
        g_session.current_target_weight = target_weight;
        g_session.stage_budget_limit_gn = ai_tuning_get_stage_budget_limit_unlocked(g_session.state);
        snprintf(g_session.status_message, sizeof(g_session.status_message),
                 "Calibrating %s %s timing %u/%u at %.2f rps for %.0f ms",
                 coarse ? "coarse" : "fine",
                 low_speed_sample ? "low-speed" : "production-speed",
                 (unsigned int)(g_session.stage_sample_index + 1u),
                 (unsigned int)sample_limit,
                 speed_rps,
                 on_time_ms);
        return;
    }

    float remaining_budget = fmaxf(g_session.stage_budget_limit_gn - g_session.stage_budget_used_gn, 0.0f);
    uint8_t remaining_samples = (sample_limit > g_session.stage_sample_index)
        ? (uint8_t)(sample_limit - g_session.stage_sample_index)
        : 1u;
    float configured_target = (g_session.state == AI_TUNING_CHARACTERIZING_FINE)
        ? g_config.fine_sample_target_gn
        : g_config.coarse_sample_target_gn;
    float target_weight = fminf(configured_target, remaining_budget / remaining_samples);
    float minimum_target = (g_session.state == AI_TUNING_CHARACTERIZING_FINE)
        ? fmaxf(0.5f, g_config.noise_margin * 6.0f)
        : fmaxf(3.0f, g_config.noise_margin * 20.0f);
    target_weight = fmaxf(target_weight, minimum_target);

    float speed_rps = ai_tuning_interpolate_stage_speed_unlocked(g_session.state, g_session.stage_sample_index);
    float on_time_ms = ai_tuning_estimate_on_time_ms_unlocked(g_session.state, speed_rps, target_weight);
    if (g_session.state == AI_TUNING_CHARACTERIZING_COARSE) {
        float max_on_time_ms =
            ai_tuning_coarse_characterization_max_on_time_ms(g_session.stage_sample_index);
        on_time_ms = fminf(on_time_ms, max_on_time_ms);
        float expected_flow = ai_tuning_estimate_flow_unlocked(g_session.state, speed_rps);
        if (expected_flow > 0.0f) {
            target_weight = fmaxf(g_config.noise_margin * 4.0f,
                                  fminf(target_weight,
                                        expected_flow * (on_time_ms / 1000.0f)));
        }
    }
    if (ai_tuning_is_fine_recovery_sample_unlocked(g_session.state, g_session.stage_sample_index)) {
        target_weight = fmaxf(0.10f, g_config.noise_margin * 2.0f);
        target_weight = fminf(target_weight, remaining_budget);
        on_time_ms = ai_tuning_fine_recovery_on_time_ms_for_sample(g_session.stage_sample_index);
    }

    g_session.current_speed_rps = speed_rps;
    g_session.current_motor_on_time_ms = on_time_ms;
    g_session.current_target_weight = target_weight;

    const char* stage_name = (g_session.state == AI_TUNING_CHARACTERIZING_FINE) ? "fine" : "coarse";
    snprintf(g_session.status_message, sizeof(g_session.status_message),
             "Characterizing %s sample %u/%u at %.2f rps for %.0f ms (target %.2f gn)",
             stage_name,
             (unsigned int)(g_session.stage_sample_index + 1u),
             (unsigned int)sample_limit,
             speed_rps,
             on_time_ms,
             target_weight);
}
