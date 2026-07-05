#ifndef AI_TUNING_H_
#define AI_TUNING_H_

#include <stdint.h>
#include <stdbool.h>
#include "profile.h"

#define AI_TUNING_HISTORY_REV 14
#define AI_TUNING_CONFIG_REV 6
#define AI_TUNING_DROP_BUF_SIZE 16
#define AI_TUNING_STAGE_SAMPLE_COUNT 12
#define AI_TUNING_FINE_RECOVERY_SAMPLE_COUNT 4
#define AI_RUNTIME_OBSERVATION_COUNT 24
#define AI_MACHINE_CAL_COARSE_SAMPLE_COUNT 8
#define AI_MACHINE_CAL_FINE_SAMPLE_COUNT 8
#define AI_MACHINE_CAL_MIN_VALID_SAMPLE_COUNT 3

typedef enum {
    AI_TUNING_IDLE = 0,
    AI_TUNING_CHARACTERIZING_COARSE,
    AI_TUNING_CHARACTERIZING_FINE,
    AI_TUNING_CALIBRATING_COARSE,
    AI_TUNING_CALIBRATING_FINE,
    AI_TUNING_READY_TO_SAVE,
    AI_TUNING_ERROR
} ai_tuning_state_t;

typedef enum {
    AI_MOTOR_MODE_NORMAL = 0,
    AI_MOTOR_MODE_COARSE_ONLY,
    AI_MOTOR_MODE_FINE_ONLY
} ai_motor_mode_t;

typedef enum {
    AI_FINE_TUBE_PROFILE_UNKNOWN = 0,
    AI_FINE_TUBE_PROFILE_LOW_FLOW_LOW_TAIL,
    AI_FINE_TUBE_PROFILE_BALANCED,
    AI_FINE_TUBE_PROFILE_HIGH_TAIL
} ai_fine_tube_profile_t;

typedef enum {
    AI_STEERING_FASTER = 0,
    AI_STEERING_SAFER,
    AI_STEERING_FINE_FINISH_FASTER,
    AI_STEERING_BULK_CLOSER,
    AI_STEERING_UNDO_LAST
} ai_steering_action_t;

typedef struct {
    uint8_t sample_number;
    ai_motor_mode_t motor_mode;
    float speed_rps;
    float motor_on_time_ms;
    float start_weight;
    float stop_weight;
    float final_weight;
    float delivered_weight;
    float tail_weight;
    float target_weight;
    float overthrow;
    float coarse_time_ms;
    float fine_time_ms;
    float total_time_ms;
    float first_response_time_ms;
    float settle_time_ms;
    float scale_sample_period_ms;
} ai_drop_telemetry_t;

typedef struct {
    float speed_rps;
    float motor_on_time_ms;
    float delivered_weight;
    float tail_weight;
    float flow_gps;
} ai_flow_sample_t;

typedef struct {
    bool valid;
    uint8_t coarse_sample_count;
    uint8_t fine_sample_count;
    float scale_sample_period_ms;

    float coarse_first_response_ms;
    float coarse_settle_ms;
    float coarse_tail_avg_gn;
    float coarse_tail_p95_gn;
    float coarse_uncertainty_gn;
    float coarse_open_loop_flow_gps;
    float trim_open_loop_flow_gps;

    float fine_first_response_ms;
    float fine_settle_ms;
    float fine_tail_avg_gn;
    float fine_tail_p95_gn;
    float fine_uncertainty_gn;
    float fine_open_loop_flow_gps;
    float micro_open_loop_flow_gps;

    float recommended_bulk_handoff_gn;
    float recommended_trim_stop_gn;
    float post_finish_watch_ms;
} ai_machine_calibration_t;

typedef struct {
    bool valid;
    bool enabled;
    uint8_t coarse_sample_count;
    uint8_t fine_sample_count;
    uint8_t fine_recovery_sample_count;

    float coarse_flow_slope;
    float coarse_flow_intercept;
    float coarse_tail_gn;
    float coarse_best_speed_rps;
    float coarse_best_flow_gps;
    float coarse_trim_speed_rps;
    float coarse_trim_flow_gps;
    float coarse_trim_tail_gn;

    float fine_flow_slope;
    float fine_flow_intercept;
    float fine_tail_gn;
    float fine_best_speed_rps;
    float fine_best_flow_gps;
    float fine_recovery_speed_rps;
    float fine_recovery_flow_gps;
    float fine_recovery_tail_gn;

    float recommended_fine_window_gn;
    float runtime_bias_gn;
    float fine_fast_flow_gps;
    float fine_fast_tail_gn;
    float fine_fast_tail_confidence;
    float fine_micro_flow_gps;
    float fine_micro_tail_gn;
    float fine_micro_tail_confidence;
    float fine_stop_safety_bias_gn;
    float steering_bulk_bias_gn;
    float steering_fine_bias_gn;
    float steering_recovery_speed_scale;
    float steering_last_bulk_bias_gn;
    float steering_last_fine_bias_gn;
    float steering_last_recovery_speed_scale;
    uint8_t fine_tube_profile;
    uint8_t steering_undo_available;
    uint16_t steering_count;
    ai_machine_calibration_t machine;

    ai_flow_sample_t coarse_samples[AI_TUNING_STAGE_SAMPLE_COUNT];
    ai_flow_sample_t fine_samples[AI_TUNING_STAGE_SAMPLE_COUNT];
    ai_flow_sample_t fine_recovery_samples[AI_TUNING_FINE_RECOVERY_SAMPLE_COUNT];
} ai_profile_model_t;

typedef struct {
    float target_weight;
    float final_error_gn;
    float total_time_ms;
    float coarse_stop_weight_gn;
    float after_coarse_settle_gn;
    float observed_coarse_tail_gn;
    float fine_stop_weight_gn;
    float after_fine_settle_gn;
    float observed_fine_tail_gn;
    float post_finish_peak_weight_gn;
    float recovery_start_weight_gn;
    float recovery_end_weight_gn;
    float recovery_motor_on_ms;
    uint16_t recovery_stall_count;
    uint8_t recovery_exit_reason;
    uint8_t profile_idx;
} ai_runtime_observation_t;

typedef struct {
    ai_tuning_state_t state;
    profile_t* target_profile;
    uint8_t target_profile_idx;
    uint8_t drops_completed;
    uint8_t total_samples_planned;
    uint8_t stage_sample_index;

    float requested_target_weight;
    float stage_budget_used_gn;
    float stage_budget_limit_gn;
    float current_speed_rps;
    float current_motor_on_time_ms;
    float current_target_weight;
    float avg_final_error_gn;
    float avg_total_time_ms;

    char status_message[96];
    char error_message[96];

    ai_drop_telemetry_t drops[AI_TUNING_DROP_BUF_SIZE];
    uint8_t drop_write_idx;

    ai_profile_model_t working_model;
} ai_tuning_session_t;

typedef struct {
    float coarse_budget_gn;
    float fine_budget_gn;
    uint8_t coarse_sample_count;
    uint8_t fine_sample_count;
    float coarse_sample_target_gn;
    float fine_sample_target_gn;
    float noise_margin;
    float time_cost_weight;
    float error_cost_weight;
} ai_tuning_config_t;

typedef struct {
    uint16_t revision;
    float coarse_budget_gn;
    float fine_budget_gn;
    uint8_t coarse_sample_count;
    uint8_t fine_sample_count;
    float coarse_sample_target_gn;
    float fine_sample_target_gn;
    float noise_margin;
    float time_cost_weight;
    float error_cost_weight;
} __attribute__((packed)) ai_tuning_config_eeprom_t;

typedef struct {
    uint16_t revision;
    ai_profile_model_t models[MAX_PROFILE_CNT];
    uint8_t observation_count;
    uint8_t observation_next_idx;
    ai_runtime_observation_t observations[AI_RUNTIME_OBSERVATION_COUNT];
} ai_tuning_history_t;

typedef struct {
    bool valid;
    ai_motor_mode_t motor_mode;
    float speed_rps;
    float motor_on_time_ms;
    float target_weight;
    float stage_budget_used_gn;
    float stage_budget_limit_gn;
    uint8_t sample_index;
    uint8_t total_samples;
    char description[96];
} ai_tuning_plan_t;

#ifdef __cplusplus
extern "C" {
#endif

void ai_tuning_init(void);
ai_tuning_config_t* ai_tuning_get_config(void);

bool ai_tuning_start(profile_t* profile, float target_weight);
bool ai_tuning_start_machine_calibration(profile_t* profile, float target_weight);
bool ai_tuning_record_drop(const ai_drop_telemetry_t* telemetry);
bool ai_tuning_get_next_params(float* coarse_a, float* coarse_b, float* fine_a, float* fine_b);
bool ai_tuning_is_complete(void);
bool ai_tuning_get_session_copy(ai_tuning_session_t* out);
bool ai_tuning_get_history_copy(ai_tuning_history_t* out);
bool ai_tuning_get_recommended_params(float* coarse_a, float* coarse_b, float* fine_a, float* fine_b);
bool ai_tuning_apply_params(void);
bool ai_tuning_cancel(void);
bool ai_tuning_is_active(void);
ai_motor_mode_t ai_tuning_get_motor_mode(void);
uint8_t ai_tuning_get_progress_percent(void);
bool ai_tuning_get_active_plan(ai_tuning_plan_t* out);
bool ai_tuning_get_profile_model_copy(uint8_t profile_idx, ai_profile_model_t* out);
bool ai_tuning_get_enabled_model_copy(uint8_t profile_idx, ai_profile_model_t* out);
bool ai_tuning_apply_steering(uint8_t profile_idx, ai_steering_action_t action);
const char* ai_tuning_steering_action_to_string(ai_steering_action_t action);
const char* ai_tuning_fine_tube_profile_to_string(ai_fine_tube_profile_t profile);

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
                             uint8_t recovery_exit_reason);
void ai_tuning_calculate_refinements(uint8_t profile_idx);
bool ai_tuning_get_refined_params(float* coarse_a, float* coarse_b, float* fine_a, float* fine_b);
bool ai_tuning_get_suggestions(uint8_t profile_idx, float* coarse_a, float* coarse_b,
                               float* fine_a, float* fine_b);
bool ai_tuning_apply_refined_params(uint8_t profile_idx);
void ai_tuning_clear_history(void);
void ai_tuning_save_config(void);
float ai_tuning_get_scale_compensation(void);

#ifdef __cplusplus
}
#endif

#endif  // AI_TUNING_H_
