#include <FreeRTOS.h>
#include <queue.h>
#include <stdlib.h>
#include <semphr.h>
#include <time.h>
#include <math.h>
#include <task.h>
#include <string.h>

#include "hardware/uart.h"
#include "configuration.h"
#include "scale.h"
#include "app.h"


#define AND_SCALE_LINE_BUFFER_LEN 32


// Forward declaration
void _and_scale_listener_task(void *p);
void scale_press_re_zero_key();

extern scale_config_t scale_config;

// Instance of the scale handle for A&D FXi series
scale_handle_t and_fxi_scale_handle = {
    .read_loop_task = _and_scale_listener_task,
    .force_zero = scale_press_re_zero_key,
};


static float _decode_measurement_line(const char *line) {
    if (line == NULL || strlen(line) < 5) {
        return NAN;
    }

    // A&D standard output is normally "ST,+0000.00 g" or "US,+0000.00 g".
    // Requiring the comma at byte 2 lets us resync cleanly after any UART noise.
    if (line[2] != ',') {
        return NAN;
    }

    if (line[0] == 'O' && line[1] == 'L') {
        return NAN;
    }

    char *endptr;
    float weight = strtof(&line[3], &endptr);

    if (endptr == &line[3] || !isfinite(weight)) {
        return NAN;
    }

    return weight;
}


void _and_scale_listener_task(void *p) {
    uint8_t line_buf_idx = 0;
    char line_buffer[AND_SCALE_LINE_BUFFER_LEN];

    while (true) {
        // Read all data 
        while (uart_is_readable(SCALE_UART)) {
            char ch = uart_getc(SCALE_UART);

            if (ch == '\r') {
                continue;
            }

            if (ch == '\n') {
                if (line_buf_idx > 0) {
                    line_buffer[line_buf_idx] = '\0';
                    scale_config.current_scale_measurement = _decode_measurement_line(line_buffer);

                    if (scale_config.scale_measurement_ready) {
                        xSemaphoreGive(scale_config.scale_measurement_ready);
                    }
                }

                line_buf_idx = 0;
                continue;
            }

            if (line_buf_idx < AND_SCALE_LINE_BUFFER_LEN - 1) {
                line_buffer[line_buf_idx++] = ch;
            }
            else {
                line_buf_idx = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


void scale_press_re_zero_key() {
    char cmd[] = "Z\r\n";
    scale_write(cmd, strlen(cmd));
}

void scale_press_print_key() {
    char cmd[] = "PRT\r\n";
    scale_write(cmd, strlen(cmd));
}

void scale_press_sample_key() {
    char cmd[] = "SMP\r\n";
    scale_write(cmd, strlen(cmd));
}

void scale_press_mode_key() {
    char cmd[] = "U\r\n";
    scale_write(cmd, strlen(cmd));
}

void scale_press_cal_key() {
    char cmd[] = "CAL\r\n";
    scale_write(cmd, strlen(cmd));
}

void scale_press_on_off_key() {
    char cmd[] = "P\r\n";
    scale_write(cmd, strlen(cmd));
}

void scale_display_off() {
    char cmd[] = "OFF\r\n";
    scale_write(cmd, strlen(cmd));
}

void scale_display_on() {
    char cmd[] = "ON\r\n";
    scale_write(cmd, strlen(cmd));
}



// AppState_t scale_enable_fast_report(AppState_t prev_state) {
//     // TODO: Finish this
// }
