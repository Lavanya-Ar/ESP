/** @file PID_Line_Follow.c
 *  @brief PID-based line following control logic (proportional + derivative;
 *         optional integral disabled by default).
 *
 *  NOTE: Empirical constants tuned for current IR sensor; revisit if hardware
 *        changes.
 *  WARNING: White ramp behavior depends on IR RAW scale (0=black, 4095=white).
 *
 *  Derived from original project code; Barr-C style applied (no public name
 *  changes).
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "motor_encoder_demo.h"
#include "ir_sensor.h"
#include "PID_Line_Follow.h"

/* ==============================
 * Configuration Constants
 * ============================== */
#define PID_SETPOINT_RAW        (700U)
#define PID_KP                  (0.019f)
#define PID_KD                  (0.006f)
#define PID_KI                  (0.001f)    /* Integral optional */
#define PID_BASE_SPEED_LEFT     (32.75f)
#define PID_BASE_SPEED_RIGHT    (30.0f)
#define PID_MAX_CORRECTION      (17.5f)
#define PID_WHITE_THRESHOLD     (300U)
#define PID_WHITE_RAMP_RATE     (0.125f)
#define PID_MAX_INTEGRAL        (1000.0f)
#define PID_MAX_DERIVATIVE      (1000.0f)
#define PID_PRINT_INTERVAL_US   (100000U)   /* 100 ms */

/* ==============================
 * Static State
 * ============================== */
static int16_t  g_prev_error       = 0;
static uint32_t g_prev_time_us     = 0;
static float    g_prev_correction  = 0.0f;
static uint16_t g_prev_ir_raw      = PID_SETPOINT_RAW;

/* ==============================
 * Private Prototypes
 * ============================== */
static float pid_compute_(uint16_t ir_raw);
static void  pid_drive_(float correction, uint16_t ir_raw);

/* ==============================
 * PID Compute
 * ============================== */
static float pid_compute_(uint16_t ir_raw)
{
    uint32_t now_us = time_us_32();
    float dt_s = (g_prev_time_us == 0) ? 0.01f :
                 ((now_us - g_prev_time_us) / 1000000.0f);

    int16_t error = (int16_t)PID_SETPOINT_RAW - (int16_t)ir_raw;
    float derivative = (dt_s > 0.0f) ? ((error - g_prev_error) / dt_s) : 0.0f;

    if (derivative > PID_MAX_DERIVATIVE)  derivative = PID_MAX_DERIVATIVE;
    if (derivative < -PID_MAX_DERIVATIVE) derivative = -PID_MAX_DERIVATIVE;

    static float integral_accum = 0.0f; /* NOTE: integral disabled; keep for future */
    /* #ifdef PID_ENABLE_INTEGRAL */
    /* integral_accum += error * dt_s; */
    /* if (integral_accum > PID_MAX_INTEGRAL)  integral_accum = PID_MAX_INTEGRAL; */
    /* if (integral_accum < -PID_MAX_INTEGRAL) integral_accum = -PID_MAX_INTEGRAL; */
    /* #endif */

    float correction = (PID_KP * (float)error) + (PID_KD * derivative);
    /* + (PID_KI * integral_accum) if integral enabled */

    if (correction > PID_MAX_CORRECTION)  correction = PID_MAX_CORRECTION;
    if (correction < -PID_MAX_CORRECTION) correction = -PID_MAX_CORRECTION;

    if ((ir_raw > PID_SETPOINT_RAW) && (g_prev_ir_raw < PID_SETPOINT_RAW)) {
        correction = g_prev_correction; /* preserve previous during crossing */
    }

    g_prev_error      = error;
    g_prev_time_us    = now_us;
    g_prev_correction = correction;
    g_prev_ir_raw     = ir_raw;

    return correction;
}

/* ==============================
 * Motor Drive
 * ============================== */
static void pid_drive_(float correction, uint16_t ir_raw)
{
    static float white_ramp = 0.0f;

    if (ir_raw < PID_WHITE_THRESHOLD) {
        white_ramp += PID_WHITE_RAMP_RATE;
        if (white_ramp > 1.0f) {
            white_ramp = 1.0f;
        }
        correction *= white_ramp;
    } else {
        white_ramp = 0.0f;
    }

    float left_speed  = PID_BASE_SPEED_LEFT  - correction;
    float right_speed = PID_BASE_SPEED_RIGHT + correction;

    if (left_speed  < 0.0f) left_speed  = 0.0f;
    if (right_speed < 0.0f) right_speed = 0.0f;
    if (left_speed  > 80.0f) left_speed  = 80.0f;
    if (right_speed > 80.0f) right_speed = 80.0f;

    static uint32_t last_print_us = 0;
    uint32_t now_us = time_us_32();
    if ((now_us - last_print_us) > PID_PRINT_INTERVAL_US) {
        printf("[PID] IR:%4u E:%4d C:%6.2f L:%5.2f R:%5.2f W:%4.2f\n",
               ir_raw,
               (int)(PID_SETPOINT_RAW - ir_raw),
               correction,
               left_speed,
               right_speed,
               white_ramp);
        last_print_us = now_us;
    }

    drive_signed(left_speed, right_speed);
}

/* ==============================
 * Public API (Names Preserved)
 * ============================== */
void follow_line_simple(void)
{
    uint16_t ir_raw = ir_read_raw();
    float correction = pid_compute_(ir_raw);
    pid_drive_(correction, ir_raw);
}

void follow_line_simple_with_params(float base_left_speed,
                                    float base_right_speed,
                                    float max_correction)
{
    /* NOTE: Parameterized variant kept for backward compatibility. */
    uint16_t ir_raw = ir_read_raw();
    uint32_t now_us = time_us_32();
    float dt_s = (g_prev_time_us == 0) ? 0.01f :
                 ((now_us - g_prev_time_us) / 1000000.0f);

    int16_t error = (int16_t)PID_SETPOINT_RAW - (int16_t)ir_raw;
    float derivative = (dt_s > 0.0f) ? ((error - g_prev_error) / dt_s) : 0.0f;

    if (derivative > PID_MAX_DERIVATIVE)  derivative = PID_MAX_DERIVATIVE;
    if (derivative < -PID_MAX_DERIVATIVE) derivative = -PID_MAX_DERIVATIVE;

    float correction = (PID_KP * (float)error) + (PID_KD * derivative);

    if (correction > max_correction)  correction = max_correction;
    if (correction < -max_correction) correction = -max_correction;

    if ((ir_raw > PID_SETPOINT_RAW) && (g_prev_ir_raw < PID_SETPOINT_RAW)) {
        correction = g_prev_correction;
    }

    static float white_ramp = 0.0f;
    if (ir_raw < PID_WHITE_THRESHOLD) {
        white_ramp += PID_WHITE_RAMP_RATE;
        if (white_ramp > 1.0f) white_ramp = 1.0f;
        correction *= white_ramp;
    } else {
        white_ramp = 0.0f;
    }

    float left_speed  = base_left_speed  - correction;
    float right_speed = base_right_speed + correction;

    if (left_speed  < 0.0f) left_speed  = 0.0f;
    if (right_speed < 0.0f) right_speed = 0.0f;
    if (left_speed  > 80.0f) left_speed  = 80.0f;
    if (right_speed > 80.0f) right_speed = 80.0f;

    drive_signed(left_speed, right_speed);

    g_prev_error      = error;
    g_prev_time_us    = now_us;
    g_prev_correction = correction;
    g_prev_ir_raw     = ir_raw;
}

/*** end of file ***/
