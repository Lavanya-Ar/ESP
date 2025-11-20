/** @file Obstacle_Avoidance.c
 *  @brief Encoder-based obstacle detection and avoidance routines with servo scan.
 *
 *  NOTE: Refactored for Barr-C style, condensed telemetry, removed repetitive prints.
 *  WARNING: TURN_*_DEG_PULSES empirical; recalibrate if wheel size or encoder resolution changes.
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#include "Obstacle_Avoidance.h"
#include "ultrasonic.h"
#include "motor_encoder_demo.h"
#include "ir_sensor.h"
#include "PID_Line_Follow.h"
#include "mqtt_client.h"
#include "encoder.h"
#include "imu_raw_demo.h"

/* ==============================
 * Configuration Constants
 * ============================== */
#define TURN_90_DEG_PULSES     (9u)
#define TURN_180_DEG_PULSES    (16u)
#define TURN_45_DEG_PULSES     (4u)
#define CIRCLE_TARGET_PULSES   (12u)

#define SERVO_MIN_PULSE_US     (500.0f)
#define SERVO_MAX_PULSE_US     (2500.0f)
#define SERVO_PWM_WRAP         (20000u)
#define SERVO_PWM_CLKDIV       (125.0f)
#define SERVO_CENTER_DEG       (85.0f)

#define OBSTACLE_DETECTION_DISTANCE_CM (50.0f)
#define SAFE_DISTANCE_CM               (25.0f)
#define INITIAL_STOP_DISTANCE_CM       (15.0f)

#define OBJECT_SCAN_STEP_DEG     (1.0f)
#define OBJECT_SCAN_LEFT_START   (150.0f)
#define OBJECT_SCAN_RIGHT_END    (30.0f)
#define OBJECT_WIDTH_THRESHOLD_CM (30.0f)

/* ==============================
 * Static Telemetry Timing
 * ============================== */
static volatile uint32_t g_last_pub_ms = 0;

/* ==============================
 * Speed & Distance State
 * ============================== */
static volatile uint32_t g_last_speed_calc_time = 0;
static volatile float    g_current_speed_cm_s   = 0.0f;
static volatile float    g_last_distance_cm     = 0.0f;

/* ==============================
 * Static Prototypes
 * ============================== */
static uint32_t get_average_pulses_(void);
static void     turn_left_encoder_(uint32_t target_pulses);
static void     turn_right_encoder_(uint32_t target_pulses);
static bool     obstacle_detected_(float distance_cm);
static bool     object_too_close_(float distance_cm);
static bool     object_at_stop_distance_(float distance_cm);
static void     servo_init_(void);
static void     servo_set_angle_(float angle);
static bool     check_if_object_still_there_(void);
static void     encircle_object_with_checking_(void);
static void     complete_avoidance_cycle_(void);
static void     update_speed_and_distance_(void);
static void     debug_encoder_pulses_(const char *context);

/* ==============================
 * Servo Setup
 * ============================== */
static void servo_init_(void)
{
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, SERVO_PWM_CLKDIV);
    pwm_config_set_wrap(&cfg, SERVO_PWM_WRAP);
    pwm_init(slice, &cfg, true);
    servo_set_angle_(SERVO_CENTER_DEG);
    printf("[SERVO] init GPIO %d\n", SERVO_PIN);
}

static void servo_set_angle_(float angle)
{
    if (angle < 0.0f)  angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    float pulse_width_us = SERVO_MIN_PULSE_US +
        (angle / 180.0f) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    pwm_set_gpio_level(SERVO_PIN, (uint16_t)pulse_width_us);
}

/* ==============================
 * Speed Calculation
 * ============================== */
void speed_calc_init(void)
{
    encoders_init(true);
    g_last_speed_calc_time = to_ms_since_boot(get_absolute_time());
    g_current_speed_cm_s   = 0.0f;
    g_last_distance_cm     = 0.0f;
    printf("[SPEED] init\n");
}

static void update_speed_and_distance_(void)
{
    uint32_t now = to_ms_since_boot(get_absolute_time());
    uint32_t dt_ms = now - g_last_speed_calc_time;
    if (dt_ms < 100)
    {
        return;
    }

    float left_dist  = encoder_get_distance_cm(ENCODER_LEFT_GPIO);
    float right_dist = encoder_get_distance_cm(ENCODER_RIGHT_GPIO);
    float current_distance = (left_dist + right_dist) * 0.5f;

    float dist_travel = current_distance - g_last_distance_cm;
    float dt_s = dt_ms / 1000.0f;

    if (dt_s > 0.0f)
    {
        g_current_speed_cm_s = dist_travel / dt_s;
        if (g_current_speed_cm_s < 0.0f)    g_current_speed_cm_s = 0.0f;
        if (g_current_speed_cm_s > 200.0f)  g_current_speed_cm_s = 200.0f;
    }
    else
    {
        g_current_speed_cm_s = 0.0f;
    }

    g_last_distance_cm        = current_distance;
    g_last_speed_calc_time    = now;
}

float get_current_speed_cm_s(void)
{
    return g_current_speed_cm_s;
}

float get_total_distance_cm(void)
{
    float left_dist  = encoder_get_distance_cm(ENCODER_LEFT_GPIO);
    float right_dist = encoder_get_distance_cm(ENCODER_RIGHT_GPIO);
    return (left_dist + right_dist) * 0.5f;
}

void reset_total_distance(void)
{
    encoder_reset_distance(ENCODER_LEFT_GPIO);
    encoder_reset_distance(ENCODER_RIGHT_GPIO);
    g_last_distance_cm = 0.0f;
    printf("[SPEED] distance reset\n");
}

/* ==============================
 * Encoder Helpers
 * ============================== */
static uint32_t get_average_pulses_(void)
{
    int32_t l = encoder_get_pulse_count(ENCODER_LEFT_GPIO);
    int32_t r = encoder_get_pulse_count(ENCODER_RIGHT_GPIO);
    return (uint32_t)((l + r) / 2);
}

static void debug_encoder_pulses_(const char *context)
{
    int32_t lcnt = encoder_get_pulse_count(ENCODER_LEFT_GPIO);
    int32_t rcnt = encoder_get_pulse_count(ENCODER_RIGHT_GPIO);
    float   lspd = encoder_get_speed_cm_s_timeout(ENCODER_LEFT_GPIO, 200);
    float   rspd = encoder_get_speed_cm_s_timeout(ENCODER_RIGHT_GPIO, 200);
    float   ldist= encoder_get_distance_cm(ENCODER_LEFT_GPIO);
    float   rdist= encoder_get_distance_cm(ENCODER_RIGHT_GPIO);

    printf("[ENC][%s] L:%ld %.1fcm %.1fcm/s | R:%ld %.1fcm %.1fcm/s\n",
           context, lcnt, ldist, lspd, rcnt, rdist, rspd);
}

/* ==============================
 * Obstacle Detection
 * ============================== */
static bool obstacle_detected_(float distance_cm)
{
    return (distance_cm > 2.0f) && (distance_cm <= OBSTACLE_DETECTION_DISTANCE_CM);
}

static bool object_too_close_(float distance_cm)
{
    return (distance_cm > 2.0f) && (distance_cm <= SAFE_DISTANCE_CM);
}

static bool object_at_stop_distance_(float distance_cm)
{
    return (distance_cm > 2.0f) && (distance_cm <= INITIAL_STOP_DISTANCE_CM);
}

/* ==============================
 * Turning (Encoder-Based)
 * ============================== */
static void turn_left_encoder_(uint32_t target_pulses)
{
    encoder_reset_distance(ENCODER_LEFT_GPIO);
    encoder_reset_distance(ENCODER_RIGHT_GPIO);

    drive_signed(-50.0f * 1.25f, 50.0f);

    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (get_average_pulses_() < target_pulses)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((now - start) > 3000 && (get_average_pulses_() == 0))
        {
            printf("[TURN] fallback timed left\n");
            break;
        }
        sleep_ms(10);
    }
    all_stop();
}

static void turn_right_encoder_(uint32_t target_pulses)
{
    encoder_reset_distance(ENCODER_LEFT_GPIO);
    encoder_reset_distance(ENCODER_RIGHT_GPIO);

    drive_signed(50.0f * 1.25f, -50.0f);

    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (get_average_pulses_() < target_pulses)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((now - start) > 3000 && (get_average_pulses_() == 0))
        {
            printf("[TURN] fallback timed right\n");
            break;
        }
        sleep_ms(10);
    }
    all_stop();
}

void turn_left_90_degrees(void)   { turn_left_encoder_(TURN_90_DEG_PULSES); }
void turn_right_90_degrees(void)  { turn_right_encoder_(TURN_90_DEG_PULSES); }
void turn_left_45_degrees(void)   { turn_left_encoder_(TURN_45_DEG_PULSES); }

/* ==============================
 * Object Width Calculation
 * ============================== */
typedef struct
{
    float left_distance;
    float right_distance;
    float left_angle;
    float right_angle;
    float width_cm;
} ObjectScanResult;

static float calculate_object_width_(float d_left,
                                     float d_right,
                                     float ang_left,
                                     float ang_right)
{
    float angle_c = fabsf((ang_left + 15.0f) - (ang_right - 15.0f)) * (float)M_PI / 180.0f;
    float a2      = d_left * d_left;
    float b2      = d_right * d_right;
    float twoab   = 2.0f * d_left * d_right * cosf(angle_c);
    float c2      = a2 + b2 - twoab;
    if (c2 < 0.0f) c2 = fabsf(c2);
    return sqrtf(c2);
}

static ObjectScanResult scan_object_width_(void)
{
    ObjectScanResult res;
    res.left_distance  = -1.0f;
    res.right_distance = -1.0f;
    res.left_angle     = -1.0f;
    res.right_angle    = -1.0f;
    res.width_cm       = -1.0f;

    float max_left_dist  = -1.0f;
    float max_right_dist = -1.0f;
    float max_left_angle = -1.0f;
    float max_right_angle= -1.0f;
    bool left_found      = false;

    servo_set_angle_(OBJECT_SCAN_LEFT_START);
    sleep_ms(500);

    for (float angle = OBJECT_SCAN_LEFT_START;
         angle >= OBJECT_SCAN_RIGHT_END;
         angle -= OBJECT_SCAN_STEP_DEG)
    {
        servo_set_angle_(angle);
        sleep_ms(75);
        float d = ultrasonic_get_distance_cm();
        if ((d > 2.0f) && (d <= 50.0f))
        {
            if ((angle > 90.0f) && !left_found)
            {
                if (d > max_left_dist)
                {
                    max_left_dist  = d;
                    max_left_angle = angle;
                    left_found     = true;
                }
            }
            else if (angle <= 90.0f)
            {
                if (d > max_right_dist)
                {
                    max_right_dist  = d;
                    max_right_angle = angle;
                }
            }
        }
    }

    if ((max_left_dist > 0.0f) && (max_right_dist > 0.0f))
    {
        res.left_distance  = max_left_dist;
        res.left_angle     = max_left_angle;
        res.right_distance = max_right_dist;
        res.right_angle    = max_right_angle;
        res.width_cm       = calculate_object_width_(res.left_distance,
                                                     res.right_distance,
                                                     res.left_angle,
                                                     res.right_angle);
    }

    servo_set_angle_(SERVO_CENTER_DEG);
    return res;
}

/* ==============================
 * Presence Check
 * ============================== */
static bool check_if_object_still_there_(void)
{
    turn_right_90_degrees();
    bool detected = false;
    for (float angle = 45.0f; angle <= 135.0f; angle += 2.0f)
    {
        servo_set_angle_(angle);
        sleep_ms(100);
        float distance = ultrasonic_get_distance_cm();
        if (obstacle_detected_(distance))
        {
            detected = true;
            break;
        }
        sleep_ms(50);
    }
    servo_set_angle_(SERVO_CENTER_DEG);
    sleep_ms(200);
    return detected;
}

/* ==============================
 * Encircle Logic
 * ============================== */
static void encircle_object_with_checking_(void)
{
    bool continue_circling = true;
    uint32_t check_counter = 0;
    uint32_t no_object_counter = 0;
    const uint32_t REQUIRED_NO_OBJECT_CHECKS = 2;
    uint8_t turns_finished = 0;

    while (continue_circling)
    {
        check_counter++;

        encoder_reset_distance(ENCODER_LEFT_GPIO);
        encoder_reset_distance(ENCODER_RIGHT_GPIO);
        drive_signed(CIRCLE_BASE_SPEED_LEFT, CIRCLE_BASE_SPEED_RIGHT);

        while (get_average_pulses_() < CIRCLE_TARGET_PULSES)
        {
            if (turns_finished >= 2)
            {
                uint16_t ir_raw = ir_read_raw();
                int color_state = classify_colour(ir_raw);
                if (color_state == 1) /* black line */
                {
                    continue_circling = false;
                    break;
                }
            }
            sleep_ms(10);
        }
        all_stop();
        sleep_ms(2000);

        if (!continue_circling) break;

        bool object_still_there = check_if_object_still_there_();

        if (object_still_there)
        {
            no_object_counter = 0;
            turn_left_90_degrees();
        }
        else
        {
            no_object_counter++;
            if ((no_object_counter >= REQUIRED_NO_OBJECT_CHECKS) && (turns_finished < 2))
            {
                drive_signed(CIRCLE_BASE_SPEED_LEFT, CIRCLE_BASE_SPEED_RIGHT);
                while (get_average_pulses_() < CIRCLE_TARGET_PULSES)
                {
                    sleep_ms(10);
                }
                all_stop();
                no_object_counter = 0;
                turns_finished++;
            }
            else
            {
                turn_left_90_degrees();
            }
        }

        if (check_counter >= 15)
        {
            drive_signed(BASE_SPEED_LEFT, BASE_SPEED_RIGHT);
            sleep_ms(3000);
            all_stop();
            continue_circling = false;
        }
        sleep_ms(500);
    }
    all_stop();
}

/* ==============================
 * Complete Avoidance Cycle
 * ============================== */
static void complete_avoidance_cycle_(void)
{
    bool cycle_active = true;
    while (cycle_active)
    {
        bool approach_active      = true;
        bool object_initial_detect = false;

        while (approach_active)
        {
            float distance = ultrasonic_get_distance_cm();
            if (distance < 0.0f)
            {
                if (!object_initial_detect)
                {
                    drive_signed(BASE_SPEED_LEFT, BASE_SPEED_RIGHT);
                }
            }
            else if (object_at_stop_distance_(distance))
            {
                all_stop();
                approach_active        = false;
                object_initial_detect   = true;
                mqtt_publish_telemetry(get_current_speed_cm_s(),
                                       get_total_distance_cm(),
                                       0.0f, distance,
                                       "scan");
                ObjectScanResult scan = scan_object_width_();
                if (scan.width_cm > 0.0f)
                {
                    char width_str[40];
                    snprintf(width_str, sizeof(width_str),
                             "objw:%.1fcm", scan.width_cm);
                    mqtt_publish_telemetry(get_current_speed_cm_s(),
                                           get_total_distance_cm(),
                                           0.0f, distance,
                                           width_str);
                    sleep_ms(1000);
                }
            }
            else if (obstacle_detected_(distance))
            {
                drive_signed(BASE_SPEED_LEFT, BASE_SPEED_RIGHT);
                object_initial_detect = true;
            }
            else
            {
                drive_signed(BASE_SPEED_LEFT, BASE_SPEED_RIGHT);
            }
            sleep_ms(50);
        }

        sleep_ms(1000);
        turn_left_90_degrees();
        sleep_ms(1000);

        encircle_object_with_checking_();

        cycle_active = false;
        servo_set_angle_(SERVO_CENTER_DEG);
        sleep_ms(2000);
        drive_signed(40.0f, 40.0f);
        sleep_ms(500);
        drive_signed(-40.0f, 60.0f);
        sleep_ms(500);
        all_stop();
        sleep_ms(2000);
    }
}

/* ==============================
 * Public Entry (Obstacle Only)
 * ============================== */
bool check_obstacle_detection(void)
{
    return ultrasonic_detect_obstacle_fast();
}

void avoid_obstacle_only(void)
{
    ObjectScanResult scan = scan_object_width_();
    if (scan.width_cm > 0.0f)
    {
        char width_str[60];
        snprintf(width_str, sizeof(width_str),
                 "OBW:%.1fcm", scan.width_cm);
        mqtt_publish_telemetry(get_current_speed_cm_s(),
                               get_total_distance_cm(),
                               0.0f,
                               ultrasonic_get_distance_cm(),
                               width_str);
        sleep_ms(500);
    }

    turn_left_90_degrees();
    sleep_ms(800);
    encircle_object_with_checking_();
    servo_set_angle_(SERVO_CENTER_DEG);
    sleep_ms(2000);
    drive_signed(50.0f, 50.0f);
    sleep_ms(700);
    drive_signed(-40.0f, 60.0f);
    sleep_ms(400);
    all_stop();
    sleep_ms(1000);
}

/*** end of file ***/
