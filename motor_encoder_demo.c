/** @file motor_encoder_demo.c
 *  @brief Motor PWM setup, signed drive, encoder polling & calibration routine.
 *
 *  NOTE: Polling-based encoder accumulation is for calibration only; may miss pulses
 *        at high rotation speeds. Consider interrupt-driven encoder for runtime control.
 *  WARNING: Magic numbers (wrap value, timing windows) derived for default Pico clock
 *           at 125 MHz. Adjust constants if clock configuration changes.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/time.h"

/* ==============================
 * Pin Configuration
 * ============================== */
#define MOTOR1_A_PIN            (9u)
#define MOTOR1_B_PIN            (8u)
#define MOTOR2_A_PIN            (10u)
#define MOTOR2_B_PIN            (11u)
#define ENCODER_LEFT_PIN        (6u)
#define ENCODER_RIGHT_PIN       (4u)

/* ==============================
 * PWM Configuration
 * ============================== */
#define PWM_WRAP_VALUE          (9999u)
#define PWM_TARGET_FREQ_HZ      (20000.0f)
#define PICO_SYS_CLOCK_HZ       (125000000.0f)

/* ==============================
 * Calibration Settings
 * ============================== */
#define CALIB_BASE_SPEED_PCT    (30.0f)
#define CALIB_RUN_MS            (2000u)
#define CALIB_VERIFY_MS         (1000u)
#define POLL_PRINT_INTERVAL_MS  (100u)
#define POLL_SAMPLE_DELAY_US    (100u)

/* ==============================
 * Types & Globals
 * ============================== */
typedef struct
{
    uint32_t pulse_count;
} enc_acc_t;

static enc_acc_t g_left_enc  = { 0 };
static enc_acc_t g_right_enc = { 0 };

/* ==============================
 * Private Prototypes
 * ============================== */
static void setup_pwm_(uint pin);
static void set_pwm_pct_(uint pin, float pct);
static void poll_encoders_(uint32_t duration_ms);
static void print_calibration_results_(uint32_t left_count,
                                       uint32_t right_count);
static float clamp_pct_(float pct);

/* ==============================
 * PWM Setup
 * ============================== */
static void setup_pwm_(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    float div = PICO_SYS_CLOCK_HZ / (PWM_TARGET_FREQ_HZ * (PWM_WRAP_VALUE + 1.0f));
    pwm_set_clkdiv(slice, div);
    pwm_set_wrap(slice, PWM_WRAP_VALUE);
    pwm_set_gpio_level(pin, 0);
    pwm_set_enabled(slice, true);
}

static float clamp_pct_(float pct)
{
    if (pct < 0.0f)  return 0.0f;
    if (pct > 100.0f) return 100.0f;
    return pct;
}

static void set_pwm_pct_(uint pin, float pct)
{
    pct = clamp_pct_(pct);
    uint slice = pwm_gpio_to_slice_num(pin);
    uint32_t top = pwm_hw->slice[slice].top;
    pwm_set_gpio_level(pin, (uint16_t)((pct / 100.0f) * (float)(top + 1)));
}

/* ==============================
 * Drive Helpers (public if used externally)
 * ============================== */
void drive_signed(float left_pct, float right_pct)
{
    if (left_pct >= 0.0f)
    {
        set_pwm_pct_(MOTOR1_A_PIN, left_pct);
        set_pwm_pct_(MOTOR1_B_PIN, 0.0f);
    }
    else
    {
        set_pwm_pct_(MOTOR1_A_PIN, 0.0f);
        set_pwm_pct_(MOTOR1_B_PIN, -left_pct);
    }

    if (right_pct >= 0.0f)
    {
        set_pwm_pct_(MOTOR2_A_PIN, right_pct);
        set_pwm_pct_(MOTOR2_B_PIN, 0.0f);
    }
    else
    {
        set_pwm_pct_(MOTOR2_A_PIN, 0.0f);
        set_pwm_pct_(MOTOR2_B_PIN, -right_pct);
    }
}

void all_stop(void)
{
    drive_signed(0.0f, 0.0f);
}

/* ==============================
 * Encoder Poll (Calibration Only)
 * ============================== */
static void poll_encoders_(uint32_t duration_ms)
{
    g_left_enc.pulse_count  = 0;
    g_right_enc.pulse_count = 0;

    bool last_left  = gpio_get(ENCODER_LEFT_PIN);
    bool last_right = gpio_get(ENCODER_RIGHT_PIN);

    absolute_time_t start_time   = get_absolute_time();
    uint32_t last_print_time_ms  = 0;
    uint32_t start_ms            = to_ms_since_boot(start_time);

    while (absolute_time_diff_us(start_time, get_absolute_time()) < (duration_ms * 1000ULL))
    {
        bool current_left  = gpio_get(ENCODER_LEFT_PIN);
        bool current_right = gpio_get(ENCODER_RIGHT_PIN);

        if (current_left && !last_left)
        {
            g_left_enc.pulse_count++;
        }
        if (current_right && !last_right)
        {
            g_right_enc.pulse_count++;
        }

        last_left  = current_left;
        last_right = current_right;

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if ((now_ms - last_print_time_ms) >= POLL_PRINT_INTERVAL_MS)
        {
            float elapsed_s = (float)(now_ms - start_ms) / 1000.0f;
            printf("t=%.1fs left=%lu right=%lu\r",
                   elapsed_s,
                   g_left_enc.pulse_count,
                   g_right_enc.pulse_count);
            last_print_time_ms = now_ms;
        }

        sleep_us(POLL_SAMPLE_DELAY_US);
    }
    printf("\n");
}

/* ==============================
 * Calibration Results
 * ============================== */
static void print_calibration_results_(uint32_t left_count, uint32_t right_count)
{
    printf("\n=== CALIBRATION RESULTS ===\n");
    printf("Left pulses:  %lu\n", left_count);
    printf("Right pulses: %lu\n", right_count);

    if ((left_count == 0) || (right_count == 0))
    {
        printf("ERROR: Zero pulses detected. Check wiring.\n");
        return;
    }

    int32_t diff = (int32_t)left_count - (int32_t)right_count;
    float avg    = (left_count + right_count) / 2.0f;
    float diff_pct = (diff / avg) * 100.0f;
    printf("Difference: %ld (%.1f%%)\n", diff, diff_pct);

    float left_comp  = 1.0f;
    float right_comp = 1.0f;

    if (left_count > right_count)
    {
        left_comp = (float)right_count / (float)left_count;
        printf("Left faster → left compensation: %.3f\n", left_comp);
    }
    else if (right_count > left_count)
    {
        right_comp = (float)left_count / (float)right_count;
        printf("Right faster → right compensation: %.3f\n", right_comp);
    }
    else
    {
        printf("Motors balanced. No compensation needed.\n");
    }

    float recommended_left  = CALIB_BASE_SPEED_PCT * left_comp;
    float recommended_right = CALIB_BASE_SPEED_PCT * right_comp;

    printf("\n=== RECOMMENDED SETTINGS ===\n");
    printf("Base %.1f%% → Left: %.1f%% Right: %.1f%%\n",
           CALIB_BASE_SPEED_PCT, recommended_left, recommended_right);
    printf("\nUsage:\n");
    printf("drive_signed(desired * LEFT_COMP, desired * RIGHT_COMP);\n");
    printf("LEFT_COMP=%.3f RIGHT_COMP=%.3f\n", left_comp, right_comp);

    printf("\n=== VERIFICATION (1s) ===\n");
    drive_signed(recommended_left, recommended_right);
    poll_encoders_(CALIB_VERIFY_MS);
    all_stop();

    uint32_t v_left  = g_left_enc.pulse_count;
    uint32_t v_right = g_right_enc.pulse_count;
    int32_t v_diff   = (int32_t)v_left - (int32_t)v_right;
    float   v_avg    = (v_left + v_right) / 2.0f;
    float   v_diff_pct = (v_diff / v_avg) * 100.0f;
    printf("Post-comp difference: %ld (%.1f%%)\n", v_diff, v_diff_pct);

    if (abs(v_diff) < abs(diff) / 2)
    {
        printf("Compensation improved balance.\n");
    }
    else
    {
        printf("Further adjustment may be needed.\n");
    }
}

/* ==============================
 * Public Calibration Function
 * ============================== */
void calibrate_motors(void)
{
    printf("\n=== MOTOR CALIBRATION ===\n");
    printf("Forward %.1f%% for %u ms\n\n", CALIB_BASE_SPEED_PCT, CALIB_RUN_MS);

    drive_signed(CALIB_BASE_SPEED_PCT, CALIB_BASE_SPEED_PCT);
    poll_encoders_(CALIB_RUN_MS);
    all_stop();

    print_calibration_results_(g_left_enc.pulse_count, g_right_enc.pulse_count);
}

/* ==============================
 * Initialization
 * ============================== */
void motor_encoder_init(void)
{
    setup_pwm_(MOTOR1_A_PIN);
    setup_pwm_(MOTOR1_B_PIN);
    setup_pwm_(MOTOR2_A_PIN);
    setup_pwm_(MOTOR2_B_PIN);

    gpio_init(ENCODER_LEFT_PIN);
    gpio_set_dir(ENCODER_LEFT_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_LEFT_PIN);

    gpio_init(ENCODER_RIGHT_PIN);
    gpio_set_dir(ENCODER_RIGHT_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_RIGHT_PIN);

    printf("[MOTOR] Polling calibration mode ready.\n");
}

/*** end of file ***/
