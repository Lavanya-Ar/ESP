/** @file ultrasonic.c
 *  @brief HC-SR04 ultrasonic sensor measurement (distance + fast obstacle detect).
 *
 *  NOTE: Barr-C style; retry logic and fast detect retained.
 */

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "ultrasonic.h"

/* ==============================
 * Constants
 * ============================== */
#define SPEED_OF_SOUND_CM_PER_US  (0.03432f)
#define ULTRA_MAX_ATTEMPTS        (3)
#define ULTRA_TRIGGER_US          (12)
#define ULTRA_WAIT_TIMEOUT_US     (30000)
#define ULTRA_PULSE_TIMEOUT_US    (60000)
#define ULTRA_MIN_VALID_CM        (2.0f)
#define ULTRA_MAX_VALID_CM        (400.0f)
#define ULTRA_FAST_WAIT_TIMEOUT   (5000)
#define ULTRA_FAST_PULSE_TIMEOUT  (10000)
#define ULTRA_FAST_NEAR_MAX_CM    (20.0f)

/* ==============================
 * Initialization
 * ============================== */
void ultrasonic_init(void)
{
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
}

/* ==============================
 * Trigger
 * ============================== */
static void ultrasonic_trigger_(void)
{
    gpio_put(TRIG_PIN, 0);
    sleep_us(5);
    gpio_put(TRIG_PIN, 1);
    sleep_us(ULTRA_TRIGGER_US);
    gpio_put(TRIG_PIN, 0);
}

/* ==============================
 * Distance Measurement
 * ============================== */
float ultrasonic_get_distance_cm(void)
{
    for (int attempt = 0; attempt < ULTRA_MAX_ATTEMPTS; attempt++)
    {
        ultrasonic_trigger_();

        uint32_t start_wait = time_us_32();
        while (!gpio_get(ECHO_PIN))
        {
            if ((time_us_32() - start_wait) > ULTRA_WAIT_TIMEOUT_US)
            {
                break;
            }
        }
        if ((time_us_32() - start_wait) > ULTRA_WAIT_TIMEOUT_US)
        {
            sleep_ms(50);
            continue;
        }

        uint32_t pulse_start = time_us_32();
        while (gpio_get(ECHO_PIN))
        {
            if ((time_us_32() - pulse_start) > ULTRA_PULSE_TIMEOUT_US)
            {
                break;
            }
        }
        uint32_t pulse_end = time_us_32();
        uint32_t pulse_dur = pulse_end - pulse_start;

        float dist_cm = (pulse_dur * SPEED_OF_SOUND_CM_PER_US) / 2.0f;
        if ((dist_cm >= ULTRA_MIN_VALID_CM) &&
            (dist_cm <= ULTRA_MAX_VALID_CM))
        {
            return dist_cm;
        }
        sleep_ms(60);
    }
    return -1.0f;
}

/* ==============================
 * Fast Obstacle Detect
 * ============================== */
bool ultrasonic_detect_obstacle_fast(void)
{
    ultrasonic_trigger_();

    uint32_t start_wait = time_us_32();
    while (!gpio_get(ECHO_PIN))
    {
        if ((time_us_32() - start_wait) > ULTRA_FAST_WAIT_TIMEOUT)
        {
            return false;
        }
    }

    uint32_t pulse_start = time_us_32();
    while (gpio_get(ECHO_PIN))
    {
        if ((time_us_32() - pulse_start) > ULTRA_FAST_PULSE_TIMEOUT)
        {
            return false;
        }
    }
    uint32_t pulse_end = time_us_32();
    uint32_t pulse_dur = pulse_end - pulse_start;

    float dist_cm = (pulse_dur * SPEED_OF_SOUND_CM_PER_US) / 2.0f;
    return (dist_cm >= ULTRA_MIN_VALID_CM) && (dist_cm <= ULTRA_FAST_NEAR_MAX_CM);
}

/*** end of file ***/
