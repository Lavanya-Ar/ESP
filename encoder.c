/** @file encoder.c
 *  @brief Interrupt-driven wheel encoder measurement (pulse count, distance, speed).
 *
 *  NOTE: Barr-C style; checks for reasonable pulse periods.
 */

#include "encoder.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "motor_encoder_demo.h"
#include <stdio.h>

/* ==============================
 * Static Instances
 * ============================== */
encoder_t left_encoder  = { ENCODER_LEFT_GPIO,  0, 0, 0, 0.0f };
encoder_t right_encoder = { ENCODER_RIGHT_GPIO, 0, 0, 0, 0.0f };

/* ==============================
 * ISR
 * ============================== */
void encoder_global_isr(uint gpio, uint32_t events)
{
    (void)events;
    uint32_t now = time_us_32();

    encoder_t *enc = NULL;
    if (gpio == ENCODER_LEFT_GPIO)
    {
        enc = &left_encoder;
    }
    else if (gpio == ENCODER_RIGHT_GPIO)
    {
        enc = &right_encoder;
    }
    else
    {
        return;
    }

    enc->pulse_count++;
    if (enc->last_time_us > 0)
    {
        uint32_t period = now - enc->last_time_us;
        if ((period > 1000u) && (period < 1000000u))
        {
            enc->last_period_us = period;
            float dist_per_pulse = (WHEEL_DIAMETER_CM * 3.14159f) /
                                   (float)PULSES_PER_REVOLUTION;
            enc->distance_cm += dist_per_pulse;
        }
    }
    enc->last_time_us = now;
}

/* ==============================
 * Initialization
 * ============================== */
void encoder_init(bool pull_up)
{
    gpio_init(ENCODER_LEFT_GPIO);
    gpio_set_dir(ENCODER_LEFT_GPIO, GPIO_IN);
    gpio_init(ENCODER_RIGHT_GPIO);
    gpio_set_dir(ENCODER_RIGHT_GPIO, GPIO_IN);

    if (pull_up)
    {
        gpio_pull_up(ENCODER_LEFT_GPIO);
        gpio_pull_up(ENCODER_RIGHT_GPIO);
    }
    else
    {
        gpio_disable_pulls(ENCODER_LEFT_GPIO);
        gpio_disable_pulls(ENCODER_RIGHT_GPIO);
    }
}

void encoders_init(bool pull_up)
{
    encoder_init(pull_up);
    uint32_t t = time_us_32();

    left_encoder.pulse_count   = 0;
    left_encoder.last_time_us  = t;
    left_encoder.last_period_us= 100000;
    left_encoder.distance_cm   = 0.0f;

    right_encoder.pulse_count   = 0;
    right_encoder.last_time_us  = t;
    right_encoder.last_period_us= 100000;
    right_encoder.distance_cm   = 0.0f;

    gpio_acknowledge_irq(ENCODER_LEFT_GPIO, GPIO_IRQ_EDGE_RISE);
    gpio_acknowledge_irq(ENCODER_RIGHT_GPIO, GPIO_IRQ_EDGE_RISE);

    gpio_set_irq_enabled(ENCODER_LEFT_GPIO, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(ENCODER_RIGHT_GPIO, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_callback(encoder_global_isr);

    irq_set_enabled(IO_IRQ_BANK0, true);
}

/* ==============================
 * Public Accessors
 * ============================== */
float encoder_get_distance_cm(uint gpio_pin)
{
    encoder_t *enc = (gpio_pin == ENCODER_LEFT_GPIO) ? &left_encoder : &right_encoder;
    return enc->distance_cm;
}

int32_t encoder_get_pulse_count(uint gpio_pin)
{
    encoder_t *enc = (gpio_pin == ENCODER_LEFT_GPIO) ? &left_encoder : &right_encoder;
    return enc->pulse_count;
}

void encoder_reset_distance(uint gpio_pin)
{
    encoder_t *enc = (gpio_pin == ENCODER_LEFT_GPIO) ? &left_encoder : &right_encoder;
    enc->distance_cm   = 0.0f;
    enc->pulse_count   = 0;
    enc->last_time_us  = time_us_32();
    enc->last_period_us= 0;
}

float encoder_get_speed_cm_s(uint gpio_pin)
{
    encoder_t *enc = (gpio_pin == ENCODER_LEFT_GPIO) ? &left_encoder : &right_encoder;
    if ((enc->last_period_us <= 1000u) || (enc->last_period_us > 10000000u))
    {
        return 0.0f;
    }
    float dist_per_pulse = (WHEEL_DIAMETER_CM * 3.14159f) / (float)PULSES_PER_REVOLUTION;
    float time_per_pulse = (float)enc->last_period_us / 1e6f;
    if (time_per_pulse < 0.0001f) return 0.0f;
    float speed = dist_per_pulse / time_per_pulse;
    if (speed > 200.0f) speed = 200.0f;
    return speed;
}

float encoder_get_speed_cm_s_timeout(uint gpio_pin, uint32_t timeout_ms)
{
    encoder_t *enc = (gpio_pin == ENCODER_LEFT_GPIO) ? &left_encoder : &right_encoder;
    uint32_t now = time_us_32();
    uint32_t since_last = (now - enc->last_time_us) / 1000u;
    if (since_last > timeout_ms)
    {
        return 0.0f;
    }
    return encoder_get_speed_cm_s(gpio_pin);
}

/*** end of file ***/
