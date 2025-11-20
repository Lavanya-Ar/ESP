/** @file ir_sensor.c
 *  @brief Analog + digital IR sensor reading and test utilities (optional).
 *
 *  NOTE: Barr-C style; classification preserved. Standalone test code retained but can be gated.
 */

#include "ir_sensor.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* ==============================
 * Initialization
 * ============================== */
void ir_init(ir_calib_t *cal)
{
    adc_init();
    adc_gpio_init(IR_GPIO);
    adc_select_input(IR_ADC_INPUT);
    if (cal)
    {
        cal->min_raw = 4095;
        cal->max_raw = 0;
    }
    printf("[IR] ADC init GPIO%d\n", IR_GPIO);
}

void ir_digital_init(void)
{
    gpio_init(RIGHT_IR_DIGITAL_PIN);
    gpio_set_dir(RIGHT_IR_DIGITAL_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_IR_DIGITAL_PIN);
}

/* ==============================
 * Read Raw (Averaged)
 * ============================== */
uint16_t ir_read_raw(void)
{
    (void)adc_read();
    sleep_us(5);
    const int N = 12;
    uint32_t acc = 0;
    for (int i = 0; i < N; i++)
    {
        acc += adc_read();
        sleep_us(5);
    }
    return (uint16_t)(acc / N);
}

bool ir_read_digital(void)
{
    return gpio_get(RIGHT_IR_DIGITAL_PIN);
}

/* ==============================
 * Classification
 * ============================== */
bool ir_is_black(uint16_t sample, const ir_calib_t *cal)
{
    uint16_t th = ir_threshold(cal);
#if IR_BLACK_IS_LOWER
    return sample < th;
#else
    return sample > th;
#endif
}

int ir_classify(uint16_t sample, const ir_calib_t *cal)
{
    return ir_is_black(sample, cal) ? 1 : 0;
}

int ir_classify_hysteresis(uint16_t sample, const ir_calib_t *cal, int prev)
{
    uint16_t th   = ir_threshold(cal);
    uint16_t low  = (th > IR_HYST_MARGIN) ? (th - IR_HYST_MARGIN) : 0;
    uint16_t high = (th + IR_HYST_MARGIN <= 400) ? (th + IR_HYST_MARGIN) : 400;

#if IR_BLACK_IS_LOWER
    if (sample <= low)  return 1;
    if (sample >= high) return 0;
    return prev;
#else
    if (sample >= high) return 1;
    if (sample <= low)  return 0;
    return prev;
#endif
}

int classify_colour(uint16_t raw)
{
    if (raw > 1000)            return 1; /* black */
    else if (raw > 400)        return 2; /* grey */
    return 0;                  /* white */
}

/*** end of file ***/
