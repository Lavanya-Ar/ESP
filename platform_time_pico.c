/** @file platform_time_pico.c
 *  @brief Helpers for millisecond timing and delays on Raspberry Pi Pico SDK.
 *
 *  NOTE: Simple thin wrapper around Pico time APIs for portability.
 */

#include <stdint.h>
#include "pico/time.h"
#include "pico/stdlib.h"

/* ==============================
 * Public API
 * ============================== */
uint64_t platform_millis_pico(void)
{
    return time_us_64() / 1000ULL;
}

void platform_delay_ms_pico(uint32_t ms)
{
    sleep_ms(ms);
}

/*** end of file ***/
