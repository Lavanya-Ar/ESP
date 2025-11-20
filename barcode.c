/** @file barcode.c
 *  @brief Interrupt or polling based Code 39 barcode capture and decoding.
 *
 *  NOTE: Refactored for Barr-C style: constants centralized, functions ordered,
 *        condensed debug, preserved original decoding approaches (forward/back).
 *  WARNING: Timing thresholds empirical; revalidate if robot speed or sensor
 *           placement changes.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "barcode.h"
#include "ir_sensor.h"
#include "motor_encoder_demo.h"
#include "PID_Line_Follow.h"
#include "mqtt_client.h"
#include "encoder.h"

#include "FreeRTOS.h"
#include "task.h"

/* ==============================
 * Configuration Constants
 * ============================== */
#define BARCODE_QUIET_US            (50000u)     /* 50 ms quiet → end frame */
#define BARCODE_MIN_TRANSITIONS     (9u)         /* min durations needed */
#define BARCODE_SORT_MIN_ENTRIES    (10u)
#define BARCODE_MEDIAN_LOWER_MIN    (5u)
#define BARCODE_MEDIAN_FALLBACK_US  (100000u)
#define BARCODE_MEDIAN_MIN_US       (10000u)
#define BARCODE_MEDIAN_MAX_US       (500000u)
#define BARCODE_MIN_NARROW_US       (40u)
#define BARCODE_MAX_NARROW_US       (7000u)
#define BARCODE_MIN_POLL_TRANS      (20u)
#define BARCODE_POLL_START_TIMEOUT  (300000u)    /* 300 ms wait start */
#define BARCODE_POLL_SLEEP_US       (50u)
#define BARCODE_IRQ_NOT_PIN         (RIGHT_IR_DIGITAL_PIN)

/* ==============================
 * Code39 Table (subset + checksum values)
 * ============================== */
typedef struct
{
    char        character;
    const char *pattern;   /* 9 symbols: W/N alternating bar/space (start with bar) */
    int         value;     /* Mod 43 value */
} code39_char_t;

/* NOTE: Table trimmed or reorganized. Add missing characters if needed. */
static const code39_char_t code39_table[] =
{
    { 'A',"WNNNNWNNW",10 }, { 'B',"NNWNNWNNW",11 },
    { 'C',"WNWNNNWN",12 },  { 'D',"NNNNWWNNW",13 }, { 'E',"WNNNWWNNN",14 },
    { 'F',"NNWNWWNNN",15 }, { 'G',"NNNNNWWNW",16 }, { 'H',"WNNNNWWNN",17 },
    { 'I',"NNWNNWWNN",18 }, { 'J',"NNNNWWWNN",19 }, { 'K',"WNNNNNNWW",20 },
    { 'L',"NNWNNNNWW",21 }, { 'M',"WNWNNNNWN",22 }, { 'N',"NNNNWNNWW",23 },
    { 'O',"WNNNWNNWN",24 }, { 'P',"NNWNWNNWN",25 }, { 'Q',"NNNNNNWWW",26 },
    { 'R',"WNNNNNWWN",27 }, { 'S',"NNWNNNWWN",28 }, { 'T',"NNNNWNWWN",29 },
    { 'U',"WWNNNNNWN",30 }, { 'V',"NWWNNNNWN",31 }, { 'W',"WWWNNNNNN",32 },
    { 'X',"NWNNWNNWN",33 }, { 'Y',"WWNNWNNNN",34 }, { 'Z',"NWWNWNNNN",35 }
};

#define CODE39_TABLE_SIZE (sizeof(code39_table)/sizeof(code39_char_t))

/* ==============================
 * Static Capture State
 * ============================== */
static volatile bool     g_capturing      = false;
static volatile uint32_t g_last_edge_us   = 0;
static volatile uint16_t g_transition_cnt = 0;
static volatile uint32_t g_durations[MAX_TRANSITIONS];
static volatile bool     g_frame_ready    = false;

/* ==============================
 * Private Types
 * ============================== */
typedef enum
{
    WIDTH_NARROW,
    WIDTH_WIDE,
    WIDTH_BAD
} width_t;

/* ==============================
 * Private Prototypes
 * ============================== */
static int        code39_value_(char c);
static char       code39_match_pattern_(const char *p);
static void       barcode_gpio_isr_(uint gpio, uint32_t events);
static uint32_t   estimate_narrow_us_(const uint32_t *w, uint16_t n);
static width_t    classify_width_(uint32_t dur_us, uint32_t narrow_us);
static bool       decode_symbols_(const uint32_t *dur, uint16_t n, char *out,
                                  uint8_t *out_len, bool expect_bar_first);
static bool       validate_and_strip_code39_(char *chars, uint8_t *len,
                                             bool *checksum_ok);
static bool       decode_with_direction_(const uint32_t *dur, uint16_t n,
                                         barcode_result_t *result, bool forward);
static bool       copy_and_decode_frame_(barcode_result_t *result);
static bool       polling_capture_and_decode_(barcode_result_t *result);

/* ==============================
 * Code39 Helpers
 * ============================== */
static int code39_value_(char c)
{
    for (uint i = 0; i < CODE39_TABLE_SIZE; i++)
    {
        if (code39_table[i].character == c)
        {
            return code39_table[i].value;
        }
    }
    return -1;
}

static char code39_match_pattern_(const char *p)
{
    for (uint i = 0; i < CODE39_TABLE_SIZE; i++)
    {
        if (strcmp(p, code39_table[i].pattern) == 0)
        {
            return code39_table[i].character;
        }
    }
    return '?';
}

/* ==============================
 * Interrupt ISR
 * ============================== */
static void barcode_gpio_isr_(uint gpio, uint32_t events)
{
    (void)events;
    if (gpio != BARCODE_IRQ_NOT_PIN)
    {
        return;
    }

    uint32_t now = time_us_32();
    if (!g_capturing)
    {
        g_capturing      = true;
        g_last_edge_us   = now;
        g_transition_cnt = 0;
        return;
    }

    uint32_t dt = now - g_last_edge_us;
    g_last_edge_us = now;

    if (g_transition_cnt < MAX_TRANSITIONS)
    {
        g_durations[g_transition_cnt++] = dt;
    }
    else
    {
        g_frame_ready  = true;
        g_capturing    = false;
    }
}

/* ==============================
 * Public Init
 * ============================== */
void barcode_init(void)
{
    ir_digital_init();
    /* user can call barcode_irq_init() subsequently if using interrupts */
    printf("[BARCODE] init digital on GPIO%d\n", RIGHT_IR_DIGITAL_PIN);
}

void barcode_irq_init(void)
{
    gpio_init(RIGHT_IR_DIGITAL_PIN);
    gpio_set_dir(RIGHT_IR_DIGITAL_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_IR_DIGITAL_PIN);
    gpio_set_irq_enabled_with_callback(RIGHT_IR_DIGITAL_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &barcode_gpio_isr_);
    printf("[BARCODE] IRQ armed GPIO%d\n", RIGHT_IR_DIGITAL_PIN);
}

/* ==============================
 * Frame readiness check
 * ============================== */
bool barcode_capture_ready(void)
{
    if (g_frame_ready)
    {
        return true;
    }

    if (g_capturing)
    {
        uint32_t quiet = time_us_32() - g_last_edge_us;
        if ((quiet > BARCODE_QUIET_US) && (g_transition_cnt >= BARCODE_MIN_TRANSITIONS))
        {
            g_frame_ready  = true;
            g_capturing    = false;
        }
    }
    return g_frame_ready;
}

/* ==============================
 * Width Estimation / Classification
 * ============================== */
static uint32_t estimate_narrow_us_(const uint32_t *w, uint16_t n)
{
    if (n < BARCODE_SORT_MIN_ENTRIES)
    {
        return BARCODE_MEDIAN_FALLBACK_US;
    }

    uint32_t tmp[MAX_TRANSITIONS];
    for (uint16_t i = 0; i < n; i++)
    {
        tmp[i] = w[i];
    }

    /* Bubble sort (small n acceptable). */
    for (uint16_t i = 0; i < n - 1; i++)
    {
        for (uint16_t j = 0; j < n - i - 1; j++)
        {
            if (tmp[j] > tmp[j + 1])
            {
                uint32_t t   = tmp[j];
                tmp[j]       = tmp[j + 1];
                tmp[j + 1]   = t;
            }
        }
    }

    uint16_t lower = n / 3;
    if (lower < BARCODE_MEDIAN_LOWER_MIN)
    {
        lower = BARCODE_MEDIAN_LOWER_MIN;
    }

    uint32_t sum = 0;
    for (uint16_t i = 0; i < lower; i++)
    {
        sum += tmp[i];
    }

    uint32_t median = sum / lower;
    if (median < BARCODE_MEDIAN_MIN_US)
    {
        return BARCODE_MEDIAN_MIN_US;
    }
    if (median > BARCODE_MEDIAN_MAX_US)
    {
        return BARCODE_MEDIAN_MAX_US;
    }
    return median;
}

static width_t classify_width_(uint32_t dur_us, uint32_t narrow_us)
{
    if (narrow_us == 0)
    {
        return WIDTH_BAD;
    }

    float ratio = (float)dur_us / (float)narrow_us;
    if (ratio < 3.0f)
    {
        return WIDTH_NARROW;
    }
    if (ratio > 3.0f)
    {
        return WIDTH_WIDE;
    }
    return WIDTH_BAD;
}

/* ==============================
 * Pattern Decoding
 * ============================== */
static bool decode_symbols_(const uint32_t *dur,
                            uint16_t n,
                            char *out,
                            uint8_t *out_len,
                            bool expect_bar_first)
{
    *out_len = 0;
    if (n < 9)
    {
        return false;
    }

    uint32_t narrow_us = estimate_narrow_us_(dur, n);
    if ((narrow_us < BARCODE_MIN_NARROW_US) || (narrow_us > BARCODE_MAX_NARROW_US))
    {
        return false;
    }

    char pattern[10];
    uint8_t pidx = 0;
    bool bar = expect_bar_first;

    for (uint16_t i = 0; i < n; i++)
    {
        (void)bar;
        width_t w = classify_width_(dur[i], narrow_us);
        if (w == WIDTH_BAD)
        {
            continue;
        }
        pattern[pidx++] = (w == WIDTH_WIDE) ? 'W' : 'N';
        if (pidx == 9)
        {
            pattern[9] = '\0';
            char c = code39_match_pattern_(pattern);
            out[(*out_len)++] = c;
            pidx = 0;
            if (*out_len >= (BARCODE_MAX_LENGTH + 2))
            {
                break; /* include start/stop */
            }
        }
        bar = !bar;
    }
    out[*out_len] = '\0';
    return (*out_len >= 3);
}

static bool validate_and_strip_code39_(char *chars,
                                       uint8_t *len,
                                       bool *checksum_ok)
{
    *checksum_ok = false;
    if (*len < 3)
    {
        return false;
    }
    if ((chars[0] != '*') || (chars[*len - 1] != '*'))
    {
        return false;
    }

    /* Optional checksum if at least 4 chars including * */
    if (*len >= 4)
    {
        int sum = 0;
        for (uint8_t i = 1; i < (*len - 2); i++)
        {
            int v = code39_value_(chars[i]);
            if (v < 0)
            {
                return false;
            }
            sum += v;
        }
        int chk  = sum % 43;
        int last = code39_value_(chars[*len - 2]);
        if ((last >= 0) && (last == chk))
        {
            *checksum_ok = true;
            /* strip checksum char */
            chars[*len - 2] = '*';
            chars[*len - 1] = '\0';
            (*len)--;
        }
    }

    /* Strip surrounding * */
    uint8_t payload = (*len) - 2;
    memmove(chars, chars + 1, payload);
    chars[payload] = '\0';
    *len = payload;
    return true;
}

static bool decode_with_direction_(const uint32_t *dur,
                                   uint16_t n,
                                   barcode_result_t *result,
                                   bool forward)
{
    char chars[BARCODE_MAX_LENGTH + 4] = {0};
    uint8_t clen = 0;
    uint32_t tmp[MAX_TRANSITIONS];

    if (forward)
    {
        for (uint16_t i = 0; i < n; i++)
        {
            tmp[i] = dur[i];
        }
    }
    else
    {
        for (uint16_t i = 0; i < n; i++)
        {
            tmp[i] = dur[n - 1U - i];
        }
    }

    bool chk = false;
    if (decode_symbols_(tmp, n, chars, &clen, true)
        && validate_and_strip_code39_(chars, &clen, &chk))
    {
        strncpy(result->data, chars, BARCODE_MAX_LENGTH);
        result->length      = (uint8_t)strnlen(result->data, BARCODE_MAX_LENGTH);
        result->valid       = (result->length > 0);
        result->checksum_ok = chk;
        return result->valid;
    }
    if (decode_symbols_(tmp, n, chars, &clen, false)
        && validate_and_strip_code39_(chars, &clen, &chk))
    {
        strncpy(result->data, chars, BARCODE_MAX_LENGTH);
        result->length      = (uint8_t)strnlen(result->data, BARCODE_MAX_LENGTH);
        result->valid       = (result->length > 0);
        result->checksum_ok = chk;
        return result->valid;
    }
    return false;
}

/* ==============================
 * Interrupt Frame Handling
 * ============================== */
static bool copy_and_decode_frame_(barcode_result_t *result)
{
    if (!g_frame_ready)
    {
        return false;
    }

    uint16_t n = g_transition_cnt;
    if (n < BARCODE_MIN_TRANSITIONS)
    {
        g_frame_ready = false;
        return false;
    }

    uint32_t buf[MAX_TRANSITIONS];
    for (uint16_t i = 0; i < n; i++)
    {
        buf[i] = g_durations[i];
    }

    g_frame_ready = false; /* consume */

    uint32_t t0 = time_us_32();
    bool ok = decode_with_direction_(buf, n, result, true)
              || decode_with_direction_(buf, n, result, false);

    result->scan_time_us = time_us_32() - t0;
    return ok;
}

/* ==============================
 * Polling Capture (Alternative)
 * ============================== */
static bool polling_capture_and_decode_(barcode_result_t *result)
{
    memset(result, 0, sizeof(*result));
    uint32_t tstart = time_us_32();
    uint32_t dur[MAX_TRANSITIONS];
    uint16_t n = 0;
    bool last_state = ir_read_digital();
    uint32_t seg_start  = time_us_32();
    uint32_t last_change = seg_start;

    /* Wait for start condition or timeout */
    uint32_t w0 = time_us_32();
    while ((time_us_32() - w0) < BARCODE_POLL_START_TIMEOUT)
    {
        bool current_state = ir_read_digital();
        if (current_state != last_state)
        {
            last_state = current_state;
            break;
        }
        sleep_us(100);
    }

    /* Measure transitions until quiet or buffer full */
    while (n < (MAX_TRANSITIONS - 1U))
    {
        bool current_state = ir_read_digital();
        if (current_state != last_state)
        {
            uint32_t now = time_us_32();
            dur[n++]     = now - seg_start;
            seg_start    = now;
            last_state   = current_state;
            last_change  = now;
        }
        else
        {
            if ((time_us_32() - last_change) > BARCODE_QUIET_US)
            {
                uint32_t now = time_us_32();
                dur[n++]     = now - seg_start;
                break;
            }
        }
        sleep_us(BARCODE_POLL_SLEEP_US);
    }

    if (n < BARCODE_MIN_POLL_TRANS)
    {
        return false;
    }

    bool ok = decode_with_direction_(dur, n, result, true)
              || decode_with_direction_(dur, n, result, false);

    result->scan_time_us = time_us_32() - tstart;
    return ok;
}

/* ==============================
 * Public API
 * ============================== */
bool barcode_decode_captured(barcode_result_t *result)
{
    return copy_and_decode_frame_(result);
}

bool barcode_scan_digital(barcode_result_t *result)
{
    return polling_capture_and_decode_(result);
}

barcode_command_t barcode_parse_command(const char *s)
{
    if ((s == NULL) || (*s == '\0'))
    {
        return BARCODE_CMD_UNKNOWN;
    }
    if ((!strcmp(s, "LEFT")) || (!strcmp(s, "L")))
    {
        return BARCODE_CMD_LEFT;
    }
    if ((!strcmp(s, "RIGHT")) || (!strcmp(s, "R")))
    {
        return BARCODE_CMD_RIGHT;
    }
    if ((!strcmp(s, "STOP")) || (!strcmp(s, "S")))
    {
        return BARCODE_CMD_STOP;
    }
    if ((!strcmp(s, "GO")) || (!strcmp(s, "F")) ||
        (!strcmp(s, "FWD")) || (!strcmp(s, "FORWARD")))
    {
        return BARCODE_CMD_FORWARD;
    }
    return BARCODE_CMD_UNKNOWN;
}

/*** end of file ***/
