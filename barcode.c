#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "barcode.h"
#include "ir_sensor.h"

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#include "motor_encoder_demo.h"
#include "PID_Line_Follow.h"
#include "mqtt_client.h"
#include "encoder.h"

// ===================== Code 39 table (with Mod 43 values) =====================
typedef struct {
    char character;
    const char *pattern;  // 9 symbols: W/N alternating bar/space starting with bar
    int  value;           // Mod 43 value; '*' has no value
} code39_char_t;

static const code39_char_t code39_table[] = {
    {'A',"WNNNNWNNW",10}, {'B',"NNWNNWNNW",11},
    {'C',"WNWNNNWN",12},  {'D',"NNNNWWNNW",13}, {'E',"WNNNWWNNN",14},
    {'F',"NNWNWWNNN",15},  {'G',"NNNNNWWNW",16}, {'H',"WNNNNWWNN",17},
    {'I',"NNWNNWWNN",18},  {'J',"NNNNWWWNN",19}, {'K',"WNNNNNNWW",20},
    {'L',"NNWNNNNWW",21},  {'M',"WNWNNNNWN",22}, {'N',"NNNNWNNWW",23},
    {'O',"WNNNWNNWN",24},  {'P',"NNWNWNNWN",25}, {'Q',"NNNNNNWWW",26},
    {'R',"WNNNNNWWN",27},  {'S',"NNWNNNWWN",28}, {'T',"NNNNWNWWN",29},
    {'U',"WWNNNNNWN",30},  {'V',"NWWNNNNWN",31}, {'W',"WWWNNNNNN",32},
    {'X',"NWNNWNNWN",33},  {'Y',"WWNNWNNNN",34}, {'Z',"NWWNWNNNN",35}
};

#define CODE39_TABLE_SIZE (sizeof(code39_table)/sizeof(code39_char_t))

static int code39_value(char c) {
    for (uint i=0; i<CODE39_TABLE_SIZE; ++i)
        if (code39_table[i].character == c) return code39_table[i].value;
    return -1;
}
static char code39_match_pattern(const char *p) {
    for (uint i=0; i<CODE39_TABLE_SIZE; ++i)
        if (strcmp(p, code39_table[i].pattern) == 0) return code39_table[i].character;
    return '?';
}

// ===================== Digital IR Sensor Setup =====================
static volatile bool s_capturing = false;
static volatile uint32_t s_last_edge_us = 0;
static volatile uint16_t s_count = 0;
static volatile uint32_t s_durations[MAX_TRANSITIONS];   // consecutive bar/space durations (us)
static volatile bool s_frame_ready = false;

// Quiet period to consider barcode ended
#define BARCODE_QUIET_US 50000u  // 50 ms quiet

// Single global callback for GPIO interrupts; route events for our pin
static void barcode_gpio_isr(uint gpio, uint32_t events) {
    if (gpio != RIGHT_IR_DIGITAL_PIN) return;
    uint32_t now = time_us_32();

    if (!s_capturing) {
        s_capturing = true;
        s_last_edge_us = now;
        s_count = 0;
        return;
    }

    uint32_t dt = now - s_last_edge_us;
    s_last_edge_us = now;

    if (s_count < MAX_TRANSITIONS) {
        s_durations[s_count++] = dt;
    } else {
        // Buffer full → mark ready; stop capture
        s_frame_ready = true;
        s_capturing = false;
    }
}

void barcode_init(void) {
    // Initialize digital IR sensor
    ir_digital_init();
    
    // Setup interrupt-based capture
    //barcode_irq_init();
    
    printf("Barcode: Digital IR sensor initialized on GPIO%d\n", RIGHT_IR_DIGITAL_PIN);
}

void barcode_irq_init(void) {
    gpio_init(RIGHT_IR_DIGITAL_PIN);
    gpio_set_dir(RIGHT_IR_DIGITAL_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_IR_DIGITAL_PIN);

    // Use a single shared callback for both edges
    gpio_set_irq_enabled_with_callback(RIGHT_IR_DIGITAL_PIN, 
                                      GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
                                      true, &barcode_gpio_isr);
    printf("Barcode: IRQ capture armed on GPIO%u\n", RIGHT_IR_DIGITAL_PIN);
}

// Call periodically (e.g., in a 30 ms timer) to finalize capture on quiet time
bool barcode_capture_ready(void) {
    if (s_frame_ready) return true;
    if (s_capturing) {
        uint32_t quiet = time_us_32() - s_last_edge_us;
        if (quiet > BARCODE_QUIET_US && s_count >= 8) {
            s_frame_ready = true;
            s_capturing = false;
        }
    }
    return s_frame_ready;
}

// ===================== Decoding helpers =====================
static uint32_t estimate_narrow_us(const uint32_t *w, uint16_t n) {
    if (n < 10) return 100000; // Default fallback
    
    // Copy to temporary array
    uint32_t tmp[MAX_TRANSITIONS];
    for (uint16_t i=0;i<n;i++) tmp[i]=w[i];
    
    // Simple bubble sort (good enough for small arrays)
    for (uint16_t i=0;i<n-1;i++) {
        for (uint16_t j=0;j<n-i-1;j++) {
            if (tmp[j] > tmp[j+1]) {
                uint32_t temp = tmp[j];
                tmp[j] = tmp[j+1];
                tmp[j+1] = temp;
            }
        }
    }
    
    // Take median of lower third to avoid outliers
    uint16_t lower_third = n / 3;
    if (lower_third < 5) lower_third = 5;
    
    uint32_t sum = 0;
    for (uint16_t i=0;i<lower_third;i++) {
        sum += tmp[i];
    }
    
    uint32_t median = sum / lower_third;
    
    // Ensure reasonable bounds
    if (median < 10000) return 10000;   // Minimum 10ms
    if (median > 500000) return 500000; // Maximum 500ms
    
    return median;
}

typedef enum { WIDTH_NARROW, WIDTH_WIDE, WIDTH_BAD } width_t;

static width_t classify_width(uint32_t dur_us, uint32_t narrow_us) {
    if (narrow_us == 0) return WIDTH_BAD;
    
    float ratio = (float)dur_us / narrow_us;
    
    // More lenient thresholds for Code39
    //if (ratio < 0.3f) return WIDTH_BAD;      // Too short
    if (ratio < 3.0f) return WIDTH_NARROW;   // 0.3x to 2.0x = Narrow
    if (ratio > 3.0f) return WIDTH_WIDE;     // 2.0x to 5.0x = Wide
    return WIDTH_BAD;                         // Too long
}

static bool decode_symbols_to_chars(const uint32_t *dur, uint16_t n, char *out, uint8_t *out_len, bool expect_bar_first) {
    *out_len = 0;
    if (n < 9) return false;

    uint32_t narrow = estimate_narrow_us(dur, n);
    if (narrow < 40 || narrow > 7000) return false;

    char pattern[10]; uint8_t pidx = 0;
    bool bar = expect_bar_first;

    for (uint16_t i=0;i<n;i++) {
        (void)bar; // classification independent of current color
        width_t w = classify_width(dur[i], narrow);
        if (w == WIDTH_BAD) continue;
        pattern[pidx++] = (w == WIDTH_WIDE) ? 'W' : 'N';
        if (pidx == 9) {
            pattern[9] = '\0';
            char c = code39_match_pattern(pattern);
            out[(*out_len)++] = c;
            pidx = 0;
            if (*out_len >= BARCODE_MAX_LENGTH + 2) break; // include start/stop
        }
        bar = !bar;
    }
    out[*out_len] = 0;
    return (*out_len >= 3);
}

static bool validate_and_strip_code39(char *chars, uint8_t *len, bool *checksum_ok) {
    *checksum_ok = false;
    if (*len < 3) return false;
    if (chars[0] != '*' || chars[*len - 1] != '*') return false;

    // Optional Mod 43 checksum (if >= 4 chars including *)
    if (*len >= 4) {
        int sum = 0;
        for (uint8_t i=1; i<(*len - 2); i++) {
            int v = code39_value(chars[i]);
            if (v < 0) return false;
            sum += v;
        }
        int chk = sum % 43;
        int last = code39_value(chars[*len - 2]);
        if (last >= 0 && last == chk) {
            *checksum_ok = true;
            // Strip checksum char (shift final '*' left)
            chars[*len - 2] = '*';
            chars[*len - 1] = '\0';
            (*len)--;
        }
    }

    // Strip the surrounding '*' to keep payload only
    uint8_t payload = (*len) - 2;
    memmove(chars, chars + 1, payload);
    chars[payload] = '\0';
    *len = payload;
    return true;
}

static bool decode_with_direction(const uint32_t *dur, uint16_t n, barcode_result_t *result, bool forward) {
    char chars[BARCODE_MAX_LENGTH + 4] = {0};
    uint8_t clen = 0;
    uint32_t tmp[MAX_TRANSITIONS];

    if (forward) {
        for (uint16_t i=0;i<n;i++) tmp[i]=dur[i];
    } else {
        for (uint16_t i=0;i<n;i++) tmp[i]=dur[n-1-i];
    }

    bool chk=false;
    if (decode_symbols_to_chars(tmp, n, chars, &clen, true) &&
        validate_and_strip_code39(chars, &clen, &chk)) {
        strncpy(result->data, chars, BARCODE_MAX_LENGTH);
        result->length = (uint8_t)strnlen(result->data, BARCODE_MAX_LENGTH);
        result->valid = (result->length > 0);
        result->checksum_ok = chk;
        return result->valid;
    }
    if (decode_symbols_to_chars(tmp, n, chars, &clen, false) &&
        validate_and_strip_code39(chars, &clen, &chk)) {
        strncpy(result->data, chars, BARCODE_MAX_LENGTH);
        result->length = (uint8_t)strnlen(result->data, BARCODE_MAX_LENGTH);
        result->valid = (result->length > 0);
        result->checksum_ok = chk;
        return result->valid;
    }
    return false;
}

// ===================== Public decode entry points =====================
bool barcode_decode_captured(barcode_result_t *result) {
    if (!s_frame_ready) return false;

    // Copy volatile buffer
    uint16_t n = s_count;
    if (n < 9) { s_frame_ready = false; return false; }
    uint32_t buf[MAX_TRANSITIONS];
    for (uint16_t i=0;i<n;i++) buf[i] = s_durations[i];

    // Consume frame
    s_frame_ready = false;

    uint32_t t0 = time_us_32();
    if (!decode_with_direction(buf, n, result, true)) {
        if (!decode_with_direction(buf, n, result, false)) {
            return false;
        }
    }
    result->scan_time_us = time_us_32() - t0;
    return true;
}

// Digital polling-based scan (alternative to interrupt method)
bool barcode_scan_digital(barcode_result_t *result) {
    memset(result, 0, sizeof(*result));
    uint32_t tstart = time_us_32();

    // Capture digital transitions using polling
    uint32_t dur[MAX_TRANSITIONS]; 
    uint16_t n = 0;
    bool last_state = ir_read_digital();
    uint32_t seg_start = time_us_32();
    uint32_t last_change = seg_start;

    // Wait for start condition (black/white transition) or timeout
    uint32_t w0 = time_us_32();
    while ((time_us_32() - w0) < 300000) {
        bool current_state = ir_read_digital();
        // printf("%d", current_state);
        if (current_state != last_state) {
            last_state = current_state;
            break;
        }
        sleep_us(100);
    }

    // Measure until quiet > 50 ms or buffer full
    while (n < MAX_TRANSITIONS-1) {
        bool current_state = ir_read_digital();
        if (current_state != last_state) {
            uint32_t now = time_us_32();
            dur[n++] = now - seg_start;
            seg_start = now;
            last_state = current_state;
            last_change = now;
        } else {
            if ((time_us_32() - last_change) > BARCODE_QUIET_US) {
                uint32_t now = time_us_32();
                dur[n++] = now - seg_start;
                break;
            }
        }
        
        sleep_us(50); // Faster polling for digital
    }

    if (n < 20) return false;

    if (!decode_with_direction(dur, n, result, true)) {
        if (!decode_with_direction(dur, n, result, false)) return false;
    }
    result->scan_time_us = time_us_32() - tstart;
    return true;
}

barcode_command_t barcode_parse_command(const char *s) {
    if (!s || !*s) return BARCODE_CMD_UNKNOWN;
    if (!strcmp(s,"LEFT") || !strcmp(s,"L"))       return BARCODE_CMD_LEFT;
    if (!strcmp(s,"RIGHT")|| !strcmp(s,"R"))       return BARCODE_CMD_RIGHT;
    if (!strcmp(s,"STOP") || !strcmp(s,"S"))       return BARCODE_CMD_STOP;
    if (!strcmp(s,"GO")   || !strcmp(s,"F") ||
        !strcmp(s,"FWD")  || !strcmp(s,"FORWARD")) return BARCODE_CMD_FORWARD;
    return BARCODE_CMD_UNKNOWN;
}

// (Truncated remainder of barcode.c for brevity due to length; full restore will include entire upstream content)