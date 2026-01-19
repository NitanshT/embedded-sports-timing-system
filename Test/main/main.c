#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_err.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "soc/gpio_reg.h"

#include "driver/rmt.h"

// PIN CONFIG

// I2C (OLED + AM2320)
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    21
#define I2C_SCL_GPIO    22
#define I2C_FREQ_HZ     50000

// OLED addresses
#define OLED_ADDR_1     0x3C
#define OLED_ADDR_2     0x3D

// OLED geometry
#define OLED_W 128
#define OLED_H 64

// AM2320 address
#define AM2320_ADDR     0x5C

// Ultrasonic
#define TRIG_GPIO       GPIO_NUM_18
#define ECHO_GPIO       GPIO_NUM_19

// Buzzer (PS1240)
#define BUZZER_GPIO     GPIO_NUM_14

// RGB LED (COMMON ANODE -> active-low per channel)
// User intensity is 0..255 where 255 = fully ON (bright).
#define RGB_RED_GPIO    GPIO_NUM_16
#define RGB_GREEN_GPIO  GPIO_NUM_13
#define RGB_BLUE_GPIO   GPIO_NUM_4

// 74HC595 chain -> 2x 7-seg (minutes)
#define SHIFT_DATA      GPIO_NUM_23
#define SHIFT_CLOCK     GPIO_NUM_5
#define SHIFT_LATCH     GPIO_NUM_17

// Stepper (28BYJ-48 + ULN2003) direct GPIO
#define IN1             GPIO_NUM_27
#define IN2             GPIO_NUM_26
#define IN3             GPIO_NUM_25
#define IN4             GPIO_NUM_33

// IR Receiver
#define IR_GPIO GPIO_NUM_32

// 1 us tick (80 MHz / 80)
#define RMT_CLK_DIV 80
#define RMT_RX_CH   RMT_CHANNEL_0

// NEC timings (us)
#define NEC_HDR_MARK   9000
#define NEC_HDR_SPACE  4500
#define NEC_BIT_MARK    560
#define NEC_0_SPACE     560
#define NEC_1_SPACE    1690

// forward declarations
static void ir_on_nec(uint8_t addr, uint8_t cmd, bool is_repeat);
static void handle_ir_events(uint8_t oled_addr);
static void start_countdown_and_go(uint8_t oled_addr);
static void request_stop(void);
static void request_finish(void);
static void request_reset(void);

// IR helpers
static bool ir_cmd_is_digit(uint8_t cmd);

// Buzzer module
static void buzzer_init(void);
static void buzzer_start_tone(uint32_t freq_hz, uint8_t duty_percent);
static void buzzer_stop(void);
static void buzzer_beep(uint32_t freq_hz, uint32_t ms);
static void buzzer_pattern_warning(void);
static void buzzer_pattern_finish(void);
static void buzzer_show_warning_then_continue(uint8_t oled_addr);

// RGB module (LEDC PWM)
static void rgb_init(void);
static void rgb_set(uint8_t r, uint8_t g, uint8_t b);
static void rgb_off(void);
static void rgb_set_mode_off(void);
static void rgb_set_mode_solid(uint8_t r, uint8_t g, uint8_t b);
static void rgb_set_mode_blink(uint8_t r, uint8_t g, uint8_t b, uint32_t period_ms, uint8_t duty_cycle_percent);

// OLED helpers used by countdown/UI
static void fb_clear(void);
static void fb_draw_text(int x, int y, const char *s);
static void fb_draw_text_scaled(int x, int y, const char *s, int scale);
static void fb_draw_text_centered(int y, const char *s);
static void fb_draw_text_centered_scaled(int y, const char *s, int scale);
static void fb_draw_icon_rewind(int x, int y);
static void oled_flush_safe(uint8_t oled_addr);

typedef enum {
    UI_OFF,
    UI_READY,
    UI_COUNTDOWN,
    UI_ARMED,
    UI_RUNNING,
    UI_STOPPED,
    UI_FINISHED,
    UI_SETLAPS
} ui_state_t;

static ui_state_t ui_state = UI_READY;
static bool system_enabled = false;          // starts OFF until POWER pressed
static int lap_target = 5;
static int lap_count = 0;
static int lap_edit_value = 0;
static bool lap_edit_active = false;
static int64_t elapsed_accum_us = 0;         // accumulated time when stopped
static int64_t go_time_us = 0;               // time when GO shown
static int64_t reaction_us = 0;
static bool reaction_valid = false;

// Warning thresholds (AM2320)
#define TEMP_WARN_C      30.0f
#define RH_WARN_PERCENT  40.0f

// Latest sensor reading for START warning gate
static float g_last_temp_c = 0.0f;
static float g_last_rh_percent = 0.0f;
static bool  g_have_sensor = false;

typedef struct {
    uint8_t cmd;
    uint8_t addr;
    bool    is_repeat;
} ir_event_t;

static QueueHandle_t ir_queue;

static volatile bool timer_running = false;
static volatile int64_t timer_start_us = 0;
static volatile bool stepper_home_request = false;

// RGB (LEDC PWM, common anode = active-low)

typedef enum {
    RGB_MODE_OFF = 0,
    RGB_MODE_SOLID,
    RGB_MODE_BLINK,
} rgb_mode_t;

typedef struct {
    rgb_mode_t mode;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint32_t period_ms;
    uint8_t duty_cycle_percent;
} rgb_mode_state_t;
// portMUX initializes a portMUX variable to handle interrupt services and prevent data corruption
static portMUX_TYPE g_rgb_mux = portMUX_INITIALIZER_UNLOCKED;
static rgb_mode_state_t g_rgb_state = { .mode = RGB_MODE_OFF, .r = 0, .g = 0, .b = 0, .period_ms = 500, .duty_cycle_percent = 50 };
static TaskHandle_t g_rgb_task = NULL;
static bool g_rgb_inited = false;

// Improve IR repeat handling 
static uint8_t  g_last_cmd  = 0;
static uint8_t  g_last_addr = 0;
static int64_t  g_last_cmd_us = 0;

// IR reliability improvements

#ifndef IR_DEBUG_VERBOSE
#define IR_DEBUG_VERBOSE 0
#endif

typedef struct {
    uint32_t frames_received;
    uint32_t decoded_ok;
    uint32_t decoded_fail;
    uint32_t repeats_received;
    uint32_t repeats_used;
    uint32_t queue_drops;
} ir_stats_t;

static ir_stats_t g_ir_stats = {0};
static int64_t g_ir_last_diag_us = 0;
static int64_t g_ir_last_repeat_enq_us = 0;

// Repeat behavior: accept repeats within 200ms of last valid cmd, and rate-limit to 10Hz.
#define IR_REPEAT_WINDOW_US  200000
#define IR_REPEAT_RATE_US    100000

static inline bool in_range_us(int v, int target, int tol)
{
    return (v >= (target - tol)) && (v <= (target + tol));
}

static inline bool in_range(int v, int target, int tol) {
    return (v > (target - tol)) && (v < (target + tol));
}

static inline int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

typedef enum {
    NEC_DECODE_OK = 0,
    NEC_DECODE_REPEAT,
    NEC_DECODE_FAIL,
} nec_decode_result_t;

typedef enum {
    NEC_FAIL_NONE = 0,
    NEC_FAIL_NO_HEADER,
    NEC_FAIL_BAD_BIT,
    NEC_FAIL_BAD_INVERSE,
} nec_fail_reason_t;

typedef struct {
    uint32_t dur;
    uint8_t level;
} ir_seg_t;

static int ir_build_segments(const rmt_item32_t *it, int n_items, ir_seg_t *out, int out_cap)
{
    int n = 0;
    for (int i = 0; i < n_items; i++) {
        uint32_t d0 = it[i].duration0;
        uint32_t d1 = it[i].duration1;
        uint8_t  l0 = it[i].level0 ? 1 : 0;
        uint8_t  l1 = it[i].level1 ? 1 : 0;

        if (d0) {
            if (n > 0 && out[n - 1].level == l0) out[n - 1].dur += d0;
            else if (n < out_cap) out[n++] = (ir_seg_t){ .dur = d0, .level = l0 };
        }
        if (d1) {
            if (n > 0 && out[n - 1].level == l1) out[n - 1].dur += d1;
            else if (n < out_cap) out[n++] = (ir_seg_t){ .dur = d1, .level = l1 };
        }
    }
    return n;
}

static nec_decode_result_t nec_try_decode(const ir_seg_t *s, int n, uint8_t mark_level,
                                         uint16_t *addr, uint8_t *cmd, bool *is_repeat,
                                         nec_fail_reason_t *fail_reason, bool relaxed)
{
    const int tol_hdr_mark  = relaxed ? 2600 : 2000;
    const int tol_hdr_space = relaxed ? 1400 : 1100;
    const int tol_rep_space = relaxed ? 900  : 750;
    const int tol_bit_mark  = relaxed ? 320  : 260;
    const int tol_0_space   = relaxed ? 320  : 260;
    const int tol_1_space   = relaxed ? 600  : 500;

    // Find header (skip leading noise)
    int k = -1;
    for (int i = 0; i + 1 < n; i++) {
        if (s[i].level == mark_level && in_range_us((int)s[i].dur, NEC_HDR_MARK, tol_hdr_mark) &&
            s[i + 1].level != mark_level) {
            int sp = (int)s[i + 1].dur;
            if (in_range_us(sp, NEC_HDR_SPACE, tol_hdr_space) || in_range_us(sp, 2250, tol_rep_space)) {
                k = i;
                break;
            }
        }
    }
    if (k < 0) {
        if (fail_reason) *fail_reason = NEC_FAIL_NO_HEADER;
        return NEC_DECODE_FAIL;
    }

    // Repeat frame: 9ms mark + 2.25ms space + 560us mark
    if (in_range_us((int)s[k + 1].dur, 2250, tol_rep_space)) {
        if (k + 2 < n && s[k + 2].level == mark_level && in_range_us((int)s[k + 2].dur, NEC_BIT_MARK, tol_bit_mark)) {
            if (is_repeat) *is_repeat = true;
            if (fail_reason) *fail_reason = NEC_FAIL_NONE;
            return NEC_DECODE_REPEAT;
        }

        if (is_repeat) *is_repeat = true;
        if (fail_reason) *fail_reason = NEC_FAIL_NONE;
        return NEC_DECODE_REPEAT;
    }

    // Normal frame
    uint32_t data = 0;
    int idx = k + 2;
    for (int bit = 0; bit < 32; bit++) {
        if (idx + 1 >= n) {
            if (fail_reason) *fail_reason = NEC_FAIL_BAD_BIT;
            return NEC_DECODE_FAIL;
        }

        if (s[idx].level != mark_level || !in_range_us((int)s[idx].dur, NEC_BIT_MARK, tol_bit_mark)) {
            if (fail_reason) *fail_reason = NEC_FAIL_BAD_BIT;
            return NEC_DECODE_FAIL;
        }
        if (s[idx + 1].level == mark_level) {
            if (fail_reason) *fail_reason = NEC_FAIL_BAD_BIT;
            return NEC_DECODE_FAIL;
        }
        int sp = (int)s[idx + 1].dur;
        if (in_range_us(sp, NEC_0_SPACE, tol_0_space)) {
            // 0
        } else if (in_range_us(sp, NEC_1_SPACE, tol_1_space)) {
            data |= (1UL << bit);
        } else {
            if (fail_reason) *fail_reason = NEC_FAIL_BAD_BIT;
            return NEC_DECODE_FAIL;
        }

        idx += 2;
    }

    uint8_t a0 = (data >> 0) & 0xFF;
    uint8_t a1 = (data >> 8) & 0xFF;
    uint8_t c0 = (data >> 16) & 0xFF;
    uint8_t c1 = (data >> 24) & 0xFF;

    if ((uint8_t)(a0 ^ a1) != 0xFF || (uint8_t)(c0 ^ c1) != 0xFF) {
        if (fail_reason) *fail_reason = NEC_FAIL_BAD_INVERSE;
        return NEC_DECODE_FAIL;
    }

    if (addr) *addr = a0;
    if (cmd)  *cmd  = c0;
    if (is_repeat) *is_repeat = false;
    if (fail_reason) *fail_reason = NEC_FAIL_NONE;
    return NEC_DECODE_OK;
}

static nec_decode_result_t nec_decode_items_robust(const rmt_item32_t *it, int n_items,
                                                   uint16_t *addr, uint8_t *cmd,
                                                   bool *is_repeat, nec_fail_reason_t *fail_reason)
{
    // Worst-case each RMT item contributes two segments; we also merge adjacent same-level segments.
    // Keep this comfortably sized to avoid truncation.
    ir_seg_t segs[600];
    int n = ir_build_segments(it, n_items, segs, (int)(sizeof(segs) / sizeof(segs[0])));
    if (n < 2) {
        if (fail_reason) *fail_reason = NEC_FAIL_NO_HEADER;
        return NEC_DECODE_FAIL;
    }

    // Try both possible polarities (demodulated receivers may invert).
    nec_decode_result_t r;
    r = nec_try_decode(segs, n, 0, addr, cmd, is_repeat, fail_reason, false);
    if (r != NEC_DECODE_FAIL) return r;
    r = nec_try_decode(segs, n, 1, addr, cmd, is_repeat, fail_reason, false);
    if (r != NEC_DECODE_FAIL) return r;

    // Software retry: relaxed tolerances
    r = nec_try_decode(segs, n, 0, addr, cmd, is_repeat, fail_reason, true);
    if (r != NEC_DECODE_FAIL) return r;
    r = nec_try_decode(segs, n, 1, addr, cmd, is_repeat, fail_reason, true);
    return r;
}

static void ir_init(void)
{
    rmt_config_t cfg = {
        .rmt_mode = RMT_MODE_RX,
        .channel = RMT_RX_CH,
        .gpio_num = IR_GPIO,
        .clk_div = RMT_CLK_DIV,
        .mem_block_num = 4,
        .rx_config.filter_en = true,
        // Filter glitches < ~100us; keep 560us marks/spaces intact.
        .rx_config.filter_ticks_thresh = 100,
        // End of frame after ~15ms idle (longer than 9ms/4.5ms/1.69ms intra-frame spaces).
        .rx_config.idle_threshold = 15000,
    };

    ESP_ERROR_CHECK(rmt_config(&cfg));
    gpio_set_pull_mode(IR_GPIO, GPIO_PULLUP_ONLY);
    // Larger ringbuffer helps avoid drops during bursts / main loop delays.
    ESP_ERROR_CHECK(rmt_driver_install(cfg.channel, 8192, 0)); // rx ringbuffer bytes
    ESP_ERROR_CHECK(rmt_rx_start(cfg.channel, true));
}

static void ir_task(void *arg)
{
    (void)arg;
    RingbufHandle_t rb = NULL;
    rmt_get_ringbuf_handle(RMT_RX_CH, &rb);

    while (1) {
        size_t rx_size = 0;
        rmt_item32_t *items = (rmt_item32_t *)xRingbufferReceive(rb, &rx_size, pdMS_TO_TICKS(1000));
        if (!items) continue;

        int n_items = rx_size / sizeof(rmt_item32_t);

        g_ir_stats.frames_received++;

        uint16_t addr = 0;
        uint8_t cmd = 0;

        int64_t now_us = esp_timer_get_time();
        bool is_repeat = false;
        nec_fail_reason_t fail_reason = NEC_FAIL_NONE;
        nec_decode_result_t res = nec_decode_items_robust(items, n_items, &addr, &cmd, &is_repeat, &fail_reason);

        if (res == NEC_DECODE_OK) {
            g_ir_stats.decoded_ok++;
            g_last_addr = (uint8_t)addr;
            g_last_cmd = cmd;
            g_last_cmd_us = now_us;
            ir_on_nec((uint8_t)addr, cmd, false);
        } else if (res == NEC_DECODE_REPEAT) {
            g_ir_stats.repeats_received++;
            // NOTE: We intentionally do NOT generate command events from repeat frames.
            // For this application, a long-press should not spam actions (POWER/START/EQ/digits/etc).
            // Single presses should be handled by the full NEC frame decode path.
        } else {
            g_ir_stats.decoded_fail++;
            if (IR_DEBUG_VERBOSE) {
                if (g_ir_last_diag_us == 0 || (now_us - g_ir_last_diag_us) > 1500000) {
                    g_ir_last_diag_us = now_us;
                    const char *reason = (fail_reason == NEC_FAIL_NO_HEADER) ? "no_header" :
                                         (fail_reason == NEC_FAIL_BAD_BIT) ? "bad_bit" :
                                         (fail_reason == NEC_FAIL_BAD_INVERSE) ? "bad_inverse" : "fail";
                    printf("IR decode fail (%s): items=%d\n", reason, n_items);
                    printf("IR stats: frames=%u ok=%u fail=%u rep_rx=%u rep_used=%u drops=%u\n",
                           (unsigned)g_ir_stats.frames_received,
                           (unsigned)g_ir_stats.decoded_ok,
                           (unsigned)g_ir_stats.decoded_fail,
                           (unsigned)g_ir_stats.repeats_received,
                           (unsigned)g_ir_stats.repeats_used,
                           (unsigned)g_ir_stats.queue_drops);
                }
            }
        }


        vRingbufferReturnItem(rb, (void*)items);
    }
}

// IR command handler
#define CMD_POWER   0x45
#define CMD_CLEAR   0x44
#define CMD_STOP    0x47   // FUNC/STOP
#define CMD_START   0x40
#define CMD_RESET   0x43
#define CMD_EQ      0x19

// Digits (Elegoo-style mapping
#define CMD_0 0x16
#define CMD_1 0x0C
#define CMD_2 0x18
#define CMD_3 0x5E
#define CMD_4 0x08
#define CMD_5 0x1C
#define CMD_6 0x5A
#define CMD_7 0x42
#define CMD_8 0x52
#define CMD_9 0x4A

static int nec_cmd_to_digit(uint8_t cmd)
{
    switch (cmd) {
        case CMD_0: return 0;
        case CMD_1: return 1;
        case CMD_2: return 2;
        case CMD_3: return 3;
        case CMD_4: return 4;
        case CMD_5: return 5;
        case CMD_6: return 6;
        case CMD_7: return 7;
        case CMD_8: return 8;
        case CMD_9: return 9;
        default:    return -1;
    }
}

static bool ir_cmd_is_digit(uint8_t cmd)
{
    return nec_cmd_to_digit(cmd) >= 0;
}


static void ir_on_nec(uint8_t addr, uint8_t cmd, bool is_repeat)
{
    // Debounce only for repeat-sourced events; allow distinct presses through immediately.
    static uint8_t last_repeat_cmd = 0;
    static int64_t last_repeat_us = 0;
    int64_t now = esp_timer_get_time();
    if (is_repeat) {
        if (cmd == last_repeat_cmd && (now - last_repeat_us) < 60000) {
            return;
        }
        last_repeat_cmd = cmd;
        last_repeat_us = now;
    }

    if (IR_DEBUG_VERBOSE) {
        printf("IR NEC: addr=0x%02X cmd=0x%02X%s\n", addr, cmd, is_repeat ? " (rep)" : "");
    }

    ir_event_t ev = { .addr = addr, .cmd = cmd, .is_repeat = is_repeat };
    if (ir_queue) {
        if (xQueueSend(ir_queue, &ev, pdMS_TO_TICKS(10)) != pdTRUE) {
            g_ir_stats.queue_drops++;
            if (IR_DEBUG_VERBOSE && (now - g_ir_last_diag_us) > 1000000) {
                g_ir_last_diag_us = now;
                printf("IR queue full (drops=%u)\n", (unsigned)g_ir_stats.queue_drops);
            }
        }
    }
}

static void handle_ir_events(uint8_t oled_addr)
{
    ir_event_t ev;
    while (xQueueReceive(ir_queue, &ev, 0) == pdTRUE) {
        printf("IR EVENT cmd=0x%02X addr=0x%02X%s\n", ev.cmd, ev.addr, ev.is_repeat ? " (rep)" : "");

        // Action-level debounce (prevents long-press / repeat frames from toggling/retriggering).
        // Uses *processing time* so queued bursts collapse into one action.
        static int64_t last_power_us = 0;
        static int64_t last_reset_us = 0;
        static int64_t last_start_us = 0;
        int64_t now_us = esp_timer_get_time();

        // POWER toggles system ON/OFF
        if (ev.cmd == CMD_POWER) {
            if (last_power_us != 0 && (now_us - last_power_us) < 400000) {
                continue;
            }
            last_power_us = now_us;

            system_enabled = !system_enabled;
            if (!system_enabled) {
                // Turning OFF: stop safely + reset runtime state
                request_reset();
                ui_state = UI_OFF;
            } else {
                // Turning ON: go to READY (keep lap_target)
                request_reset();
                ui_state = UI_READY;
            }
            continue;
        }

        // Ignore everything else if system is off
        if (!system_enabled) {
            ui_state = UI_OFF;
            continue;
        }

        // RESET always available when enabled
        if (ev.cmd == CMD_RESET) {
            if (last_reset_us != 0 && (now_us - last_reset_us) < 250000) {
                continue;
            }
            last_reset_us = now_us;
            request_reset();
            continue;
        }

        // Enter/exit lap setting mode
        if (ev.cmd == CMD_EQ) {
            if (ui_state != UI_SETLAPS) {
                ui_state = UI_SETLAPS;
                lap_edit_value = lap_target;
                lap_edit_active = false;
            } else {
                lap_target = clamp_int(lap_edit_value, 1, 99);
                ui_state = UI_READY;
            }
            continue;
        }

        // Clear/correct key only in lap setting mode
        if (ui_state == UI_SETLAPS && ev.cmd == CMD_CLEAR) {
            if (lap_edit_active) {
                lap_edit_value = 0;
                lap_edit_active = false;
            }
            continue;
        }

        // Digits only do something in UI_SETLAPS
        int d = nec_cmd_to_digit(ev.cmd);
        if (ui_state == UI_SETLAPS && d >= 0) {
            if (!lap_edit_active) {
                lap_edit_active = true;
                lap_edit_value = d;
            } else {
                int next = lap_edit_value * 10 + d;
                if (next > 99) next = 99;
                lap_edit_value = next;
            }
            continue;
        }

        // START: countdown 3..1 then GO; arm system (timer starts on ultrasonic)
        if (ev.cmd == CMD_START) {
            if (ui_state == UI_READY || ui_state == UI_STOPPED) {
                if (last_start_us != 0 && (now_us - last_start_us) < 250000) {
                    continue;
                }
                last_start_us = now_us;
                buzzer_show_warning_then_continue(oled_addr);
                start_countdown_and_go(oled_addr);
            }
            continue;
        }

        // STOP: stop race
        if (ev.cmd == CMD_STOP) {
            request_stop();
            continue;
        }
    }
}

static void request_stop(void)
{
    int64_t now = esp_timer_get_time();
    if (timer_running) {
        elapsed_accum_us += (now - timer_start_us);
    }
    timer_running = false;
    stepper_home_request = true;
    ui_state = UI_STOPPED;

    // STOPPED: show a calm blue
    rgb_set_mode_solid(0, 0, 80);
}

static void request_finish(void)
{
    int64_t now = esp_timer_get_time();
    if (timer_running) {
        elapsed_accum_us += (now - timer_start_us);
    }
    timer_running = false;
    stepper_home_request = true;
    ui_state = UI_FINISHED;

    // FINISHED: magenta
    rgb_set_mode_solid(255, 0, 255);
}

static void request_reset(void)
{
    timer_running = false;
    timer_start_us = 0;
    elapsed_accum_us = 0;
    lap_count = 0;
    lap_edit_value = 0;
    lap_edit_active = false;
    reaction_valid = false;
    reaction_us = 0;
    go_time_us = 0;
    stepper_home_request = true;
    buzzer_stop();
    ui_state = system_enabled ? UI_READY : UI_OFF;

    if (!system_enabled || ui_state == UI_OFF) {
        rgb_set_mode_off();
    } else {
        // READY: dim blue
        rgb_set_mode_solid(0, 0, 60);
    }
}

// BUZZER (LEDC PWM)

#define BUZZER_DUTY_DEFAULT_PERCENT  33

static const ledc_mode_t      BUZZER_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t     BUZZER_LEDC_TIMER = LEDC_TIMER_0;
static const ledc_channel_t   BUZZER_LEDC_CH = LEDC_CHANNEL_0;
static const ledc_timer_bit_t BUZZER_DUTY_RES = LEDC_TIMER_10_BIT;

static void buzzer_init(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode = BUZZER_LEDC_MODE,
        .timer_num = BUZZER_LEDC_TIMER,
        .duty_resolution = BUZZER_DUTY_RES,
        .freq_hz = 3000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    ledc_channel_config_t ccfg = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel = BUZZER_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BUZZER_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ccfg));

    buzzer_stop();
}

static void buzzer_start_tone(uint32_t freq_hz, uint8_t duty_percent)
{
    if (freq_hz == 0) {
        buzzer_stop();
        return;
    }
    if (duty_percent > 100) duty_percent = 100;
    if (duty_percent == 0) {
        buzzer_stop();
        return;
    }

    ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, freq_hz);

    uint32_t max_duty = (1U << BUZZER_DUTY_RES) - 1U;
    uint32_t duty = (max_duty * (uint32_t)duty_percent) / 100U;
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CH, duty);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CH);
}

static void buzzer_stop(void)
{
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CH, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CH);
}

static void buzzer_beep(uint32_t freq_hz, uint32_t ms)
{
    buzzer_start_tone(freq_hz, BUZZER_DUTY_DEFAULT_PERCENT);
    vTaskDelay(pdMS_TO_TICKS(ms));
    buzzer_stop();
}

static void buzzer_pattern_warning(void)
{
    // 3 short beeps
    for (int i = 0; i < 3; i++) {
        buzzer_beep(3000, 100);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void buzzer_pattern_finish(void)
{
    // Two-tone finish
    buzzer_beep(2500, 250);
    vTaskDelay(pdMS_TO_TICKS(100));
    buzzer_beep(3500, 250);
}

static void buzzer_show_warning_then_continue(uint8_t oled_addr)
{
    if (!g_have_sensor) return;
    bool warn = (g_last_temp_c >= TEMP_WARN_C) || (g_last_rh_percent >= RH_WARN_PERCENT);
    if (!warn) return;

    // Snapshot current RGB mode and blink red while warning is shown.
    rgb_mode_state_t prev;
    portENTER_CRITICAL(&g_rgb_mux);
    prev = g_rgb_state;
    portEXIT_CRITICAL(&g_rgb_mux);

    rgb_set_mode_blink(255, 0, 0, 500 /* 2Hz */, 50);

    fb_clear();
    fb_draw_text_centered(0, "SPORT TIMER");
    fb_draw_text_centered_scaled(14, "WARNING", 2);
    fb_draw_text_centered_scaled(28, "TRACK UNSAFE", 1);
    {
        char th[32];
        snprintf(th, sizeof(th), "T %.1fC H %.1f%%", g_last_temp_c, g_last_rh_percent);
        fb_draw_text_centered(40, th);
    }
    fb_draw_text_centered(56, "STARTING...");
    oled_flush_safe(oled_addr);

    buzzer_pattern_warning();
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Restore RGB mode after warning
    portENTER_CRITICAL(&g_rgb_mux);
    g_rgb_state = prev;
    portEXIT_CRITICAL(&g_rgb_mux);
}

// RGB (LEDC PWM)

static const ledc_mode_t      RGB_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t     RGB_LEDC_TIMER = LEDC_TIMER_1;
static const ledc_timer_bit_t RGB_DUTY_RES = LEDC_TIMER_10_BIT;
static const uint32_t         RGB_FREQ_HZ = 4000;

static const ledc_channel_t RGB_CH_R = LEDC_CHANNEL_1;
static const ledc_channel_t RGB_CH_G = LEDC_CHANNEL_2;
static const ledc_channel_t RGB_CH_B = LEDC_CHANNEL_3;

static void rgb_apply_hw(uint8_t r, uint8_t g, uint8_t b)
{
    // COMMON ANODE LED (active-low): LED is ON when GPIO is LOW.
    // LEDC duty controls HIGH-time; so we invert duty:
    // user 0..255 where 255=fully ON => duty=0 (always low)
    // user 0 => duty=max (always high)
    const uint32_t max_duty = (1U << RGB_DUTY_RES) - 1U;

    uint32_t dr_on = (max_duty * (uint32_t)r) / 255U;
    uint32_t dg_on = (max_duty * (uint32_t)g) / 255U;
    uint32_t db_on = (max_duty * (uint32_t)b) / 255U;

    uint32_t dr = max_duty - dr_on;
    uint32_t dg = max_duty - dg_on;
    uint32_t db = max_duty - db_on;

    ledc_set_duty(RGB_LEDC_MODE, RGB_CH_R, dr);
    ledc_update_duty(RGB_LEDC_MODE, RGB_CH_R);
    ledc_set_duty(RGB_LEDC_MODE, RGB_CH_G, dg);
    ledc_update_duty(RGB_LEDC_MODE, RGB_CH_G);
    ledc_set_duty(RGB_LEDC_MODE, RGB_CH_B, db);
    ledc_update_duty(RGB_LEDC_MODE, RGB_CH_B);
}

static void rgb_task(void *arg)
{
    (void)arg;
    while (1) {
        rgb_mode_state_t st;
        portENTER_CRITICAL(&g_rgb_mux);
        st = g_rgb_state;
        portEXIT_CRITICAL(&g_rgb_mux);

        if (!g_rgb_inited) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (st.mode == RGB_MODE_OFF) {
            rgb_apply_hw(0, 0, 0);
        } else if (st.mode == RGB_MODE_SOLID) {
            rgb_apply_hw(st.r, st.g, st.b);
        } else {
            uint32_t period = (st.period_ms == 0) ? 500 : st.period_ms;
            uint32_t on_ms = (period * (uint32_t)st.duty_cycle_percent) / 100U;
            if (on_ms > period) on_ms = period;

            uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
            uint32_t phase = (uint32_t)(now_ms % period);
            if (phase < on_ms) rgb_apply_hw(st.r, st.g, st.b);
            else               rgb_apply_hw(0, 0, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void rgb_init(void)
{
    const uint32_t max_duty = (1U << RGB_DUTY_RES) - 1U;

    ledc_timer_config_t tcfg = {
        .speed_mode = RGB_LEDC_MODE,
        .timer_num = RGB_LEDC_TIMER,
        .duty_resolution = RGB_DUTY_RES,
        .freq_hz = RGB_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    ledc_channel_config_t rcfg = {
        .gpio_num = RGB_RED_GPIO,
        .speed_mode = RGB_LEDC_MODE,
        .channel = RGB_CH_R,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = RGB_LEDC_TIMER,
        // For common-anode, OFF means output HIGH all the time.
        .duty = max_duty,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    ledc_channel_config_t gcfg = rcfg;
    gcfg.gpio_num = RGB_GREEN_GPIO;
    gcfg.channel = RGB_CH_G;
    ledc_channel_config_t bcfg = rcfg;
    bcfg.gpio_num = RGB_BLUE_GPIO;
    bcfg.channel = RGB_CH_B;

    ESP_ERROR_CHECK(ledc_channel_config(&rcfg));
    ESP_ERROR_CHECK(ledc_channel_config(&gcfg));
    ESP_ERROR_CHECK(ledc_channel_config(&bcfg));

    g_rgb_inited = true;
    rgb_apply_hw(0, 0, 0);
    rgb_off();

    if (g_rgb_task == NULL) {
        xTaskCreate(rgb_task, "rgb_task", 2048, NULL, 5, &g_rgb_task);
    }
}

static void rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
    rgb_set_mode_solid(r, g, b);
}

static void rgb_off(void)
{
    rgb_set_mode_off();
}

static void rgb_set_mode_off(void)
{
    portENTER_CRITICAL(&g_rgb_mux);
    g_rgb_state.mode = RGB_MODE_OFF;
    g_rgb_state.r = 0;
    g_rgb_state.g = 0;
    g_rgb_state.b = 0;
    portEXIT_CRITICAL(&g_rgb_mux);
}

static void rgb_set_mode_solid(uint8_t r, uint8_t g, uint8_t b)
{
    portENTER_CRITICAL(&g_rgb_mux);
    g_rgb_state.mode = RGB_MODE_SOLID;
    g_rgb_state.r = r;
    g_rgb_state.g = g;
    g_rgb_state.b = b;
    portEXIT_CRITICAL(&g_rgb_mux);
}

static void rgb_set_mode_blink(uint8_t r, uint8_t g, uint8_t b, uint32_t period_ms, uint8_t duty_cycle_percent)
{
    if (duty_cycle_percent > 100) duty_cycle_percent = 100;
    if (period_ms < 50) period_ms = 50;

    portENTER_CRITICAL(&g_rgb_mux);
    g_rgb_state.mode = RGB_MODE_BLINK;
    g_rgb_state.r = r;
    g_rgb_state.g = g;
    g_rgb_state.b = b;
    g_rgb_state.period_ms = period_ms;
    g_rgb_state.duty_cycle_percent = duty_cycle_percent;
    portEXIT_CRITICAL(&g_rgb_mux);
}

static void start_countdown_and_go(uint8_t oled_addr)
{
    if (!system_enabled) {
        ui_state = UI_OFF;
        rgb_set_mode_off();
        return;
    }

    ui_state = UI_COUNTDOWN;

    fb_clear();
    fb_draw_text_centered(0, "SPORT TIMER");
    fb_draw_text_centered(16, "STARTING IN");
    oled_flush_safe(oled_addr);

    for (int i = 3; i >= 1; i--) {
        // Countdown colors
        if (i == 3) {
            rgb_set_mode_solid(255, 0, 0);          // RED
        } else if (i == 2) {
            rgb_set_mode_solid(255, 140, 0);        // YELLOW (R+G)
        } else {
            rgb_set_mode_solid(255, 220, 0);        // YELLOW, slightly greener/brighter
        }

        char line[2] = { (char)('0' + i), '\0' };
        const int scale_digit = 4;
        const int top_reserved = 24;
        const int avail_h = OLED_H - top_reserved; // Check how many height pixels are available
        const int digit_h = 7 * scale_digit; // Make symbol larger than initial size
        const int y_digit = top_reserved + (avail_h - digit_h) / 2;

        // Width for a single glyph is 5 columns, scaled
        const int digit_w = 5 * scale_digit;
        const int x_digit = (OLED_W - digit_w) / 2;

        fb_clear();
        fb_draw_text_centered(0, "SPORT TIMER");
        fb_draw_text_centered(16, "STARTING IN");
        fb_draw_text_scaled(x_digit, y_digit, line, scale_digit);
        oled_flush_safe(oled_addr);
        buzzer_beep(2600, 80);
        vTaskDelay(pdMS_TO_TICKS(920));
    }

    fb_clear();
    fb_draw_text_centered(0, "SPORT TIMER");

    {
        const char *go = "GO!"; // write to address of go
        const int scale_go = 3;
        const int top_reserved = 24;
        const int avail_h = OLED_H - top_reserved;
        const int go_h = 7 * scale_go;
        const int y_go = top_reserved + (avail_h - go_h) / 2;
        const int go_w = ((int)strlen(go) * 6 - 1) * scale_go;
        const int x_go = (OLED_W - go_w) / 2;
        fb_draw_text_scaled(x_go, y_go, go, scale_go);
    }

    oled_flush_safe(oled_addr);
    rgb_set_mode_solid(0, 255, 0); // GO = GREEN
    buzzer_beep(3600, 250);
    vTaskDelay(pdMS_TO_TICKS(250));

    lap_count = 0;
    elapsed_accum_us = 0;
    timer_running = false;
    reaction_valid = false;
    reaction_us = 0;
    go_time_us = esp_timer_get_time();
    ui_state = UI_ARMED;

    // After GO, show an "armed" color while waiting for the runner crossing.
    rgb_set_mode_solid(0, 0, 120); // ARMED = BLUE
}

// am2320
static uint16_t crc16_modbus(const uint8_t *data, int len);

static uint16_t crc16_modbus(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
        }
    }
    return crc;
}

// OLED (SSD1306 minimal)
#define OLED_BUF_SIZE (OLED_W * OLED_H / 8)

static esp_err_t i2c_probe(uint8_t addr);
static uint8_t oled_detect_addr(void);

// framebuffer
static uint8_t fb[OLED_BUF_SIZE];

static void fb_clear(void) { memset(fb, 0, sizeof(fb)); }

static void fb_set_pixel(int x, int y, int on)
{
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;
    int byte_index = x + (y / 8) * OLED_W;
    uint8_t bit = 1U << (y % 8);
    if (on) fb[byte_index] |= bit;
    else    fb[byte_index] &= (uint8_t)~bit;
}

static void fb_draw_icon_rewind(int x, int y)
{
    // Rewind icon in a 14x10 box: two left chevrons "<<"
    // Drawn centered within the box for consistent alignment.
    int bx = x + 3; // center an ~8px glyph inside 14px box
    int by = y;

    for (int i = 0; i <= 2; i++) {
        // first chevron
        fb_set_pixel(bx + i,     by + 3 - i, 1);
        fb_set_pixel(bx + i,     by + 4 + i, 1);
        // second chevron
        fb_set_pixel(bx + 4 + i, by + 3 - i, 1);
        fb_set_pixel(bx + 4 + i, by + 4 + i, 1);
    }
    // Thicken slightly
    for (int i = 0; i <= 2; i++) {
        fb_set_pixel(bx + i,     by + 3 - i + 1, 1);
        fb_set_pixel(bx + i,     by + 4 + i - 1, 1);
        fb_set_pixel(bx + 4 + i, by + 3 - i + 1, 1);
        fb_set_pixel(bx + 4 + i, by + 4 + i - 1, 1);
    }
}

// super tiny 5x7 font subset (space, digits, letters we use)
static const uint8_t font5x7[][5] = {
    // 32..90
    {0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x00,0x00,0x5F,0x00,0x00}, // '!'
    {0x00,0x07,0x00,0x07,0x00}, // '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
    {0x23,0x13,0x08,0x64,0x62}, // '%'
    {0x36,0x49,0x55,0x22,0x50}, // '&'
    {0x00,0x05,0x03,0x00,0x00}, // '''
    {0x00,0x1C,0x22,0x41,0x00}, // '('
    {0x00,0x41,0x22,0x1C,0x00}, // ')'
    {0x14,0x08,0x3E,0x08,0x14}, // '*'
    {0x08,0x08,0x3E,0x08,0x08}, // '+'
    {0x00,0x50,0x30,0x00,0x00}, // ','
    {0x08,0x08,0x08,0x08,0x08}, // '-'
    {0x00,0x60,0x60,0x00,0x00}, // '.'
    {0x20,0x10,0x08,0x04,0x02}, // '/'

    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'

    {0x00,0x36,0x36,0x00,0x00}, // ':'
    {0x00,0x56,0x36,0x00,0x00}, // ';'
    {0x08,0x14,0x22,0x41,0x00}, // '<'
    {0x14,0x14,0x14,0x14,0x14}, // '='
    {0x00,0x41,0x22,0x14,0x08}, // '>'
    {0x02,0x01,0x51,0x09,0x06}, // '?'
    {0x32,0x49,0x79,0x41,0x3E}, // '@'

    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'
    {0x7F,0x02,0x0C,0x02,0x7F}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
    {0x7F,0x20,0x18,0x20,0x7F}, // 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 'X'
    {0x03,0x04,0x78,0x04,0x03}, // 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 'Z'
};

static const uint8_t* glyph5x7(char c)
{
    if (c < 32 || c > 90) c = ' ';
    return font5x7[c - 32];
}

static void fb_draw_char(int x, int y, char c)
{
    const uint8_t *g = glyph5x7(c);
    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 7; row++) {
            fb_set_pixel(x + col, y + row, (bits >> row) & 1);
        }
    }
    for (int row = 0; row < 7; row++) fb_set_pixel(x + 5, y + row, 0);
}

static void fb_draw_char_scaled(int x, int y, char c, int scale)
{
    if (scale < 1) scale = 1;
    const uint8_t *g = glyph5x7(c);
    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 7; row++) {
            int on = (bits >> row) & 1;
            if (!on) continue;
            for (int dx = 0; dx < scale; dx++) {
                for (int dy = 0; dy < scale; dy++) {
                    fb_set_pixel(x + col * scale + dx, y + row * scale + dy, 1);
                }
            }
        }
    }
}

static void fb_draw_text(int x, int y, const char *s)
{
    int cx = x;
    while (*s) {
        fb_draw_char(cx, y, *s++);
        cx += 6;
        if (cx > OLED_W - 6) break;
    }
}

static void fb_draw_text_scaled(int x, int y, const char *s, int scale)
{
    if (scale < 1) scale = 1;
    int cx = x;
    while (*s) {
        fb_draw_char_scaled(cx, y, *s++, scale);
        cx += 6 * scale;
        if (cx > OLED_W - 6 * scale) break;
    }
}

static void fb_draw_text_centered(int y, const char *s)
{
    int text_width_px = (int)strlen(s) * 6;
    int x = (OLED_W - text_width_px) / 2;
    if (x < 0) x = 0;
    fb_draw_text(x, y, s);
}

// Helper Function to ensure scaled symbols are drawn centered. Calls fb_draw_text_scaled(x, y, s, scale)
static void fb_draw_text_centered_scaled(int y, const char *s, int scale)
{
    if (scale < 1) scale = 1;
    int text_width_px = (int)strlen(s) * 6 * scale;
    int x = (OLED_W - text_width_px) / 2;
    if (x < 0) x = 0;
    fb_draw_text_scaled(x, y, s, scale);
}

static int fb_center_x_for_width(int width_px) //
{
    int x = (OLED_W - width_px) / 2;
    if (x < 0) x = 0;
    return x;
}

static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
}

static esp_err_t i2c_probe(uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return err;
}

static uint8_t oled_detect_addr(void)
{
    if (i2c_probe(OLED_ADDR_1) == ESP_OK) return OLED_ADDR_1;
    if (i2c_probe(OLED_ADDR_2) == ESP_OK) return OLED_ADDR_2;
    return 0;
}

// Function to write command to OLED, requires address and command byte as inputs.
static esp_err_t oled_write_cmd(uint8_t addr, uint8_t cmd_byte)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr<<1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, cmd_byte, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

// function to write data chunk to OLED, requires address, it takes the memory address of the data variable (changing that variable directly), and a size_t variable len.
static esp_err_t oled_write_data_chunk(uint8_t addr, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr<<1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true);
    i2c_master_write(cmd, (uint8_t*)data, len, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    return err;
}

// Same as above
static esp_err_t oled_write_data(uint8_t addr, const uint8_t *data, size_t len)
{
    const size_t CHUNK = 16;
    for (size_t i = 0; i < len; i += CHUNK) {
        size_t n = (len - i > CHUNK) ? CHUNK : (len - i);
        esp_err_t err = oled_write_data_chunk(addr, &data[i], n);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

// Initialization of OLED
static esp_err_t oled_init(uint8_t addr)
{
    const uint8_t init_cmds[] = {
        0xAE,
        0xD5, 0x80,
        0xA8, 0x3F,
        0xD3, 0x00,
        0x40,
        0x8D, 0x14,
        0x20, 0x00,
        0xA1,
        0xC8,
        0xDA, 0x12,
        0x81, 0x7F,
        0xD9, 0xF1,
        0xDB, 0x40,
        0xA4,
        0xA6,
        0xAF
    };
    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        esp_err_t err = oled_write_cmd(addr, init_cmds[i]);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

// function to write to entire OLED.
static esp_err_t oled_set_full_window(uint8_t addr)
{
    esp_err_t err;
    err = oled_write_cmd(addr, 0x21); if (err) return err;
    err = oled_write_cmd(addr, 0x00); if (err) return err;
    err = oled_write_cmd(addr, 0x7F); if (err) return err;
    err = oled_write_cmd(addr, 0x22); if (err) return err;
    err = oled_write_cmd(addr, 0x00); if (err) return err;
    err = oled_write_cmd(addr, 0x07); if (err) return err;
    return ESP_OK;
}
// Unsafe flush
static esp_err_t oled_flush(uint8_t addr)
{
    esp_err_t err = oled_set_full_window(addr);
    if (err != ESP_OK) return err;
    return oled_write_data(addr, fb, sizeof(fb));
}
// safe flush
static void oled_flush_safe(uint8_t oled_addr)
{
    esp_err_t err = oled_flush(oled_addr);
    if (err != ESP_OK) {
        printf("oled_flush failed: %s (0x%x)\n", esp_err_to_name(err), (unsigned)err);
        (void)oled_init(oled_addr);
    }
}

// AM2320 Functions
// Function to change the temp_c and rh_percent variables. Takes pointers to them as parameters.
static esp_err_t am2320_read(float *temp_c, float *rh_percent)
{
    // Try a few times (AM2320 can be flaky)
    for (int attempt = 0; attempt < 5; attempt++) {

        // 1) wake (often NACKs; do NOT require ACK)
        {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (AM2320_ADDR << 1) | I2C_MASTER_WRITE, false); // don't care about ACK
            i2c_master_stop(cmd);
            (void)i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(20));
            i2c_cmd_link_delete(cmd);
        }

        vTaskDelay(pdMS_TO_TICKS(5));   // was 2ms

        // 2) request: 03 00 04
        uint8_t req[3] = {0x03, 0x00, 0x04};
        esp_err_t req_err;
        {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (AM2320_ADDR << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write(cmd, req, sizeof(req), true);
            i2c_master_stop(cmd);
            req_err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
            i2c_cmd_link_delete(cmd);
        }
        if (req_err != ESP_OK) {
            printf("AM2320 req err: %s (0x%x)\n", esp_err_to_name(req_err), (unsigned)req_err);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(10));   // was 2ms

        // 3) read 8 bytes
        uint8_t buf[8] = {0};
        esp_err_t rd_err;
        {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (AM2320_ADDR << 1) | I2C_MASTER_READ, true);
            i2c_master_read(cmd, buf, 7, I2C_MASTER_ACK);
            i2c_master_read_byte(cmd, &buf[7], I2C_MASTER_NACK);
            i2c_master_stop(cmd);
            rd_err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(300));   // was 100
            i2c_cmd_link_delete(cmd);
        }
        if (rd_err != ESP_OK) {
            printf("AM2320 read err: %s (0x%x)\n", esp_err_to_name(rd_err), (unsigned)rd_err);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // 4) validate header
        if (buf[0] != 0x03 || buf[1] != 0x04) {
            printf("AM2320 bad header\n");
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // 5) CRC
        uint16_t crc_calc = crc16_modbus(buf, 6);
        uint16_t crc_recv = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
        if (crc_calc != crc_recv) {
            printf("AM2320 CRC fail calc=%04X recv=%04X\n", crc_calc, crc_recv);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // 6) parse
        uint16_t hum_raw = ((uint16_t)buf[2] << 8) | buf[3];
        *rh_percent = hum_raw / 10.0f;

        uint16_t t_raw = ((uint16_t)buf[4] << 8) | buf[5];
        bool neg = (t_raw & 0x8000) != 0;
        t_raw &= 0x7FFF;
        float t = t_raw / 10.0f;
        *temp_c = neg ? -t : t;

        return ESP_OK;
    }

    return ESP_FAIL;
}


// HC-SR04 (interrupt pulse width)
// Usage of volatile varible (can change "randomly") which is required for the t_rise_us, since it updates.
static QueueHandle_t echo_queue;

typedef struct {
    int64_t width_us;
} echo_event_t;

static volatile int64_t t_rise_us = 0; // volatile

static void IRAM_ATTR echo_isr(void *arg)
{
    (void)arg;
    int level = gpio_get_level(ECHO_GPIO);
    int64_t now = esp_timer_get_time();

    if (level) {
        t_rise_us = now;
    } else {
        int64_t width = now - t_rise_us;
        if (width > 0 && width < 30000) {
            echo_event_t ev = { .width_us = width };
            BaseType_t hp = pdFALSE;
            xQueueSendFromISR(echo_queue, &ev, &hp);
            if (hp) portYIELD_FROM_ISR();
        }
    }
}

// initialization of HCSR04
static void hcsr04_init(void)
{
    gpio_config_t trig_conf = {
        .pin_bit_mask = 1ULL << TRIG_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&trig_conf);
    gpio_set_level(TRIG_GPIO, 0);

    gpio_config_t echo_conf = {
        .pin_bit_mask = 1ULL << ECHO_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&echo_conf);

    echo_queue = xQueueCreate(8, sizeof(echo_event_t));

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ECHO_GPIO, echo_isr, NULL);
}

//
static void hcsr04_trigger(void)
{
    gpio_set_level(TRIG_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(12);
    gpio_set_level(TRIG_GPIO, 0);
}

static float hcsr04_read_cm(void)
{
    hcsr04_trigger();

    echo_event_t ev;
    if (xQueueReceive(echo_queue, &ev, pdMS_TO_TICKS(60)) == pdTRUE) {
        // speed of sound => cm: width_us * 0.0343 / 2
        return (float)ev.width_us * 0.0343f / 2.0f;
    }
    return 0.0f;
}

// RACER EDGE DETECT -> TIMER TOGGLE

#define THRESH_CM        10.0f
#define COOLDOWN_MS      1000

static bool racer_near = false;
static int64_t last_trigger_us = 0;

// Function to update racer 'position'
static bool racer_update(float dist_cm)
{
    bool near_now = (dist_cm > 0.0f && dist_cm < THRESH_CM);
    int64_t now = esp_timer_get_time();

    bool event = (!racer_near && near_now); // FAR->NEAR
    if (event) {
        int64_t since_ms = (now - last_trigger_us) / 1000;
        if (since_ms < COOLDOWN_MS) event = false;
        else last_trigger_us = now;
    }

    racer_near = near_now;
    return event;
}

// 2x7-SEG via 74HC595 chain

static inline void busy_wait(void) { *(volatile uint32_t *)GPIO_IN_REG; }

// digit encoding
static uint8_t digits[10] = {
    0xFC, // 0
    0x60, // 1
    0xDA, // 2
    0xF2, // 3
    0x66, // 4
    0xB6, // 5
    0xBE, // 6
    0xE0, // 7
    0xFE, // 8
    0xE6  // 9
};

// shift out LSB (least significant bit) first
static void shift_out(uint8_t x)
{
    gpio_set_level(SHIFT_LATCH, 0);
    for (int j = 0; j < 8; j++) {
        gpio_set_level(SHIFT_DATA, x & 1);
        busy_wait();
        gpio_set_level(SHIFT_CLOCK, 1);
        x >>= 1;
        busy_wait();
        gpio_set_level(SHIFT_CLOCK, 0);
        busy_wait();
    }
}

// Latch functionality, shift out 8-bit data containing information about the minutes.
static void display_minutes(uint8_t minutes)
{
    uint8_t tens = minutes / 10;
    uint8_t ones = minutes % 10;

    shift_out(digits[ones]);
    shift_out(digits[tens]);

    gpio_set_level(SHIFT_LATCH, 1);
    gpio_set_level(SHIFT_LATCH, 0);
}


// STEPPER (1 rev / 60s when timer running)
#define STEPS_PER_REV  4096
#define REV_TIME_US    (60ULL * 1000000ULL)

// homing speed (microseconds per half-step when returning home)
#define HOME_DELAY_US  800

// track position within current revolution [0..STEPS_PER_REV-1], 0 = home
static volatile uint32_t step_pos = 0;

// coil sequence index (0..7)
static int seq_idx = 0;

static void stepper_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<IN1) | (1ULL<<IN2) | (1ULL<<IN3) | (1ULL<<IN4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 0);
}

// half-step sequence (8)
static const uint8_t SEQ8[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1},
};

static inline void step_write(int idx)
{
    gpio_set_level(IN1, SEQ8[idx][0]);
    gpio_set_level(IN2, SEQ8[idx][1]);
    gpio_set_level(IN3, SEQ8[idx][2]);
    gpio_set_level(IN4, SEQ8[idx][3]);
}

static inline void stepper_off(void)
{
    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 0);
}

static inline void step_forward_one(void)
{
    step_write(seq_idx);
    seq_idx = (seq_idx + 1) & 7;

    step_pos++;
    if (step_pos >= STEPS_PER_REV) step_pos = 0;
}

static inline void step_backward_one(void)
{
    seq_idx = (seq_idx + 7) & 7; // -1 mod 8
    step_write(seq_idx);

    if (step_pos == 0) step_pos = STEPS_PER_REV - 1;
    else step_pos--;
}

static void stepper_task(void *arg)
{
    (void)arg;
    stepper_init();

    // Drift-free timing for 60s/rev
    const uint64_t base_us = REV_TIME_US / (uint64_t)STEPS_PER_REV;
    const uint64_t rem_us  = REV_TIME_US % (uint64_t)STEPS_PER_REV;
    uint64_t acc = 0;

    int64_t next_deadline = esp_timer_get_time();

    while (1) {

        // If asked to home, return to step_pos == 0
        if (stepper_home_request) {
            while (step_pos != 0) {
                step_backward_one();
                esp_rom_delay_us(HOME_DELAY_US);
            }
            step_backward_one(); // Drift correction
            esp_rom_delay_us(HOME_DELAY_US);
            seq_idx = 0;
            step_write(seq_idx);
            step_pos = 0;
            stepper_home_request = false;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // If not running, keep motor off
        if (!timer_running) {
            step_write(seq_idx);
            vTaskDelay(pdMS_TO_TICKS(20));
            // Reset deadline so timing doesn't "jump" when resuming
            next_deadline = esp_timer_get_time();
            continue;
        }

        // Normal stepping (forward)
        step_forward_one();

        // compute delay with fractional distribution
        uint64_t step_delay_us = base_us;
        acc += rem_us;
        if (acc >= (uint64_t)STEPS_PER_REV) {
            acc -= (uint64_t)STEPS_PER_REV;
            step_delay_us += 1;
        }

        next_deadline += (int64_t)step_delay_us;

        int64_t now = esp_timer_get_time();
        int64_t wait_us = next_deadline - now;

        if (wait_us > 2000) {
            vTaskDelay(pdMS_TO_TICKS((uint32_t)(wait_us / 1000)));
            now = esp_timer_get_time();
            wait_us = next_deadline - now;
        }
        if (wait_us > 0) {
            esp_rom_delay_us((uint32_t)wait_us);
        }
    }
}



// app_main (We've tried to "reduce" how much clutter is included in the app main)

void app_main(void)
{
    
    ir_queue = xQueueCreate(32, sizeof(ir_event_t));


    printf("IR RX init on GPIO32...\n");
    ir_init();
    xTaskCreate(ir_task, "ir_task", 4096, NULL, 5, NULL);

    printf("MERGED: OLED + AM2320 + HC-SR04 + 2x7SEG(MIN) + STEPPER\n");

    // init GPIO for shift register pins
    gpio_config_t sr_conf = {
        .pin_bit_mask = (1ULL<<SHIFT_DATA) | (1ULL<<SHIFT_CLOCK) | (1ULL<<SHIFT_LATCH),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&sr_conf);
    gpio_set_level(SHIFT_DATA, 0);
    gpio_set_level(SHIFT_CLOCK, 0);
    gpio_set_level(SHIFT_LATCH, 0);

    // start stepper task
    xTaskCreate(stepper_task, "stepper_task", 2048, NULL, 5, NULL);

    // init I2C + OLED + AM2320
    i2c_master_init();
    for (int a=1; a<127; a++) {
        if (i2c_probe(a) == ESP_OK) printf("I2C device at 0x%02X\n", a);
    }

    uint8_t oled_addr = oled_detect_addr();
    if (!oled_addr) {
        printf("OLED not found at 0x3C/0x3D\n");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("OLED at 0x%02X\n", oled_addr);
    ESP_ERROR_CHECK(oled_init(oled_addr));

    // RGB init (LEDC PWM)
    rgb_init();

    // ultrasonic init (interrupt timing)
    hcsr04_init();

    // buzzer init
    buzzer_init();

    // show boot screen
    fb_clear();
    fb_draw_text_centered(0, "SPORT TIMER");
    fb_draw_text_centered_scaled(24, "POWER OFF", 2);
    oled_flush_safe(oled_addr);

    // Prime AM2320 readings for START warning gate
    {
        float t0 = 0.0f, rh0 = 0.0f;
        esp_err_t am0 = am2320_read(&t0, &rh0);
        if (am0 == ESP_OK) {
            g_last_temp_c = t0;
            g_last_rh_percent = rh0;
            g_have_sensor = true;
        } else {
            g_have_sensor = false;
        }
    }

    while (1) {
        handle_ir_events(oled_addr);
        float t = 0.0f, rh = 0.0f;
        esp_err_t am_err = am2320_read(&t, &rh);
        if (am_err == ESP_OK) {
            g_last_temp_c = t;
            g_last_rh_percent = rh;
            g_have_sensor = true;
        } else {
            g_have_sensor = false;
        }

        float dist_cm = hcsr04_read_cm();
        bool crossed = racer_update(dist_cm);

        // time base
        int64_t now_us = esp_timer_get_time();

        // Ultrasonic crossing behavior (no start/stop toggle)
        if (system_enabled) {
            if (ui_state == UI_ARMED && crossed) {
                reaction_us = now_us - go_time_us;
                reaction_valid = true;
                timer_start_us = now_us;
                timer_running = true;
                ui_state = UI_RUNNING;
                rgb_set_mode_solid(0, 255, 0); // RUNNING = GREEN
                printf("TIMER START (ARMED)\n");
            } else if (ui_state == UI_RUNNING && crossed) {
                lap_count++;
                buzzer_beep(3200, 50);
                printf("LAP %d/%d\n", lap_count, lap_target);
                if (lap_count >= lap_target) {
                    buzzer_pattern_finish();
                    request_finish();
                    printf("TARGET REACHED -> FINISH\n");
                }
            }
        } else {
            ui_state = UI_OFF;
            buzzer_stop();
            rgb_set_mode_off();
        }

        // compute elapsed time (skip entirely when OFF / SETLAPS / ARMED)
        int minutes = 0;
        int seconds = 0;
        int millis = 0;
        int64_t elapsed_us = 0;
        bool show_time = system_enabled && (ui_state != UI_OFF) && (ui_state != UI_SETLAPS) && (ui_state != UI_ARMED);
        if (show_time) {
            elapsed_us = elapsed_accum_us + (timer_running ? (now_us - timer_start_us) : 0);
            int total_ms = (int)(elapsed_us / 1000);
            minutes = total_ms / 60000;
            seconds = (total_ms / 1000) % 60;
            millis  = total_ms % 1000;
            display_minutes((uint8_t)(minutes));
        } else {
            display_minutes(0);
        }


        // OLED UI
        fb_clear();
        fb_draw_text_centered(0, "SPORT TIMER");

        // UI_OFF: big POWER OFF only
        if (!system_enabled || ui_state == UI_OFF) {
            fb_draw_text_centered_scaled(24, "POWER OFF", 2);
            oled_flush_safe(oled_addr);
            vTaskDelay(pdMS_TO_TICKS(150));
            continue;
        }

        // UI_SETLAPS: no TIME, big laps value
        if (ui_state == UI_SETLAPS) {
            fb_draw_text_centered(12, "MODE: SET LAPS");
            {
                char lline[32];
                snprintf(lline, sizeof(lline), "LAPS=%02d", clamp_int(lap_edit_value, 0, 99));
                fb_draw_text_centered_scaled(24, lline, 3);
            }
            // Footer: centered save hint + icon for CLEAR
            fb_draw_text_centered(48, "SAVE = EQ");
            {
                const int icon_w = 14;
                const int gap = 4;
                const char *txt = "CLR";
                const int text_w = (int)strlen(txt) * 6;
                const int group_w = icon_w + gap + text_w;
                const int x0 = fb_center_x_for_width(group_w);
                fb_draw_icon_rewind(x0, 55);
                fb_draw_text(x0 + icon_w + gap, 56, txt);
            }
            oled_flush_safe(oled_addr);
            vTaskDelay(pdMS_TO_TICKS(150));
            continue;
        }

        // UI_FINISHED: finished screen with total time (MM:SS.mmm)
        if (ui_state == UI_FINISHED) {
            fb_draw_text_centered_scaled(12, "RACE FINISHED", 1);
            {
                char tbuf[32];
                snprintf(tbuf, sizeof(tbuf), "%02d:%02d.%03d", minutes, seconds, millis);
                fb_draw_text_centered_scaled(28, tbuf, 2);
            }
            fb_draw_text_centered(56, "RESET");
            oled_flush_safe(oled_addr);
            vTaskDelay(pdMS_TO_TICKS(150));
            continue;
        }

        // Status line
        if (ui_state == UI_READY) {
            fb_draw_text_centered_scaled(16, "READY", 2);
        } else if (ui_state == UI_STOPPED) {
            fb_draw_text_centered_scaled(14, "STOPPED", 2);
        } else if (ui_state == UI_ARMED) {
            // Split across lines so it always fits 128px
            fb_draw_text_centered_scaled(16, "WAIT", 2);
            fb_draw_text_centered_scaled(34, "RUNNER", 2);
            fb_draw_text_centered(56, "CROSS TO START");
        } else if (ui_state == UI_RUNNING) {
            // Make RUNNING clearly larger
            fb_draw_text_centered_scaled(10, "RUNNING", 2);
        } else {
            fb_draw_text_centered(12, "STATE");
        }

        // Time (only when show_time)
        if (show_time) {
            char line[32];
            snprintf(line, sizeof(line), "TIME %02d:%02d.%03d", minutes, seconds, millis);
            int y_time = (ui_state == UI_RUNNING) ? 28 : 32;
            fb_draw_text_centered(y_time, line);
        }

        // Laps (hide in ARMED)
        if (ui_state != UI_ARMED) {
            char lline[32];
            snprintf(lline, sizeof(lline), "LAPS %d/%d", lap_count, lap_target);
            fb_draw_text_centered(40, lline);
        }

        // Temp/Hum (keep clean; skip in STOPPED and ARMED)
        if (ui_state != UI_STOPPED && ui_state != UI_ARMED) {
            if (g_have_sensor) {
                char th[32];
                snprintf(th, sizeof(th), "T %.1fC H %.1f%%", g_last_temp_c, g_last_rh_percent);
                fb_draw_text_centered(48, th);
            } else {
                fb_draw_text_centered(48, "AM2320 FAIL");
            }
        }

        // Reaction time
        if (reaction_valid) {
            char rtline[32];
            int64_t rt_ms = reaction_us / 1000;
            snprintf(rtline, sizeof(rtline), "RT %d.%02ds", (int)(rt_ms / 1000), (int)((rt_ms % 1000) / 10));
            fb_draw_text_centered(56, rtline);
        }

        oled_flush_safe(oled_addr);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}