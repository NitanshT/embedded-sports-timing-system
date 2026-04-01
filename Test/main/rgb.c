#include "rgb.h"

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_err.h"
#include "esp_timer.h"

// RGB LED (COMMON ANODE -> active-low per channel)
// User intensity is 0..255 where 255 = fully ON (bright).
#define RGB_RED_GPIO    GPIO_NUM_16
#define RGB_GREEN_GPIO  GPIO_NUM_13
#define RGB_BLUE_GPIO   GPIO_NUM_4

static const ledc_mode_t      RGB_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t     RGB_LEDC_TIMER = LEDC_TIMER_1;
static const ledc_timer_bit_t RGB_DUTY_RES = LEDC_TIMER_10_BIT;
static const uint32_t         RGB_FREQ_HZ = 4000;

static const ledc_channel_t RGB_CH_R = LEDC_CHANNEL_1;
static const ledc_channel_t RGB_CH_G = LEDC_CHANNEL_2;
static const ledc_channel_t RGB_CH_B = LEDC_CHANNEL_3;

static portMUX_TYPE g_rgb_mux = portMUX_INITIALIZER_UNLOCKED;
static rgb_mode_state_t g_rgb_state = {
    .mode = RGB_MODE_OFF,
    .r = 0,
    .g = 0,
    .b = 0,
    .period_ms = 500,
    .duty_cycle_percent = 50
};
static TaskHandle_t g_rgb_task = NULL;
static bool g_rgb_inited = false;

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

void rgb_init(void)
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

void rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
    rgb_set_mode_solid(r, g, b);
}

void rgb_off(void)
{
    rgb_set_mode_off();
}

void rgb_set_mode_off(void)
{
    portENTER_CRITICAL(&g_rgb_mux);
    g_rgb_state.mode = RGB_MODE_OFF;
    g_rgb_state.r = 0;
    g_rgb_state.g = 0;
    g_rgb_state.b = 0;
    portEXIT_CRITICAL(&g_rgb_mux);
}

void rgb_set_mode_solid(uint8_t r, uint8_t g, uint8_t b)
{
    portENTER_CRITICAL(&g_rgb_mux);
    g_rgb_state.mode = RGB_MODE_SOLID;
    g_rgb_state.r = r;
    g_rgb_state.g = g;
    g_rgb_state.b = b;
    portEXIT_CRITICAL(&g_rgb_mux);
}

void rgb_set_mode_blink(uint8_t r, uint8_t g, uint8_t b, uint32_t period_ms, uint8_t duty_cycle_percent)
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

void rgb_get_mode_state(rgb_mode_state_t *out_state)
{
    if (!out_state) return;

    portENTER_CRITICAL(&g_rgb_mux);
    *out_state = g_rgb_state;
    portEXIT_CRITICAL(&g_rgb_mux);
}

void rgb_set_mode_state(const rgb_mode_state_t *in_state)
{
    if (!in_state) return;

    portENTER_CRITICAL(&g_rgb_mux);
    g_rgb_state = *in_state;
    portEXIT_CRITICAL(&g_rgb_mux);
}
