#include "buzzer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_err.h"

// Buzzer (PS1240)
#define BUZZER_GPIO     GPIO_NUM_14

#define BUZZER_DUTY_DEFAULT_PERCENT  33

static const ledc_mode_t      BUZZER_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t     BUZZER_LEDC_TIMER = LEDC_TIMER_0;
static const ledc_channel_t   BUZZER_LEDC_CH = LEDC_CHANNEL_0;
static const ledc_timer_bit_t BUZZER_DUTY_RES = LEDC_TIMER_10_BIT;

void buzzer_init(void)
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

void buzzer_start_tone(uint32_t freq_hz, uint8_t duty_percent)
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

void buzzer_stop(void)
{
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CH, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CH);
}

void buzzer_beep(uint32_t freq_hz, uint32_t ms)
{
    buzzer_start_tone(freq_hz, BUZZER_DUTY_DEFAULT_PERCENT);
    vTaskDelay(pdMS_TO_TICKS(ms));
    buzzer_stop();
}

void buzzer_pattern_warning(void)
{
    // 3 short beeps
    for (int i = 0; i < 3; i++) {
        buzzer_beep(3000, 100);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void buzzer_pattern_finish(void)
{
    // Two-tone finish
    buzzer_beep(2500, 250);
    vTaskDelay(pdMS_TO_TICKS(100));
    buzzer_beep(3500, 250);
}
