#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "soc/gpio_reg.h"

// ===================== PIN CONFIG =====================

// I2C (OLED + AM2320)
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    8
#define I2C_SCL_GPIO    9
#define I2C_FREQ_HZ     100000

// OLED addresses
#define OLED_ADDR_1     0x3C
#define OLED_ADDR_2     0x3D

// AM2320 address
#define AM2320_ADDR     0x5C

// Ultrasonic
#define TRIG_GPIO       GPIO_NUM_19
#define ECHO_GPIO       GPIO_NUM_18

// 74HC595 chain -> 2x 7-seg (minutes)
#define SHIFT_DATA      GPIO_NUM_2
#define SHIFT_CLOCK     GPIO_NUM_10
#define SHIFT_LATCH     GPIO_NUM_1

// Stepper (28BYJ-48 + ULN2003) direct GPIO
#define IN1             GPIO_NUM_7
#define IN2             GPIO_NUM_6
#define IN3             GPIO_NUM_5
#define IN4             GPIO_NUM_4

// ===================== OLED (SSD1306 minimal) =====================

#define OLED_W 128
#define OLED_H 64
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

static void fb_draw_text(int x, int y, const char *s)
{
    int cx = x;
    while (*s) {
        fb_draw_char(cx, y, *s++);
        cx += 6;
        if (cx > OLED_W - 6) break;
    }
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

static esp_err_t oled_flush(uint8_t addr)
{
    esp_err_t err = oled_set_full_window(addr);
    if (err != ESP_OK) return err;
    return oled_write_data(addr, fb, sizeof(fb));
}

static void oled_flush_safe(uint8_t oled_addr)
{
    esp_err_t err = oled_flush(oled_addr);
    if (err != ESP_OK) {
        printf("oled_flush failed: %s (0x%x)\n", esp_err_to_name(err), (unsigned)err);
        (void)oled_init(oled_addr);
    }
}

// ===================== AM2320 =====================

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

static esp_err_t am2320_read(float *temp_c, float *rh_percent)
{
    // wake (may NACK)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (AM2320_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        (void)i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(20));
        i2c_cmd_link_delete(cmd);
    }

    vTaskDelay(pdMS_TO_TICKS(2));

    // request 0x03 0x00 0x04
    uint8_t req[3] = {0x03, 0x00, 0x04};
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (AM2320_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write(cmd, req, sizeof(req), true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (err != ESP_OK) return err;
    }

    vTaskDelay(pdMS_TO_TICKS(2));

    // read 8 bytes
    uint8_t buf[8] = {0};
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (AM2320_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, buf, sizeof(buf) - 1, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &buf[sizeof(buf) - 1], I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (err != ESP_OK) return err;
    }

    if (buf[0] != 0x03 || buf[1] != 0x04) return ESP_FAIL;

    uint16_t crc_calc = crc16_modbus(buf, 6);
    uint16_t crc_recv = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
    if (crc_calc != crc_recv) return ESP_ERR_INVALID_CRC;

    uint16_t hum_raw = ((uint16_t)buf[2] << 8) | buf[3];
    *rh_percent = hum_raw / 10.0f;

    uint16_t t_raw = ((uint16_t)buf[4] << 8) | buf[5];
    int neg = (t_raw & 0x8000) != 0;
    t_raw &= 0x7FFF;
    float t = t_raw / 10.0f;
    *temp_c = neg ? -t : t;

    return ESP_OK;
}

// ===================== HC-SR04 (interrupt pulse width) =====================

static QueueHandle_t echo_queue;

typedef struct {
    int64_t width_us;
} echo_event_t;

static volatile int64_t t_rise_us = 0;

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

// ===================== RACER EDGE DETECT -> TIMER TOGGLE =====================

#define THRESH_CM        10.0f
#define COOLDOWN_MS      1000

static bool racer_near = false;
static int64_t last_trigger_us = 0;

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

// ===================== 2x7-SEG via 74HC595 chain =====================

static inline void busy_wait(void) { *(volatile uint32_t *)GPIO_IN_REG; }

// your digit encoding (kept as-is)
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

// shift out LSB first
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

static void display_seconds(uint8_t sec_0_59)
{
    uint8_t tens = sec_0_59 / 10;
    uint8_t ones = sec_0_59 % 10;

    shift_out(digits[ones]);
    shift_out(digits[tens]);

    gpio_set_level(SHIFT_LATCH, 1);
    gpio_set_level(SHIFT_LATCH, 0);
}


// ===================== STEPPER (1 rev / 60s when timer running) =====================

static volatile bool timer_running = false;
static volatile int64_t timer_start_us = 0;

#define STEPS_PER_REV  4096
#define REV_TIME_US    (60ULL * 1000000ULL)

// homing speed (microseconds per half-step when returning home)
#define HOME_DELAY_US  800

static volatile bool stepper_home_request = false;

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
            stepper_off();
            stepper_home_request = false;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // If not running, keep motor off
        if (!timer_running) {
            stepper_off();
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



// ===================== app_main =====================

void app_main(void)
{
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

    uint8_t oled_addr = oled_detect_addr();
    if (!oled_addr) {
        printf("OLED not found at 0x3C/0x3D\n");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("OLED at 0x%02X\n", oled_addr);
    ESP_ERROR_CHECK(oled_init(oled_addr));

    // ultrasonic init (interrupt timing)
    hcsr04_init();

    // show boot screen
    fb_clear();
    fb_draw_text(0, 0, "SPORT TIMER");
    fb_draw_text(0, 16, "READY");
    oled_flush_safe(oled_addr);

    while (1) {
        float t = 0.0f, rh = 0.0f;
        esp_err_t terr = am2320_read(&t, &rh);

        float dist_cm = hcsr04_read_cm();
        bool crossed = racer_update(dist_cm);

        // Toggle timer on racer crossing (you can replace with countdown later)
        if (crossed) {
            if (!timer_running) {
                timer_running = true;
                timer_start_us = esp_timer_get_time();
                printf("TIMER START\n");
            } else {
                timer_running = false;
                stepper_home_request = true;
                printf("TIMER STOP\n");
            }
        }

        // compute elapsed time
        int64_t now_us = esp_timer_get_time();
        int64_t elapsed_us = timer_running ? (now_us - timer_start_us) : 0;

        int total_ms = (int)(elapsed_us / 1000);
        int total_s  = total_ms / 1000;
        int mm       = total_s / 60;
        int ss       = total_s % 60;

        // 2x7seg shows TOTAL MINUTES (00-99)
        // 2x7seg shows SECONDS (00-59)
        display_seconds((uint8_t)(ss % 60));


        // OLED UI
        fb_clear();
        fb_draw_text(0, 0, "SPORT TIMER");

        if (timer_running) fb_draw_text(0, 12, "RUNNING");
        else              fb_draw_text(0, 12, "READY/STOP");

        // show time MM:SS on OLED (always)
        {
            char line[32];
            snprintf(line, sizeof(line), "TIME %02d:%02d", mm, ss);
            fb_draw_text(0, 24, line);
        }

        // temp/hum
        if (terr == ESP_OK) {
            char line1[32], line2[32];
            snprintf(line1, sizeof(line1), "T %.1fC", t);
            snprintf(line2, sizeof(line2), "H %.1f%%", rh);
            fb_draw_text(0, 36, line1);
            fb_draw_text(64,36, line2);
        } else {
            fb_draw_text(0, 36, "AM2320 FAIL");
        }

        // distance
        {
            char dline[32];
            if (dist_cm > 0.0f) snprintf(dline, sizeof(dline), "D %.1fcm", dist_cm);
            else                snprintf(dline, sizeof(dline), "D --");
            fb_draw_text(0, 48, dline);
        }

        oled_flush_safe(oled_addr);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

