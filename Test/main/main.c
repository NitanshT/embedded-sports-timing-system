#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/queue.h"


// ---------------- I2C config ----------------
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    2
#define I2C_SCL_GPIO    3
#define I2C_FREQ_HZ     100000

// OLED common addresses
#define OLED_ADDR_1     0x3C
#define OLED_ADDR_2     0x3D

// AM2320 address
#define AM2320_ADDR     0x5C

// SSD1306 control bytes
#define OLED_CTRL_CMD   0x00
#define OLED_CTRL_DATA  0x40

// OLED size
#define OLED_W 128
#define OLED_H 64
#define OLED_BUF_SIZE (OLED_W * OLED_H / 8)

// ---------------- HC-SR04 ----------------
#define TRIG_GPIO GPIO_NUM_19
#define ECHO_GPIO GPIO_NUM_18

// ---------------- Simple 5x7 font (ASCII 32..127) ----------------
// Only include needed glyphs to keep it short: space, digits, letters used, colon, dot, percent, C
// Each char is 5 bytes (columns), LSB at top (for our draw routine we’ll interpret bits)
static const uint8_t font5x7[][5] = {
    // ' ' 32
    {0x00,0x00,0x00,0x00,0x00},
    // '!' 33 (unused)
    {0x00,0x00,0x5F,0x00,0x00},
    // '"' 34 (unused)
    {0x00,0x07,0x00,0x07,0x00},
    // '#' 35 (unused)
    {0x14,0x7F,0x14,0x7F,0x14},
    // '$' 36 (unused)
    {0x24,0x2A,0x7F,0x2A,0x12},
    // '%' 37
    {0x23,0x13,0x08,0x64,0x62},
    // '&' 38 (unused)
    {0x36,0x49,0x55,0x22,0x50},
    // ''' 39 (unused)
    {0x00,0x05,0x03,0x00,0x00},
    // '(' 40 (unused)
    {0x00,0x1C,0x22,0x41,0x00},
    // ')' 41 (unused)
    {0x00,0x41,0x22,0x1C,0x00},
    // '*' 42 (unused)
    {0x14,0x08,0x3E,0x08,0x14},
    // '+' 43 (unused)
    {0x08,0x08,0x3E,0x08,0x08},
    // ',' 44 (unused)
    {0x00,0x50,0x30,0x00,0x00},
    // '-' 45
    {0x08,0x08,0x08,0x08,0x08},
    // '.' 46
    {0x00,0x60,0x60,0x00,0x00},
    // '/' 47 (unused)
    {0x20,0x10,0x08,0x04,0x02},

    // '0' 48
    {0x3E,0x51,0x49,0x45,0x3E},
    // '1' 49
    {0x00,0x42,0x7F,0x40,0x00},
    // '2' 50
    {0x42,0x61,0x51,0x49,0x46},
    // '3' 51
    {0x21,0x41,0x45,0x4B,0x31},
    // '4' 52
    {0x18,0x14,0x12,0x7F,0x10},
    // '5' 53
    {0x27,0x45,0x45,0x45,0x39},
    // '6' 54
    {0x3C,0x4A,0x49,0x49,0x30},
    // '7' 55
    {0x01,0x71,0x09,0x05,0x03},
    // '8' 56
    {0x36,0x49,0x49,0x49,0x36},
    // '9' 57
    {0x06,0x49,0x49,0x29,0x1E},

    // ':' 58
    {0x00,0x36,0x36,0x00,0x00},
    // ';' 59 (unused)
    {0x00,0x56,0x36,0x00,0x00},
    // '<' 60 (unused)
    {0x08,0x14,0x22,0x41,0x00},
    // '=' 61 (unused)
    {0x14,0x14,0x14,0x14,0x14},
    // '>' 62 (unused)
    {0x00,0x41,0x22,0x14,0x08},
    // '?' 63 (unused)
    {0x02,0x01,0x51,0x09,0x06},
    // '@' 64 (unused)
    {0x32,0x49,0x79,0x41,0x3E},

    // 'A' 65
    {0x7E,0x11,0x11,0x11,0x7E},
    // 'B' 66
    {0x7F,0x49,0x49,0x49,0x36},
    // 'C' 67
    {0x3E,0x41,0x41,0x41,0x22},
    // 'D' 68 (unused)
    {0x7F,0x41,0x41,0x22,0x1C},
    // 'E' 69
    {0x7F,0x49,0x49,0x49,0x41},
    // 'F' 70 (unused)
    {0x7F,0x09,0x09,0x09,0x01},
    // 'G' 71 (unused)
    {0x3E,0x41,0x49,0x49,0x7A},
    // 'H' 72
    {0x7F,0x08,0x08,0x08,0x7F},
    // 'I' 73 (unused)
    {0x00,0x41,0x7F,0x41,0x00},
    // 'J' 74 (unused)
    {0x20,0x40,0x41,0x3F,0x01},
    // 'K' 75 (unused)
    {0x7F,0x08,0x14,0x22,0x41},
    // 'L' 76
    {0x7F,0x40,0x40,0x40,0x40},
    // 'M' 77 (unused)
    {0x7F,0x02,0x0C,0x02,0x7F},
    // 'N' 78 (unused)
    {0x7F,0x04,0x08,0x10,0x7F},
    // 'O' 79 (unused)
    {0x3E,0x41,0x41,0x41,0x3E},
    // 'P' 80 (unused)
    {0x7F,0x09,0x09,0x09,0x06},
    // 'Q' 81 (unused)
    {0x3E,0x41,0x51,0x21,0x5E},
    // 'R' 82 (unused)
    {0x7F,0x09,0x19,0x29,0x46},
    // 'S' 83 (unused)
    {0x46,0x49,0x49,0x49,0x31},
    // 'T' 84
    {0x01,0x01,0x7F,0x01,0x01},
    // 'U' 85 (unused)
    {0x3F,0x40,0x40,0x40,0x3F},
    // 'V' 86 (unused)
    {0x1F,0x20,0x40,0x20,0x1F},
    // 'W' 87 (unused)
    {0x7F,0x20,0x18,0x20,0x7F},
    // 'X' 88 (unused)
    {0x63,0x14,0x08,0x14,0x63},
    // 'Y' 89 (unused)
    {0x03,0x04,0x78,0x04,0x03},
    // 'Z' 90 (unused)
    {0x61,0x51,0x49,0x45,0x43},
};

// Map ASCII to our font table index (we only guarantee 32..90 here)
static const uint8_t* glyph5x7(char c)
{
    if (c < 32 || c > 90) c = ' ';
    return font5x7[c - 32];
}

// ---------------- I2C helpers ----------------
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

// ---------------- SSD1306 (minimal) ----------------
static esp_err_t oled_write_cmd(uint8_t addr, uint8_t cmd_byte)
{
    // START -> addrW -> 0x00(control=cmd) -> cmd -> STOP
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr<<1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);     // control byte: command stream
    i2c_master_write_byte(cmd, cmd_byte, true); // SSD1306 command
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}


static esp_err_t oled_write_data_chunk(uint8_t addr, const uint8_t *data, size_t len)
{
    // START -> addrW -> 0x40(control=data) -> data... -> STOP
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr<<1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true); // control byte: data stream
    i2c_master_write(cmd, (uint8_t*)data, len, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t oled_write_data(uint8_t addr, const uint8_t *data, size_t len)
{
    // send in small chunks
    const size_t CHUNK = 16;
    size_t i = 0;
    while (i < len) {
        size_t n = (len - i > CHUNK) ? CHUNK : (len - i);
        esp_err_t err = oled_write_data_chunk(addr, &data[i], n);
        if (err != ESP_OK) return err;
        i += n;
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

// Framebuffer for drawing
static uint8_t fb[OLED_BUF_SIZE];

static void fb_clear(void)
{
    memset(fb, 0x00, sizeof(fb));
}

static void fb_set_pixel(int x, int y, int on)
{
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;
    int byte_index = x + (y / 8) * OLED_W;
    uint8_t bit = 1U << (y % 8);
    if (on) fb[byte_index] |= bit;
    else    fb[byte_index] &= (uint8_t)~bit;
}

static void fb_draw_char(int x, int y, char c)
{
    const uint8_t *g = glyph5x7(c);
    // 5 columns wide, 7 pixels tall
    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 7; row++) {
            int on = (bits >> row) & 1;
            fb_set_pixel(x + col, y + row, on);
        }
    }
    // 1 column spacing
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

static esp_err_t oled_flush(uint8_t addr)
{
    esp_err_t err = oled_set_full_window(addr);
    if (err != ESP_OK) return err;
    return oled_write_data(addr, fb, sizeof(fb));
}

// ---------------- AM2320 ----------------
// CRC-16/MODBUS (poly 0xA001, init 0xFFFF)
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
    // 1) wake (may NACK; ignore)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (AM2320_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        (void)i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(20));
        i2c_cmd_link_delete(cmd);
    }

    vTaskDelay(pdMS_TO_TICKS(2));

    // 2) request: 0x03 0x00 0x04
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

    // after sending request (0x03 0x00 0x04)
    vTaskDelay(pdMS_TO_TICKS(2));   // <-- IMPORTANT: let AM2320 prepare data


    // 3) read 8 bytes
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

static void oled_flush_safe(uint8_t oled_addr)
{
    esp_err_t err = oled_flush(oled_addr);
    if (err != ESP_OK) {
        printf("oled_flush failed: %s (0x%x)\n", esp_err_to_name(err), (unsigned)err);
        // Try re-init once if the bus glitched
        (void)oled_init(oled_addr);
    }
}

static QueueHandle_t echo_queue;

typedef struct {
    int64_t width_us;
} echo_event_t;

static volatile int64_t t_rise_us = 0;

static void IRAM_ATTR echo_isr(void *arg)
{
    int level = gpio_get_level(ECHO_GPIO);
    int64_t now = esp_timer_get_time();

    if (level) {
        t_rise_us = now;  // rising edge timestamp
    } else {
        int64_t width = now - t_rise_us; // pulse width us
        if (width > 0 && width < 30000) { // sanity (30ms ~ 5m)
            echo_event_t ev = { .width_us = width };
            BaseType_t hp = pdFALSE;
            xQueueSendFromISR(echo_queue, &ev, &hp);
            if (hp) portYIELD_FROM_ISR();
        }
    }
}

static void hcsr04_init(void)
{
    // TRIG output
    gpio_config_t trig_conf = {
        .pin_bit_mask = 1ULL << TRIG_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&trig_conf);
    gpio_set_level(TRIG_GPIO, 0);

    // ECHO input with interrupt on both edges
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
    esp_rom_delay_us(12); // >=10us
    gpio_set_level(TRIG_GPIO, 0);
}

static float width_us_to_cm(int64_t width_us)
{
    return (float)width_us * 0.0343f / 2.0f;
}

// returns 0 if timeout/no echo
static float hcsr04_read_cm(void)
{
    hcsr04_trigger();

    echo_event_t ev;
    if (xQueueReceive(echo_queue, &ev, pdMS_TO_TICKS(60)) == pdTRUE) {
        return width_us_to_cm(ev.width_us);
    }
    return 0.0f;
}


// ---------------- app_main ----------------
void app_main(void)
{
    printf("OLED + AM2320 demo starting...\n");
    i2c_master_init();
    hcsr04_init();


    uint8_t oled_addr = oled_detect_addr();
    if (!oled_addr) {
        printf("OLED not found at 0x3C/0x3D\n");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("OLED at 0x%02X\n", oled_addr);

    ESP_ERROR_CHECK(oled_init(oled_addr));

    // Show "HELLO" once
    fb_clear();
    fb_draw_text(10, 10, "HELLO");
    fb_draw_text(10, 22, "AM2320 DEMO");
    ESP_ERROR_CHECK(oled_flush(oled_addr));
    vTaskDelay(pdMS_TO_TICKS(1200));

    while (1) {
        float t = 0.0f, rh = 0.0f;
        esp_err_t err = am2320_read(&t, &rh);

        float dist_cm = hcsr04_read_cm();
        printf("Distance: %.1f cm\n", dist_cm);

        fb_clear();
        fb_draw_text(0, 0, "SPORT TIMER");

        if (err == ESP_OK) {
            char line1[32], line2[32];
            snprintf(line1, sizeof(line1), "TEMP: %.1f C", t);
            snprintf(line2, sizeof(line2), "HUM : %.1f %%", rh);
            fb_draw_text(0, 16, line1);
            fb_draw_text(0, 28, line2);
        } else {
            char e[32];
            fb_draw_text(0, 16, "AM2320 READ FAIL");
            snprintf(e, sizeof(e), "ERR: %s", esp_err_to_name(err));
            fb_draw_text(0, 28, e);
            printf("AM2320 read failed: %s (0x%x)\n", esp_err_to_name(err), (unsigned)err);
        }

        // Show distance on OLED under the others
        {
            char dline[32];
            if (dist_cm > 0.0f) snprintf(dline, sizeof(dline), "DIST: %.1f cm", dist_cm);
            else                snprintf(dline, sizeof(dline), "DIST: --");
            fb_draw_text(0, 44, dline);
        }

        oled_flush_safe(oled_addr);

        // Faster update is nicer for ultrasound; 200ms is smooth
        vTaskDelay(pdMS_TO_TICKS(200));
    }


}
