#ifndef RGB_H
#define RGB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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

void rgb_init(void);
void rgb_set(uint8_t r, uint8_t g, uint8_t b);
void rgb_off(void);
void rgb_set_mode_off(void);
void rgb_set_mode_solid(uint8_t r, uint8_t g, uint8_t b);
void rgb_set_mode_blink(uint8_t r, uint8_t g, uint8_t b, uint32_t period_ms, uint8_t duty_cycle_percent);
void rgb_get_mode_state(rgb_mode_state_t *out_state);
void rgb_set_mode_state(const rgb_mode_state_t *in_state);

#ifdef __cplusplus
}
#endif

#endif
