#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void buzzer_init(void);
void buzzer_start_tone(uint32_t freq_hz, uint8_t duty_percent);
void buzzer_stop(void);
void buzzer_beep(uint32_t freq_hz, uint32_t ms);
void buzzer_pattern_warning(void);
void buzzer_pattern_finish(void);

#ifdef __cplusplus
}
#endif

#endif
