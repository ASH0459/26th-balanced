#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BUZZER_H */
