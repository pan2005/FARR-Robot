//
// Created by PanZhaoHUi on 2025/11/10.
//

#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "main.h"

#define MAX_PSC                 1000

#define MAX_BUZZER_PWM      20000
#define MIN_BUZZER_PWM      10000

extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);

void buzzer_init(void);

#endif
