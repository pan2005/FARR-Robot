//
// Created by PanZhaoHui on 2025/11/10.
//

#include "bsp_buzzer.h"



extern TIM_HandleTypeDef htim4;
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    if (psc > MAX_PSC) {
        psc = MAX_PSC;
    }
    else if (psc < 0) {
        psc = 0;
    }
    __HAL_TIM_PRESCALER(&htim4, psc);
    if (pwm < MIN_BUZZER_PWM) {
        pwm = MIN_BUZZER_PWM;
    }
    else if (pwm > MAX_BUZZER_PWM) {
        pwm = MAX_BUZZER_PWM;
    }
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

void buzzer_init(void) {
    HAL_TIM_Base_Start(&htim4);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}