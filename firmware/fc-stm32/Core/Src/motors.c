#include "motors.h"


uint8_t power_on = 0;
void Motors_Run(uint8_t msg[8]){
    uint8_t pwr = 10;

    if(msg[0]-70 > 10){
    pwr = msg[0] - 70;
    }
    
    if(msg[4] == 1 && power_on == 0){
    power_on = 1;
    }

    if(msg[5] == 1 && power_on == 1){
    power_on = 0;
    }

    TIM2->CCR1 = pwr;
    TIM2->CCR2 = pwr;
    TIM2->CCR3 = pwr;
    TIM2->CCR4 = pwr;

    if(power_on){
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);   
    }else{
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
    }
      
}