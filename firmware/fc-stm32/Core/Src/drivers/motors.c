#include <drivers/motors.h>

/**
 * @brief Run DC motors using PWM
 *        -  msg[0] - Thrust
 *        -  msg[1] - Pitch
 *        -  msg[2] - Yaw
 *        -  msg[3] - Roll
 *        -  msg[4] - Power on
 *        -  msg[5] - Power off
 *        - (msg[6] - msg[8]) - free 
  */
uint8_t power_on = 0;
void Motors_Run(int8_t msg[8]){
    uint8_t thrust = 10;
    volatile int8_t yaw = 0;
    volatile int8_t pitch = 0;
    volatile int8_t roll = 0;

    if(msg[0] > ACTIVATION_THRESHOLD){
      thrust = msg[0];
    }
    if(msg[1] > ACTIVATION_THRESHOLD || msg[1] < - ACTIVATION_THRESHOLD){
      pitch = msg[1];
    }
    if(msg[2] > ACTIVATION_THRESHOLD || msg[2] < - ACTIVATION_THRESHOLD){
      yaw = msg[2];
    }
    if(msg[3] > ACTIVATION_THRESHOLD || msg[3] < - ACTIVATION_THRESHOLD){
      roll = msg[3];
    }
    if(msg[4] == 1 && power_on == 0){
      power_on = 1;
    }
    if(msg[5] == 1 && power_on == 1){
      power_on = 0;
    }

    Motors_SetPWR(thrust, yaw, pitch, roll);

    if(power_on){
      //Start all PWM chanels
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);   
    }else{
      //Stop all PWM chanels
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
    }
      
}

void Motors_SetPWR(uint8_t thrust, int8_t yaw, int8_t pitch, int8_t roll)
{
  //Motor 1 PWM duty cycle
  TIM2->CCR1 = THRUST_CONST * thrust - PITCH_CONST * pitch + YAW_CONST * yaw - ROLL_CONST * roll;
  //Motor 2 PWM duty cycle
  TIM2->CCR2 = THRUST_CONST * thrust + PITCH_CONST * pitch - YAW_CONST * yaw - ROLL_CONST * roll;
  //Motor 3 PWM duty cycle
  TIM2->CCR3 = THRUST_CONST * thrust + PITCH_CONST * pitch + YAW_CONST * yaw + ROLL_CONST * roll;
  //Motor 4 PWM duty cycle
  TIM2->CCR4 = THRUST_CONST * thrust - PITCH_CONST * pitch - YAW_CONST * yaw + ROLL_CONST * roll;
}