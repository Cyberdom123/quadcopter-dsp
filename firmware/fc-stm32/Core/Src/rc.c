#include <rc.h>

/**
 * @brief decode message
 *        -  msg[0] - Thrust
 *        -  msg[1] - Pitch
 *        -  msg[2] - Yaw
 *        -  msg[3] - Roll
 *        -  msg[4] - Power on
 *        -  msg[5] - Power off
 *        - (msg[6] - msg[8]) - free 
  */
void RC_Receive_Message(uint8_t message[8], RC_t *rc){
    int8_t *message_int = (int8_t*) message;
    
    if(message_int[0] > ACTIVATION_THRESHOLD_THRUST){
      rc->controls_inputs[thrust] = message_int[0];
    }else{
      rc->controls_inputs[thrust] = 10;
    }

    if(message_int[1] > ACTIVATION_THRESHOLD || message_int[1] < - ACTIVATION_THRESHOLD){
      rc->controls_inputs[pitch] = message_int[1] * PITCH_ANGLE_SCALE;
    }else{
      rc->controls_inputs[pitch] = 0;
    }

    if(message_int[2] > ACTIVATION_THRESHOLD || message_int[2] < - ACTIVATION_THRESHOLD){
      rc->controls_inputs[yaw] = message_int[2] * YAW_ANGLE_SCALE;
    }else{
      rc->controls_inputs[yaw] = 0;
    }

    if(message_int[3] > ACTIVATION_THRESHOLD || message_int[3] < - ACTIVATION_THRESHOLD){
      rc->controls_inputs[roll] = message_int[3] * ROLL_ANGLE_SCALE;
    }else{
      rc->controls_inputs[roll] = 0;
    }

    if(message_int[4] == 1 && rc->power_on == 0){
      rc->power_on = 1;
    }

    if(message_int[5] == 1 && rc->power_on == 1){
      rc->power_on = 0;
    }
}
