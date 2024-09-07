/**
 * @file rc.c
 * @author Dominik Michalczyk
 * @brief Radio control liblary for decoding
 *        packets
 * @version 0.1
 * @date 2024-01-26
 *
 *
 */
#include <rc.h>

enum {
  MAXIMUM_PACKET_SIZE = 8,
  MINIMUM_PACKET_SIZE = 6
};

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
void RC_Decode_Message(RC_t *rc, const uint8_t *buffer, size_t size)
{
  int8_t* message = (int8_t*)buffer;
  if (size < MINIMUM_PACKET_SIZE)
  {
    return;
  }

  if (message[0] > ACTIVATION_THRESHOLD_THRUST)
  {
    rc->controls_inputs[thrust] = message[0];
  }
  else
  {
    rc->controls_inputs[thrust] = 10;
  }

  if (message[1] > ACTIVATION_THRESHOLD || message[1] < -ACTIVATION_THRESHOLD)
  {
    rc->controls_inputs[pitch] = message[1] * PITCH_ANGLE_SCALE;
  }
  else
  {
    rc->controls_inputs[pitch] = 0;
  }

  if (message[2] > ACTIVATION_THRESHOLD || message[2] < -ACTIVATION_THRESHOLD)
  {
    rc->controls_inputs[yaw] = message[2] * YAW_ANGLE_SCALE;
  }
  else
  {
    rc->controls_inputs[yaw] = 0;
  }

  if (message[3] > YAW_ACTIVATION_THRESHOLD || message[3] < -YAW_ACTIVATION_THRESHOLD)
  {
    rc->controls_inputs[roll] = message[3] * ROLL_ANGLE_SCALE;
  }
  else
  {
    rc->controls_inputs[roll] = 0;
  }

  if (message[4] == 1 && rc->power_on == 0)
  {
    rc->power_on = 1;
  }

  if (message[5] == 1 && rc->power_on == 1)
  {
    rc->power_on = 0;
  }
}

void RC_Connection_Tick(RC_t *rc)
{
  rc->timeout_cnt = 0;
}

void RC_Check_Connection(RC_t *rc)
{
  HAL_Delay(1);
  if (rc->timeout_cnt < MAX_CONTROLLER_TIMEOUT)
  {
    rc->timeout_cnt++;
  }
}

bool RC_Get_Connection_Status(RC_t *rc)
{
  return (rc->timeout_cnt >= MAX_CONTROLLER_TIMEOUT);
}

void Lower_Altitude(RC_t *rc)
{
  rc->controls_inputs[roll] = 0;
  rc->controls_inputs[pitch] = 0;
  rc->controls_inputs[yaw] = 0;
  while (rc->controls_inputs[thrust] > 10)
  {
    HAL_Delay(800);
    rc->controls_inputs[thrust]--;
  }
}