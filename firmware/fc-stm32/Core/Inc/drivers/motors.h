#if !defined(MOTORS)
#define MOTORS

#include "gpio.h"
#include "tim.h"

#define THRUST_CONST    10
#define YAW_CONST       1
#define PITCH_CONST     4.7
#define ROLL_CONST      4.7

#define MAX_CONTROLLER_VALUE    100

void Motors_Run(int8_t thrust, int8_t yaw, int8_t pitch, int8_t roll, int8_t power_on);

void Motors_SetPWR(uint8_t thrust, int8_t yaw, int8_t pitch, int8_t roll);
void Motors_Switch(uint8_t power_on);

#endif // MOTORS
