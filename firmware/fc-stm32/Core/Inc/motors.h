#if !defined(MOTORS)
#define MOTORS

#include "gpio.h"
#include "tim.h"

#define THRUST_CONST    0.9
#define YAW_CONST       0.05
#define PITCH_CONST     0.05
#define ROLL_CONST      0.05

#define MAX_CONTROLLER_VALUE    100
#define ACTIVATION_THRESHOLD    10

void Motors_Run(uint8_t msg[8]);

void Motors_SetPWR(uint8_t thrust, int8_t yaw, int8_t pitch, int8_t roll);

#endif // MOTORS
