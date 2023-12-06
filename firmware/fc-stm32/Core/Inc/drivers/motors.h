#if !defined(MOTORS)
#define MOTORS

#include "gpio.h"
#include "tim.h"

#define THRUST_CONST    1
#define YAW_CONST       0.1
#define PITCH_CONST     0.47
#define ROLL_CONST      0.47

#define MAX_CONTROLLER_VALUE    100
#define ACTIVATION_THRESHOLD    5

void Motors_Run(int8_t msg[8]);

void Motors_SetPWR(uint8_t thrust, int8_t yaw, int8_t pitch, int8_t roll);

#endif // MOTORS
