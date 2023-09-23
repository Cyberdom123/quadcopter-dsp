#if !defined(MOTORS)
#define MOTORS

#include "gpio.h"
#include "tim.h"

void Motors_Run(uint8_t msg[8]);

#endif // MOTORS
