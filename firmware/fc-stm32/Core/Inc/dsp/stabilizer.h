#if !defined(STABILIZER)
#define STABILIZER

#include <dsp/filters.h>
#include <dsp/angle_estimation.h>
#include <dsp/pid.h>
#include <drivers/motors.h>



void Stabilizer_init();
void Stabilize(float acc_buff[3], float gyro_buff[3], int8_t control_inputs[4]);

#endif // STABILIZER
