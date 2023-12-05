#if !defined(STABILIZER)
#define STABILIZER

#include <dsp/filters.h>
#include <dsp/angle_estimation.h>
#include <dsp/pid.h>
#include <drivers/motors.h>

#define ROLL_ANGLE_SCALE 0.2f
#define PITCH_ANGLE_SCALE 0.2f

void Stabilizer_init();
void Stabilize(float acc_buff[3], float gyro_buff[3], int8_t command[8]);

#endif // STABILIZER
