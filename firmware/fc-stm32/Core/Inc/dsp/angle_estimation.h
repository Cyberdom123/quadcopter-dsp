#if !defined(ANGLE_ESTIMATION)
#define ANGLE_ESTIMATION
#include "math.h"

#define AX_OFFSET   0.064

void Get_Roll_Pitch(float acc_buf[3], float angles[2]);

#endif // ANGLE_ESTIMATION
