#if !defined(ANGLE_ESTIMATION)
#define ANGLE_ESTIMATION
#include "math.h"

#define AX_OFFSET   0.064

void Get_Roll_Pitch(float acc_buf[3], float angles[2]);
void update_euler_angles(float gyro_angles[3], float angles[3], float gyro[3], float dt);
void Get_Complementary_Roll_Pitch(float angles[3], float acc[3], float gyro[3], float dt, float alpha);
#endif // ANGLE_ESTIMATION
