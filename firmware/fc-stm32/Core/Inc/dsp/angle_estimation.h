#if !defined(ANGLE_ESTIMATION)
#define ANGLE_ESTIMATION
#include "math.h"

#define degToRad(angleInDegrees) ((angleInDegrees) * 3.14159f / 180.0f)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0f / 3.14159f)

#define AX_OFFSET   0.02f

void Calculate_Angles_acc(float acc_buf[3], float angles[2]);
void Calculate_Angles_gyro(float angles[3], float gyro[3], float dt);
void Get_Complementary_Roll_Pitch(float angles[3], float acc[3], float gyro[3], float dt, float alpha);

#endif // ANGLE_ESTIMATION
