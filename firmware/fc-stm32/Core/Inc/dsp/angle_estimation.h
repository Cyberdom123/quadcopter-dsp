#if !defined(ANGLE_ESTIMATION)
#define ANGLE_ESTIMATION
#include "math.h"

#define degToRad(angleInDegrees) ((angleInDegrees) * 3.14159f / 180.0f)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0f / 3.14159f)

#define AX_OFFSET   0.05f

void Calculate_Angles_acc(float acc_buf[3], float angles[2]);
void Calculate_Angular_Velocities(float angle_change[3], float angles[2], float gyro[3]);
void Get_Complementary_Roll_Pitch(float angles[2], float acc_angles[2], float angle_change[3], float dt, float alpha);

#endif // ANGLE_ESTIMATION
