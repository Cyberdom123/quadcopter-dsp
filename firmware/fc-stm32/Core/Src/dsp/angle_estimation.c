/**
 * @author Kacper Filipek & Dominik Michalczyk
 * @date  10-09-2023
 */

#include <dsp/angle_estimation.h>

/**
 * @brief Calculate fixed frame Euler roll and pitch 
 * @details [ax, ay, az] = (rotation matrix form fixed frame to body frame) * (g vector) 
 *          ax = g * sin(pitch), ay = -g * sin(roll) * cos(pitch), az = -g * cos(roll) * cos(pitch) 
  */
void Calculate_Angles_Acc_Euler(float acc_buf[3], float angles[2]){
  const int x    = 0, y     = 1, z   = 2;
  //remove offset
  acc_buf[x] = acc_buf[x] - AX_OFFSET;

  //cannot divide by 0
  if(acc_buf[z] == 0){
    acc_buf[z] = 0.000001f;
  }
  
  angles[0] = atanf(acc_buf[y]/acc_buf[z]);

  //sin must be in <-1, 1>
  if(acc_buf[x] > 1){
    acc_buf[x] = 1;
  }
  if(acc_buf[x] < -1){
    acc_buf[x] = -1;
  }

  angles[1] = asinf(acc_buf[x]);
}

/**
 * @brief Calculate fixed frame Euler roll and pitch 
 * @note Remap gyro angular velocity (mpu on pcb orientation)
 *       gyro_x = -gyro_x;
  */
void Calculate_Angles_acc(float acc_buf[3], float angles[2]){
  const int x    = 0, y     = 1, z   = 2;
  
  angles[0] = atanf(acc_buf[y]/acc_buf[z]);
  angles[1] = atanf(acc_buf[x]/sqrtf(acc_buf[y] * acc_buf[y] + acc_buf[z] * acc_buf[z]));
}

/**
 * @brief calculates the rotational speed of Euler angles
 * @note Remap gyro angular velocity (mpu on pcb orientation)
 *       gyro_x = -gyro_x;
 * @param angle_change Euler angular velocities
 * @param angles current estimation of euler angles [roll, pitch, yaw]
 * @param gyro gyro inputs [x, y, z]
 */

void Calculate_Angular_Velocities(float angle_change[3], float angles[2], float gyro[3]){
  const int roll = 0, pitch = 1;
  const int x    = 0, y     = 1, z   = 2;

  // NOTE: this are variables, so the trig functions are only evaluated once for speed
  float sin_psi   = sinf(angles[roll]);
  float cos_psi   = cosf(angles[roll]);
  float cos_theta = cosf(angles[pitch]);
  float tan_theta = tanf(angles[pitch]);  

  angle_change[0] = -gyro[x] + tan_theta * (sin_psi  * gyro[y] + cos_psi * gyro[z]);
  angle_change[1] = cos_psi * gyro[y] - sin_psi * gyro[z];
  angle_change[2] = (sin_psi / cos_theta) * gyro[y] + (cos_psi / cos_theta) * gyro[z];
}

/**
 * @brief estimates Euler angles using a complementary filter
  */
void Get_Complementary_Roll_Pitch(float angles[2], float acc_angles[2], float angle_change[3], float dt, float alpha){
  angles[0] += degToRad(angle_change[0]) * dt;   
  angles[1] += degToRad(angle_change[1]) * dt;   

  angles[0] = alpha * acc_angles[0] + (1-alpha) * angles[0];
  angles[1] = alpha * acc_angles[1] + (1-alpha) * angles[1];
}

void Get_XY_Velocities(float acc[3], float angles[3]){

}