/**
 * @author Kacper Filipek & Dominik Michalczyk
 * @date  10-09-2023
 */

#include <dsp/angle_estimation.h>

/**
 * @brief Estimates fixed frame Euler roll and pitch 
 * @details [ax, ay, az] = (rotation matrix form fixed frame to body frame) * (g vector) 
 *          ax = g * sin(pitch), ay = -g * sin(roll) * cos(pitch), az = -g * cos(roll) * cos(pitch) 
  */
void Calculate_Angles_acc(float acc_buf[3], float angles[2]){

  //remove offset
  acc_buf[0] = acc_buf[0] - AX_OFFSET;

  //cannot divide by 0
  if(acc_buf[2] == 0){
    acc_buf[2] = 0.0001f;
  }
  
  angles[0] = atan2f(acc_buf[1], acc_buf[2]);

  //sin must be in <-1, 1>
  if(acc_buf[0] > 1){
    acc_buf[0] = 1;
  }
  if(acc_buf[0] < -1){
    acc_buf[0] = -1;
  }

  angles[1] = asinf(acc_buf[0]);
}
/**
 * @brief 
 * 
 * @param angles current estimation of euler angles [roll, pitch, yaw]
 * @param gyro gyro inputs [x, y, z]
 * @param dt time since last gyro sample
 */

void Calculate_Angles_gyro(float angles[3], float gyro[3], float dt) {
  const int roll = 0, pitch = 1, yaw = 2;
  const int x    = 0, y     = 1, z   = 2;

  // NOTE: this are variables, so the trig functions are only evaluated once for speed
  float sin_psi   = sinf(angles[roll]);
  float cos_psi   = cosf(angles[roll]);
  float tan_theta = tanf(angles[pitch]);  
  
  //Remap gyro angular velocity
  gyro[0] = -gyro[0];

  float angle_change[3] = {gyro[x] + tan_theta * (sin_psi  * gyro[y] + cos_psi * gyro[z]),
                           cos_psi * gyro[y] - sin_psi * gyro[z],
                           0};
  //(sin_psi / cos_theta) * gyro[y] + (cos_psi / cos_theta) * gyro[z]}

  for(int i = 0; i < 3; i++) {
    // NOTE: this function currently returns the new euler angles but may be chaged to return the deltas for use in a complementary filter
    angles[i] += degToRad(angle_change[i]) * dt;   
  }
}

void Get_Complementary_Roll_Pitch(float angles[3], float acc[3], float gyro[3], float dt, float alpha){
  float acc_angles[2] = {0};

  Calculate_Angles_acc(acc, acc_angles);
  Calculate_Angles_gyro(angles, gyro, dt);

  angles[0] = alpha * acc_angles[0] + (1-alpha) * angles[0];
  angles[1] = alpha * acc_angles[1] + (1-alpha) * angles[1];
}


void Get_XY_Velocities(float acc[3], float angles[3]){

}