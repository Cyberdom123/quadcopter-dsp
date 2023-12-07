#include <dsp/angle_estimation.h>

/**
 * @brief Estimates fixed frame Euler roll and pitch 
 * @details [ax, ay, az] = (rotation matrix form fixed frame to body frame) * (g vector) 
 *          ax = g * sin(pitch), ay = -g * sin(roll) * cos(pitch), az = -g * cos(roll) * cos(pitch) 
  */
void Get_Roll_Pitch(float acc_buf[3], float angles[2]){

  //remove offset
  acc_buf[0] = acc_buf[0] - AX_OFFSET;

  //cannot divide by 0
  if(acc_buf[2] == 0){
    acc_buf[2] = 0.0001;
  }
  
  angles[0] = atan2(acc_buf[1], acc_buf[2]);

  //sin must be in <-1, 1>
  if(acc_buf[0] > 1){
    acc_buf[0] = 1;
  }
  if(acc_buf[0] < -1){
    acc_buf[0] = -1;
  }

  angles[1] = asin(acc_buf[0]);
}
/**
 * @brief 
 * 
 * @param angles current estimation of euler angles [roll, pitch, yaw]
 * @param gyro gyro inputs [x, y, z]
 * @param dt time since last gyro sample
 */

void update_euler_angles(float angles[3], float gyro[3], float dt) {
  const int roll = 0, pitch = 1, yaw = 2;
  const int x    = 0, y     = 1, z   = 2;

  float sin_psi   = sin(angles[roll]);
  float cos_psi   = cos(angles[roll]), cos_theta = cos(angles[pitch]);
  float tan_theta = tan(angles[pitch]);

  float angle_change[3] = {gyro[x] + sin_psi * cos_theta * gyro[y],
                           cos_psi * gyro[y] - sin_psi * gyro[z],
                           (sin_psi / cos_theta) * gyro[y] + (cos_psi / cos_theta) * gyro[z]};

  for(int i = 0; i < 3; i++) {
    // NOTE: this function currently returns the new euler angles but may be chaged to return the deltas for use in a complementary filter
    angles[i] += angle_change[i] * dt;   
  }
}