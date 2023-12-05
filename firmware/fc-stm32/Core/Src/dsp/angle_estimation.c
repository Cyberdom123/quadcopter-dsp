#include <dsp/angle_estimation.h>

/**
 * @brief Estimates fixed frame Euler roll and pitch 
 * @details [ax, ay, az] = (rotation matrix form fixed frame to body frame) * (g vector) 
 *          ax = g * sin(pitch), ay = -g * sin(roll) * cos(pitch), az = -g * cos(roll) * cos(pitch) 
  */
void Get_Roll_Pitch_Acc(float acc_buf[3], float angles[2]){

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