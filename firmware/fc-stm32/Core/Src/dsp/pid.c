/**
 * @author Dominik Michalczyk
 * @date  10-09-2023
 */

#include <pid.h>

void PID_init(pid_t *pid){
    pid->i_error = 0;
    pid->last_error = 0;
}

float PID_Calculate(pid_t *pid, float input, float target){

    float error = target - input;

    pid->i_error += error * pid->sample_time;

    float output  = error * pid->kp + pid->i_error * pid->ki + 
                   (error - pid->last_error) * pid->kd / pid->sample_time;

    if(output > pid->max_out){
        output = pid->max_out;
    }

    if(output < 0){
        output = 0;
    }

    return output; 
}  
