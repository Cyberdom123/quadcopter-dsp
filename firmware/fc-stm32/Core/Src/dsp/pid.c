/**
 * @author Dominik Michalczyk
 * @date  10-09-2023
 */

#include <pid.h>

/** 
 * @brief Initialize pid controller
 * Call this function at the beginning
 */
void PID_init(pid_t *pid){
    pid->IntError = 0;
    pid->lastError = 0;
}

/**
 * @brief Calculate pid output
 */
float PID_Calculate(pid_t *pid, float input, float target){

    float error = target - input;

    pid->IntError +=  pid->ki * 0.5f * (error + pid->lastError) * pid->sampleTime;

    if (pid->IntError > pid->maxInt)
    {
        pid->IntError = pid->maxInt;
    }
    if(pid->IntError < - pid->minInt)
    {
        pid->IntError = - pid->minInt;
    }

    float output  = error * pid->kp + pid->IntError + 
                   (error - pid->lastError) * pid->kd / pid->sampleTime;

    if(output > pid->maxOut){
        output = pid->maxOut;
    }

    if(output < pid->minOut){
        output = pid->minOut;
    }

    /* shit values */
    pid->lastError = error;

    return output; 
}  
