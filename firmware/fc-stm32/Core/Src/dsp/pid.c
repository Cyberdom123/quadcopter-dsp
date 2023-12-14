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

    pid->ki = pid->ki * pid->sampleTime * 0.5f;
    pid->kd = pid->kd * 2/(2*pid->tau + pid->sampleTime);
    pid->lowPassTerm = (2 * pid->tau - pid->sampleTime)/(2 * pid->tau + pid->sampleTime);

    pid->IntError = 0;
    pid->lastError = 0;
    pid->derivError = 0;
    pid->lastDeriv = 0;
}

/**
 * @brief Calculate pid output
 */
float PID_Calculate(pid_t *pid, float input, float target){

    float error = target - input;

    pid->IntError +=  pid->ki * (error + pid->lastError);

    /* Integral anti windup */
    if (pid->IntError > pid->maxInt) pid->IntError = pid->maxInt;

    if(pid->IntError < pid->minInt)  pid->IntError = pid->minInt;

    /* Derivative on measurement */
    float derivative = (input - pid->derivError)  * pid->kd + pid->lowPassTerm * pid->lastDeriv;
    
    /* Calculate PID output */
    float output  = error * pid->kp + pid->IntError + derivative;

    /* Apply output limits */
    if(output > pid->maxOut) output = pid->maxOut;

    if(output < pid->minOut) output = pid->minOut;

    /* Shit values */
    pid->derivError = input;
    pid->lastError = error;
    pid->lastDeriv = derivative;

    return output; 
}  
