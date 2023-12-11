#if !defined(PID)
#define PID

typedef struct 
{
    /* constants */
    float kp;
    float ki;
    float kd;
    float sampleTime;

    /* errors */
    float IntError;
    float lastError;
    float derivError;
    float lastDeriv;

    /* filter */
    float tau;
    float lowPassTerm;

    /* limits */
    float maxOut;
    float minOut;
    float maxInt;
    float minInt;

}pid_t;

void PID_init(pid_t *pid);
float PID_Calculate(pid_t *pid, float input, float target);

#endif // PID
