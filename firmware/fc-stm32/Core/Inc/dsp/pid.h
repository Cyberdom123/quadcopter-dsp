#if !defined(PID)
#define PID

typedef struct 
{
    /* constants */
    float kp;
    float ki;
    float kd;

    /* errors */
    float IntError;
    float lastError;
    
    float maxOut;
    float minOut;
    float maxInt;
    float minInt;
    float sampleTime;
}pid_t;

void PID_init(pid_t *pid);
float PID_Calculate(pid_t *pid, float input, float target);

#endif // PID
