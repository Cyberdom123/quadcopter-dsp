#if !defined(PID)
#define PID

typedef struct 
{
    /* constants */
    float kp;
    float ki;
    float kd;

    /* errors */
    float i_error;
    float last_error;

    float max_out;
    float sample_time;
}pid_t;

void PID_init(pid_t *pid);
float PID_Calculate(pid_t *pid, float input, float target);

#endif // PID
