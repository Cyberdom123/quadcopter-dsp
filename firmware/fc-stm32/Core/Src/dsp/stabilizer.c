#include <dsp/stabilizer.h>

static pid_t roll_pid, pitch_pid;

static IIR_filter_t iir;

void Stabilizer_init(){
    //initialize all pids sample_times and max_values
    roll_pid.maxOut =   75;
    roll_pid.minOut =  -75;
    roll_pid.maxInt =    5;
    roll_pid.minInt =   -5; 
    
    pitch_pid.maxOut =  75;
    pitch_pid.minOut = -75;
    pitch_pid.maxInt =   5;
    pitch_pid.minInt =  -5; 

    roll_pid.sampleTime  =  0.001;
    pitch_pid.sampleTime =  0.001;

    /* Initialize all constants for each pid */

    //roll pid
    roll_pid.tau  =  0.006;   // 25Hz cutoff freq
    roll_pid.kp   =  1.8;
    roll_pid.ki   =  0;  
    roll_pid.kd   = -0.14;
    
    //pitch pid
    pitch_pid.tau =  0.006f; // 25Hz cutoff freq
    pitch_pid.kp  =  1.5;
    pitch_pid.ki  =  0;
    pitch_pid.kd  = -0.16;

    iir.tau = 0.002273f;
    iir.samplingTime = 0.001f;
    Low_Pass_IIR_Filter_Init(&iir);

    PID_init(&roll_pid);
    PID_init(&pitch_pid);
}

static float filter_input_x[2] = {0};
static float filter_output_x[2] = {0};

static float filter_input_y[2] = {0};
static float filter_output_y[2] = {0};

static float angles[3] = {0};
void Stabilize(float acc_buff[3], float gyro_buff[3], int8_t command[8]){
    float roll, pitch;
    volatile float set_angles[3] = {0};   // roll, pitch, yaw
    volatile int8_t duty_cycles[3] = {0}; // roll, pitch, yaw
    
    const float dt = 0.001f, alpha = 0.000001f;
    
    filter_input_x[0] = acc_buff[0];
    filter_input_y[0] = acc_buff[1];
    Low_Pass_IIR_Filter(&iir, filter_output_x, filter_input_x);
    Low_Pass_IIR_Filter(&iir, filter_output_y, filter_input_y);    
    acc_buff[0] = filter_output_x[0];
    acc_buff[1] = filter_output_y[0];
    
    Get_Complementary_Roll_Pitch(angles, acc_buff, gyro_buff, dt, alpha);


    pitch = radToDeg(angles[0]);
    roll  = radToDeg(angles[1]);
    //angles[2] = 0;

    set_angles[0] = command[3] * ROLL_ANGLE_SCALE;
    set_angles[1] = command[1] * PITCH_ANGLE_SCALE;
    //set_angles[2] = 0;

    /* Angle PID's */
    //pitch
    duty_cycles[0] = (int8_t) PID_Calculate(&pitch_pid, pitch, set_angles[0]); 
    //roll    
    duty_cycles[1] = (int8_t) PID_Calculate(&roll_pid, roll, set_angles[1]);
    //yaw
    //duty_cycles[2] = 0;

    int8_t command2[8];
    for (size_t i = 0; i < 8; i++)
    {
        command2[i] = command[i];
    }
    
    //thrust
    command2[0] = command2[0] - 100;
    //pitch
    command2[3] = duty_cycles[0];
    //roll
    command2[1] = duty_cycles[1];
    
    Motors_Run(command2);
}
