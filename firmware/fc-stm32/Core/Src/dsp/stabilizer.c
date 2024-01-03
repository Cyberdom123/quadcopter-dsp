#include <dsp/stabilizer.h>

static pid_t roll_pid, pitch_pid, yaw_pid;
static IIR_filter_t iir;
static enum{
    thrust = 0,
    pitch  = 1,
    yaw    = 2,
    roll   = 3
}Control_Inputs_t;

void Stabilizer_init(){
    // Initialize all pids sample times and max_values
    roll_pid.maxOut =   75;
    roll_pid.minOut =  -75;
    roll_pid.maxInt =    5;
    roll_pid.minInt =   -5; 
    
    pitch_pid.maxOut =  75;
    pitch_pid.minOut = -75;
    pitch_pid.maxInt =   5;
    pitch_pid.minInt =  -5; 

    yaw_pid.maxOut =  75;
    yaw_pid.minOut = -75;
    yaw_pid.maxInt =   5;
    yaw_pid.minInt =  -5; 

    roll_pid.sampleTime  =  0.001f;
    pitch_pid.sampleTime =  0.001f;
    yaw_pid.sampleTime   =  0.001f;
    /* Initialize all constants for each pid */

    //roll pid
    roll_pid.tau  =  0.006f;  // 25Hz cutoff freq
    roll_pid.kp   =  2.1f;
    roll_pid.ki   =  0.0f;  
    roll_pid.kd   = -0.2f;
    
    //pitch pid
    pitch_pid.tau =  0.006f; // 25Hz cutoff freq
    pitch_pid.kp  =  2.1f;
    pitch_pid.ki  =  0.0f;
    pitch_pid.kd  = -0.2f;

    //yaw pid
    yaw_pid.tau =  0.008f;  // 25Hz cutoff freq
    yaw_pid.kp  = -2.1f;
    yaw_pid.ki  =  0.0f;
    yaw_pid.kd  =  0.0f;

    //yaw filter
    iir.tau = 0.007f;
    iir.samplingTime = 0.001f;
    Low_Pass_IIR_Filter_Init(&iir);

    PID_init(&roll_pid);
    PID_init(&pitch_pid);
    PID_init(&yaw_pid);
}

static float filter_input[2] = {0};
static float filter_output[2] = {0};
static float angles[2] = {0};
void Stabilize(float acc_buff[3], float gyro_buff[3], int8_t control_inputs[4]){
    float roll_val , pitch_val;
    float set_angles[3] = {0};
    float acc_angles[2];
    float angle_change[3];
    const float dt = 0.001f, alpha = 0.001f;
    int8_t duty_cycles[3] = {0};
    

    Calculate_Angles_acc(acc_buff, acc_angles);
    Calculate_Angular_Velocities(angle_change, angles, gyro_buff);
    Get_Complementary_Roll_Pitch(angles, acc_angles, angle_change, dt, alpha);

    filter_input[0] = angle_change[2];
    Low_Pass_IIR_Filter(&iir, filter_output, filter_input);
    angle_change[2] = filter_output[0];

    roll_val = radToDeg(angles[0]);
    pitch_val  = radToDeg(angles[1]);

    set_angles[0] = control_inputs[pitch] - 1.2f;  //+2.1
    set_angles[1] = control_inputs[roll] - 1.1f;  //+1.9

    /* Angle PID's */
    //pitch
    duty_cycles[1] = (int8_t) PID_Calculate(&roll_pid, pitch_val, set_angles[1]);
    //roll    
    duty_cycles[0] = (int8_t) PID_Calculate(&pitch_pid, roll_val, set_angles[0]); 
    //yaw
    duty_cycles[2] = (int8_t) PID_Calculate(&yaw_pid, angle_change[2], set_angles[2]);

    Motors_SetPWR(control_inputs[thrust], duty_cycles[2], duty_cycles[1], duty_cycles[0]);    
}
