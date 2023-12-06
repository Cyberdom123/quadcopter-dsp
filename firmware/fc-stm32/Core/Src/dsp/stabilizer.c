#include <dsp/stabilizer.h>

static pid_t roll_pid, pitch_pid;
static EMA_filter_t ema;
static float pitch_filtered[2], roll_filtered[2]; //previously defined in Stabilize
                                                  //thats mean angle multiplied by 0.456
void Stabilizer_init(){
    //initialize all pids sample_times and max_values
    roll_pid.maxOut = 75;
    roll_pid.minOut = -75;
    roll_pid.maxInt = 5;  //30
    roll_pid.minInt = -5; //-30 
    
    pitch_pid.maxOut = 55;
    pitch_pid.minOut = -55;
    pitch_pid.maxInt = 20; //30 
    pitch_pid.minInt= -20; //30 

    roll_pid.sampleTime = 0.001;
    pitch_pid.sampleTime = 0.001;

    //initialize all constants for each pid
    //roll pid
    roll_pid.kp = 0.9; //.8 
    roll_pid.ki = 0;  //0
    roll_pid.kd = 0;
    
    //pitch pid
    pitch_pid.kp = 0.3;
    pitch_pid.ki = 0;
    pitch_pid.kd = 0;


    //initialize low pass filters
    ema.alpha = 0.8; //250Hz

    PID_init(&roll_pid);
    PID_init(&pitch_pid);
}

void Stabilize(float acc_buff[3], float gyro_buff[3], int8_t command[8]){
    float angles[3];
    float roll, pitch;
    volatile float set_angles[3];
    volatile int8_t duty_cycles[3]; //roll, pitch, yaw
    
    Get_Roll_Pitch_Acc(acc_buff, angles);
    EMA_filter(&ema, pitch_filtered, angles[0]);
    EMA_filter(&ema, roll_filtered, angles[1]);


    pitch = (pitch_filtered[0]/3.1412)*180;
    roll = (roll_filtered[1]/3.1412)*180;
    //angles[2] = 0;

    set_angles[0] = command[3] * ROLL_ANGLE_SCALE;
    set_angles[1] = command[1] * PITCH_ANGLE_SCALE;
    //set_angles[2] = 0;

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
    //command2[3] = duty_cycles[0];
    //roll
    command2[1] = duty_cycles[1];
    
    Motors_Run(command2);
}
