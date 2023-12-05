#include <dsp/stabilizer.h>

static pid_t roll_pid, pitch_pid;

void Stabilizer_init(){
    //initialize all pids sample_times and max_values
    roll_pid.maxOut = 55;
    roll_pid.minOut = -55;
    roll_pid.maxInt = 20;  //30
    roll_pid.minInt = -20; //-30 
    
    pitch_pid.maxOut = 55;
    pitch_pid.minOut = -55;
    pitch_pid.maxInt = 20; //30 
    pitch_pid.minInt= -20; //030 

    roll_pid.sampleTime = 0.001;
    pitch_pid.sampleTime = 0.001;

    //initialize all constants for each pid
    //roll pid
    roll_pid.kp = 0.8; //0.4
    roll_pid.ki = 0.8; //0.1
    roll_pid.kd = 0.0005;
    
    //pitch pid
    pitch_pid.kp = 0.3;
    pitch_pid.ki = 1.5;
    pitch_pid.kd = 0;

    PID_init(&roll_pid);
    PID_init(&pitch_pid);
}

void Stabilize(float acc_buff[3], float gyro_buff[3], int8_t command[8]){
    float angles[3];
    volatile float set_angles[3];
    volatile int8_t duty_cycles[3]; //roll, pitch, yaw
    
    Get_Roll_Pitch_Acc(acc_buff, angles);

    angles[0] = (angles[0]/3.1412)*180;
    angles[1] = (angles[1]/3.1412)*180;
    angles[2] = 0;

    set_angles[0] = command[3] * ROLL_ANGLE_SCALE;
    set_angles[1] = command[1] * PITCH_ANGLE_SCALE;
    set_angles[2] = 0;

    //pitch
    duty_cycles[0] = (int8_t) PID_Calculate(&pitch_pid, angles[0], set_angles[0]); 
    //roll    
    duty_cycles[1] = (int8_t) PID_Calculate(&roll_pid, angles[1], set_angles[1]);
    //yaw
    duty_cycles[2] = 0;


    int8_t command2[8];
    for (size_t i = 0; i < 8; i++)
    {
        command2[i] = command[i];
    }
    
    command2[0] = command2[0] - 100;
    //command2[3] = duty_cycles[0]; //pitch
    command2[1] = duty_cycles[1]; //roll
    Motors_Run(command2);
}
