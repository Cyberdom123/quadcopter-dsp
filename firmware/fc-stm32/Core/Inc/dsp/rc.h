#if !defined(RC)
#define RC
#include <stdint.h>

#define ACTIVATION_THRESHOLD  3
#define ACTIVATION_THRESHOLD_THRUST 10

#define ROLL_ANGLE_SCALE      0.15f
#define PITCH_ANGLE_SCALE     0.15f
#define YAW_ANGLE_SCALE       0.05f

typedef enum Control_inputs_t{
  thrust = 0,
  pitch  = 1,
  yaw    = 2,
  roll   = 3
}Control_Inputs_t;

typedef union Telemetry_t {
  float floatingPoint[6];
  uint8_t bytes[24];
}Telemetry_t;

typedef struct RC_t
{
  int8_t controls_inputs[4];
  int8_t power_on;
}RC_t;

void RC_Receive_Message(uint8_t message[8], RC_t *rc);

#endif // RC
