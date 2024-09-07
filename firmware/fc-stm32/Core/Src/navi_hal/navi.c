#include "navi_hal/navi.h"

#include "baro.h"
#include "external_memory.h"
#include "imu.h"
#include "radio.h"

#include "rc.h"

#include "dsp/angle_estimation.h"
#include "dsp/stabilizer.h"

#include "main.h"
// #include "dma.h"
// #include "i2c.h"
// #include "spi.h"
// #include "tim.h"
// #include "usart.h"
#include "gpio.h"

// TODO: make these platform independent

// TODO: add struct
static RC_t rc;
static Telemetry_t telemetry;

void RADIO_packet_received_callback(const uint8_t *packet,
                                    uint8_t packet_length) {
  RC_Connection_Tick(&rc);
  // TODO: replace with component
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
  RC_Decode_Message(&rc, packet, packet_length);
}

void IMU_conversion_complete_callback(const float *acc, const float *gyro) {
  float angle_change[3]; // TODO add this buffer to angle_estimation.h
  float angles[2];

  Estimate_Angles(angles, angle_change, acc, gyro);
  Stabilize(angles, angle_change, rc.controls_inputs);
}

// void BARO_conversion_callback(const float *pressure, const float *temperature) {
//   // TODO: implement
// }

// Move this to platform specific file
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == NRF_INT_Pin) { // Rename to RADIO_INT_Pin
    RADIO_write_telemetry_payload(telemetry.bytes, 24);
    // RADIO_request_readout();
  }
  if (GPIO_Pin == MPU_INT_Pin) { // Rename to IMU_INT_Pin
    // IMU_request_readout();
  }
}

void NAVI_init() {
  // Stabilizer_init();
  // const float dt = 0.001f, comp_alpha = 0.001f, iir_tau = 0.04f;
  // Estimate_Angles_Init(dt, comp_alpha, iir_tau);
  // Initialize modules
  RADIO_init(RADIO_packet_received_callback);
  IMU_init(IMU_conversion_complete_callback);
  // BARO_init(BARO_conversion_callback);
}

void NAVI_deinit() {
  // Deinitialize modules
  // RADIO_deinit();
  // IMU_deinit();
  // BARO_deinit();
}

void NAVI_proc() {
  // RADIO_proc();
  // IMU_proc();
  // BARO_proc();
}