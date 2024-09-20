#include "flight_controller.h"
#include "main.h"

RC_t rc;
Telemetry_t telemetry;

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  (void)hi2c;

#ifdef HAL_RADIO_INTERFACE_I2C
  HAL_RADIO_receive_payload();
#endif // RADIO_INTERFACE_I2C

#ifdef HAL_IMU_INTERFACE_I2C
  HAL_IMU_readout();
#endif // IMU_INTERFACE_I2C
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  (void)hspi;

#ifdef HAL_RADIO_INTERFACE_SPI
  HAL_RADIO_receive_payload();
#endif // RADIO_INTERFACE_SPI

#ifdef HAL_IMU_INTERFACE_SPI
  HAL_IMU_readout();
#endif // IMU_INTERFACE_SPI

#ifdef HAL_EXTERNAL_MEMORY_INTERFACE_SPI
  HAL_EXTERNAL_MEMORY_readout();
#endif // EXTERNAL_MEMORY_INTERFACE_SPI
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == NRF_INT_Pin) { // Rename to RADIO_INT_Pin
    HAL_RADIO_write_telemetry_payload(telemetry.bytes, 24);
    HAL_RADIO_request_readout();
  }
  if (GPIO_Pin == MPU_INT_Pin) { // Rename to IMU_INT_Pin
    HAL_IMU_request_readout();
  }
}

void RADIO_packet_received_callback(const uint8_t *packet,
                                    uint8_t packet_length) {
  (void)packet_length;
  RC_Connection_Tick(&rc);
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
  RC_Receive_Message(packet, &rc);
}

void IMU_conversion_complete_callback(const float *acc, const float *gyro) {
  static float angle_change[3];
  static float angles[2];

  Estimate_Angles(angles, angle_change, acc, gyro);
  Stabilize(angles, angle_change, rc.controls_inputs);
  Motors_Switch(rc.power_on);

  telemetry.floatingPoint[0] = radToDeg(angles[0]);
  telemetry.floatingPoint[1] = radToDeg(angles[1]);

  telemetry.floatingPoint[2] = (float)rc.controls_inputs[thrust];
  telemetry.floatingPoint[3] = (float)rc.controls_inputs[pitch];
  telemetry.floatingPoint[4] = (float)rc.controls_inputs[yaw];
  telemetry.floatingPoint[5] = (float)rc.controls_inputs[roll];
}

void FC_init() {
  Stabilizer_init();
  const float dt = 0.001f, comp_alpha = 0.001f, iir_tau = 0.04f;
  Estimate_Angles_Init(dt, comp_alpha, iir_tau);

  HAL_RADIO_init(RADIO_packet_received_callback);
  HAL_IMU_init(IMU_conversion_complete_callback);

  HAL_IMU_start_conversion();
  HAL_RADIO_start_listening();
}

void FC_deinit() {
  HAL_RADIO_deinit();
  HAL_IMU_deinit();
}

void FC_proc() {
  if (RC_Check_Connection()) {
    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
    Lower_Altitude(&rc);
  }
}