/**
 * @file radio.h
 * @author Dominik Michalczyk
 *
 * Radio module platform independent interface.
 */

#ifndef RADIO_H_
#define RADIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef void (*RADIO_receive_callback_t)(const uint8_t *data, uint8_t len);

void HAL_RADIO_init(RADIO_receive_callback_t radio_receive_callback);
void HAL_RADIO_deinit();
void HAL_RADIO_proc();
void HAL_RADIO_enable_telemetry();
void HAL_RADIO_start_listening();
void HAL_RADIO_request_readout();
void HAL_RADIO_write_telemetry_payload(const uint8_t *data, uint8_t len); // TODO: ADD CIRCULAR BUFFER

#ifdef __cplusplus
}
#endif

#endif // RADIO_H_
