/**
 * @file radio.h
 * @author Dominik Michalczyk
 * 
 * Radio module platform independent interface.
 */

#if !defined(RADIO_H_)
#define RADIO_H_

#include <stdbool.h>
#include <stdint.h>

typedef void (*RADIO_receive_callback_t)(const uint8_t* data, uint8_t len);

void RADIO_init(RADIO_receive_callback_t* radio_receive_callback);
void RADIO_deinit();
void RADIO_enable_telemetry();
void RADIO_start_listening();
void RADIO_write_telemetry_payload(const uint8_t* data, uint8_t len); // TODO: ADD CIRCULAR BUFFER

#endif // RADIO_H_
