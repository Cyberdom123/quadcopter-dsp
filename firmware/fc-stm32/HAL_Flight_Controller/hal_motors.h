/**
 * @file hal_motors.h
 * @author Dominik Michalczyk
 * 
 * Motor control module platform independent interface.
 */

#ifndef HAL_MOTORS_H_
#define HAL_MOTORS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

void HAL_MOTORS_init();
void HAL_MOTORS_deinit();
void HAL_MOTORS_set(uint8_t thrust, int8_t yaw, int8_t pitch, int8_t roll, bool power_on);

#ifdef __cplusplus
}
#endif

#endif // HAL_MOTORS_H_