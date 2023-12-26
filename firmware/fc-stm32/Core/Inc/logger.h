#if !defined(LOGGER)
#define LOGGER

#include <w25q64.h>
#include <spi.h>
#include <stdbool.h>

#define MAX_BYTES_IN_PAGE   255
#define INITIAL_PAGE_POINTER 0x001000

// typedef union TELEMETRY_DATA
// {
//     float floating_point[2];
//     uint8_t bytes[8];
// }TELEMETRY_DATA;


HAL_StatusTypeDef Logger_Init();
void Logger_Log_Data(uint8_t *data, uint8_t size);
void Logger_Read_Data(uint8_t *data, uint8_t size);
void Logger_Erase();

#endif // LOGGER


