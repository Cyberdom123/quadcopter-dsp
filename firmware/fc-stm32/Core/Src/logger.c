// #include <logger.h>

// static W25Q64_STRUCT w25q = {
//     .hspi = &hspi1,
//     .cs_gpio = SPI_CS_GPIO_Port,
//     .cs_pin = SPI_CS_Pin,
// };

// static union page_pointer
// {
//     uint32_t uint32;     // points to the last saved data in spi flash
//     uint8_t bytes[4];
// }page_pointer;

// static uint16_t bytes_cnt = 0;                // count written bytes in page
// static uint8_t page_buffer[256] = {0};
// static uint32_t page_pointer_address = 0x000000;     // points to the last saved data in spi flash

// HAL_StatusTypeDef Logger_Init()
// {
//     HAL_StatusTypeDef status = W25Q64_Init(&w25q);
//     //read page_pointer
//     status = W25Q64_Read_Data(&w25q, page_pointer_address, page_pointer.bytes, 3);

//     //if page_pointer doesn't init yet, init page pointer
//     if(page_pointer.uint32 == 0xFFFFFF){
//         page_pointer.uint32 = INITIAL_PAGE_POINTER;
//         W25Q64_Write_Page(&w25q, page_pointer_address, page_pointer.bytes, 3);
//     }    

//     return status;
// }

// void Logger_Log_Data(uint8_t *data, uint8_t size){    
//     if(bytes_cnt + size < MAX_BYTES_IN_PAGE){
//         for (size_t i = 0; i < size; i++)
//         {
//             page_buffer[bytes_cnt + i] = data[i]; 
//         }
//     }else{
//         //Replace with DMA
//         W25Q64_Write_Page(&w25q, page_pointer.uint32, page_buffer, MAX_BYTES_IN_PAGE);
//         page_pointer.uint32 += bytes_cnt;
//         W25Q64_Write_Page(&w25q, page_pointer_address, page_pointer.bytes, 3);
//         bytes_cnt = 0;

//         for (size_t i = 0; i < size; i++)
//         {
//             page_buffer[bytes_cnt + i] = data[i]; 
//         }
//     }
//     bytes_cnt += size;
// }

// void Logger_Read_Data(uint8_t *data, uint8_t size){
//     W25Q64_Read_Data(&w25q, INITIAL_PAGE_POINTER, data, size);
// }

// void Logger_Erase(){
//     W25Q64_Chip_Erase(&w25q);
// }