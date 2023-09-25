// #include "rc.h"

// NRF24L01_STRUCT nrf24l01;

// HAL_StatusTypeDef RC_init(){
//     /* Initialize nrf24l01 */
//     nrf24l01.nrf24l01GpioPort = CE_GPIO_Port;
//     nrf24l01.csnPin = CSN_Pin;
//     nrf24l01.cePin = CE_Pin;
//     nrf24l01.spiHandle = &hspi1;

//     HAL_Delay(200);
//     HAL_StatusTypeDef status =  NRF24L01_Init(
//         &nrf24l01,
//         RX_DR_INT_SET,   //Interrupt not reflected on IRQ
//         TX_DS_INT_SET,   //Interrupt not reflected on IRQ
//         MAX_RT_INT_SET,  //Interrupt not reflected on IRQ
//         EN_CRC_SET,       //Enable CRC
//         ONE_BYTE_ENCODING,
//         PRX,
//         FIVE_BYTES_ADDR,
//         0xF, 0x7, //500uS delay, 15 re-transmissions
//         ONE_MBPS_DATA_RATE,
//         RF_POWER_1,
//         LNA_HCURR_SET
//     );
//     if(status != HAL_OK){ return status; }

//     status = NRF24L01_Open_Reading_Pipe(&nrf24l01, RX_ADDR_P1, 0xc2c2c2c2c2LL, 8);
//     if(status != HAL_OK){ return status; }
    
//     return NRF24L01_Chanel(&nrf24l01, 44);
// }
