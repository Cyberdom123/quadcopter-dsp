#include "nrf24l01.h"

/* Initialize device */
HAL_StatusTypeDef NRF24L01_Init(NRF24L01_STRUCT *nrf24l01, uint8_t maskRX, uint8_t maskTx, uint8_t maskMaxRT,
                                uint8_t enCrc, CRCO_ENCODING crco, RX_TX_CONTROL rxTx, RX_TX_ADDRESS_WIDTH addrWidth,
                                uint8_t retDelay, uint8_t retCount, AIR_DATA_RATE rfDr, RF_POWER_SEL rfPwrSel,
                                uint8_t lnaGain)

{
    uint8_t data; HAL_StatusTypeDef status; 

    HAL_Delay(5); //Wait for device to power up 

    HAL_GPIO_WritePin(nrf24l01->nrf24l01GpioPort, nrf24l01->csnPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(nrf24l01->nrf24l01GpioPort, nrf24l01->cePin, GPIO_PIN_RESET);

    /* Setup Address Widths */
    data = addrWidth;
    status = NRF24L01_Write_Byte(nrf24l01, SETUP_AW, data);
    if(status != HAL_OK){ return status; }
    
    /* Setup Retransmission */
    data = (retDelay << ARD | retCount);
    status = NRF24L01_Write_Byte(nrf24l01, SETUP_RETR, data);
    if(status != HAL_OK){ return status; }

    /* RF Setup */
    data = (rfDr << RF_DR| rfPwrSel << RF_PWR | lnaGain);
    status = NRF24L01_Write_Byte(nrf24l01, RF_SETUP, data);
    if(status != HAL_OK){ return status; }

    /* Disable Dynamic Payloads */
    data = 0;
    status = NRF24L01_Write_Byte(nrf24l01, DYNPD, data);
    if(status != HAL_OK){ return status; }

    /* Clear Status Register */
    data = (1 << MASK_RX_DR | 1 << MASK_TX_DS | 1 << MASK_MAX_RT);
    status = NRF24L01_Write_Byte(nrf24l01, NRF_STATUS, data);
    if(status != HAL_OK){ return status; }

    /* FLush RX TX */
    NRF24L01_Flush_Tx(nrf24l01);
    NRF24L01_Flush_Tx(nrf24l01);
    
    /* Config & Start Device*/
    data = ( maskRX << MASK_RX_DR | maskTx << MASK_TX_DS | maskMaxRT << MASK_MAX_RT | enCrc << EN_CRC_MASK | crco << CRCO | 1 << PWR_UP | rxTx); 
    status = NRF24L01_Write_Byte(nrf24l01, NRF_CONFIG, data);
    if(status != HAL_OK){ return status; }

    HAL_Delay(2);

    return HAL_OK;
}

/* Set rf chanel */
HAL_StatusTypeDef NRF24L01_Chanel(NRF24L01_STRUCT *nrf24l01, uint8_t rfCh)
{
    return NRF24L01_Write_Byte(nrf24l01, RF_CH, rfCh);
}

/* Read one byte from device */
HAL_StatusTypeDef NRF24L01_Read_Byte(NRF24L01_STRUCT *nrf24l01, uint8_t addr, uint8_t *data)
{
  HAL_GPIO_WritePin(nrf24l01->nrf24l01GpioPort, nrf24l01->csnPin, GPIO_PIN_RESET);
  uint8_t tx_buff = R_REGISTER | addr;
  uint8_t rx_buff[2];
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(nrf24l01->spiHandle, &tx_buff, rx_buff, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(nrf24l01->nrf24l01GpioPort, nrf24l01->csnPin, GPIO_PIN_SET);
  *data = rx_buff[1];
  
  return status;
}

/* Write one byte to device */
HAL_StatusTypeDef NRF24L01_Write_Byte(NRF24L01_STRUCT *nrf24l01, uint8_t addr, uint8_t data)
{
    HAL_GPIO_WritePin(nrf24l01->nrf24l01GpioPort, nrf24l01->csnPin, GPIO_PIN_RESET);
    uint8_t tx_buff[2] = {W_REGISTER | addr, data}; 
    HAL_StatusTypeDef status = HAL_SPI_Transmit(nrf24l01->spiHandle, tx_buff, 2, HAL_MAX_DELAY);
    while(HAL_SPI_GetState(nrf24l01->spiHandle) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(nrf24l01->nrf24l01GpioPort, nrf24l01->csnPin, GPIO_PIN_SET);
    
    return status;
}

/* Write number of bytes to device */
HAL_StatusTypeDef NRF24L01_Write(NRF24L01_STRUCT *nrf24l01, uint8_t addr, uint64_t data, uint8_t len)
{
    HAL_GPIO_WritePin( nrf24l01->nrf24l01GpioPort, nrf24l01->csnPin, GPIO_PIN_RESET);
    
    uint8_t tx_buff = {W_REGISTER | addr};
    HAL_StatusTypeDef status = HAL_SPI_Transmit(nrf24l01->spiHandle, &tx_buff, 1, HAL_MAX_DELAY);
    if(status != HAL_OK){ return status; }
    while(HAL_SPI_GetState(nrf24l01->spiHandle) != HAL_SPI_STATE_READY);

    status = HAL_SPI_Transmit(nrf24l01->spiHandle, &data, len, HAL_MAX_DELAY);
    if(status != HAL_OK){ return status; }
    while(HAL_SPI_GetState(nrf24l01->spiHandle) != HAL_SPI_STATE_READY);

    HAL_GPIO_WritePin(nrf24l01->nrf24l01GpioPort, nrf24l01->csnPin, GPIO_PIN_SET);
    
    return status;
}

/* Read number of bytes from device */
HAL_StatusTypeDef NRF24L01_Read(NRF24L01_STRUCT *nrf24l01, uint8_t addr, uint64_t *data, uint8_t len)
{
    HAL_GPIO_WritePin(nrf24l01->nrf24l01GpioPort, nrf24l01->csnPin, GPIO_PIN_RESET);

    uint8_t tx_buff = (R_REGISTER | addr);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(nrf24l01->spiHandle, &tx_buff, 1, HAL_MAX_DELAY);
    if(status != HAL_OK){ return status; }

    uint8_t tx_dummy[len];
    status = HAL_SPI_TransmitReceive(nrf24l01->spiHandle, tx_dummy, data, len, HAL_MAX_DELAY);
    if(status != HAL_OK){ return status; }

    HAL_GPIO_WritePin(nrf24l01->nrf24l01GpioPort, nrf24l01->csnPin, GPIO_PIN_SET);
    
    return status;
}


/* Flush Tx */
HAL_StatusTypeDef NRF24L01_Flush_Tx(NRF24L01_STRUCT *nrf24l01)
{
    HAL_GPIO_WritePin(nrf24l01->nrf24l01GpioPort, nrf24l01->csnPin, GPIO_PIN_RESET);
    uint8_t tx_buff = FLUSH_TX; 
    HAL_StatusTypeDef status = HAL_SPI_Transmit(nrf24l01->spiHandle, &tx_buff, 1, HAL_MAX_DELAY);
    while(HAL_SPI_GetState(nrf24l01->spiHandle) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(nrf24l01->nrf24l01GpioPort, nrf24l01->csnPin, GPIO_PIN_SET);
    
    return status ;
}

/* Flush Rx */
HAL_StatusTypeDef NRF24L01_Flush_Rx(NRF24L01_STRUCT *nrf24l01)
{
    HAL_GPIO_WritePin(nrf24l01->nrf24l01GpioPort, nrf24l01->csnPin, GPIO_PIN_RESET);
    uint8_t tx_buff = FLUSH_RX; 
    HAL_StatusTypeDef status = HAL_SPI_Transmit(nrf24l01->spiHandle, &tx_buff, 1, HAL_MAX_DELAY);
    while(HAL_SPI_GetState(nrf24l01->spiHandle) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(nrf24l01->nrf24l01GpioPort, nrf24l01->csnPin, GPIO_PIN_SET);
    
    return status;
}

/* WRITE PAYLOAD TO TX FIFO */
HAL_StatusTypeDef NRF24L01_Write_Tx_Payload(NRF24L01_STRUCT *nrf24l01, void *payload, uint8_t len)
{
    uint8_t *data = (uint8_t*) payload; 
    HAL_GPIO_WritePin(nrf24l01->nrf24l01GpioPort, nrf24l01->csnPin, GPIO_PIN_RESET);

    uint8_t tx_buff = W_TX_PAYLOAD;
    HAL_StatusTypeDef status = HAL_SPI_Transmit(nrf24l01->spiHandle, &tx_buff, 1, HAL_MAX_DELAY);
    while(HAL_SPI_GetState(nrf24l01->spiHandle) != HAL_SPI_STATE_READY);
    if(status != HAL_OK){ return status; }
    
    status = HAL_SPI_Transmit(nrf24l01->spiHandle, data, len, HAL_MAX_DELAY); 
    while(HAL_SPI_GetState(nrf24l01->spiHandle) != HAL_SPI_STATE_READY);      
    if(status != HAL_OK){ return status; }                                    

    HAL_GPIO_WritePin(nrf24l01->nrf24l01GpioPort, nrf24l01->csnPin, GPIO_PIN_SET);
    
    return HAL_OK;
}

/* SEND PAYLOAD OVER THE AIR */
HAL_StatusTypeDef NRF24L01_Send_Payload(NRF24L01_STRUCT *nrf24l01)
{
    HAL_GPIO_WritePin(CE_GPIO_Port, nrf24l01->cePin, GPIO_PIN_SET);
    TIM1_Delay_Microseconds(15);
    HAL_GPIO_WritePin(CE_GPIO_Port, nrf24l01->cePin, GPIO_PIN_RESET);

    uint8_t data;
    HAL_StatusTypeDef status = NRF24L01_Read_Byte(nrf24l01, NRF_STATUS, &data);
    if(status != HAL_OK){ return status; }                                    
    /* If max retransmissions occur clear MAX_RT flag to enable further communication */
    return NRF24L01_Write_Byte(nrf24l01, NRF_STATUS, (data | (1 << MASK_MAX_RT)));
}

/* READ PAYLOAD FROM RX FIF0 */
HAL_StatusTypeDef NRF24L01_Read_Payload(NRF24L01_STRUCT *nrf24l01, uint8_t *data, uint8_t len)
{
    uint8_t tx_buff = R_RX_PAYLOAD;
    uint8_t rx_buff[len];
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(nrf24l01->spiHandle, &tx_buff, rx_buff, len+1, HAL_MAX_DELAY);
    while(HAL_SPI_GetState(nrf24l01->spiHandle) != HAL_SPI_STATE_READY);
    return status;
}

/* OPEN READING PIPE FOR RX DATA */
HAL_StatusTypeDef NRF24L01_Open_Reading_Pipe(NRF24L01_STRUCT *nrf24l01, uint8_t pipeAddr, uint64_t rxAddr, uint8_t payloadSize)
{   
    HAL_StatusTypeDef status; 
    if(pipeAddr == RX_ADDR_P0 || pipeAddr == RX_ADDR_P1){
        status = NRF24L01_Write(nrf24l01, pipeAddr, rxAddr, 5);
    }else{
        status = NRF24L01_Write_Byte(nrf24l01, pipeAddr, rxAddr);
    }
    if(status != HAL_OK){ return status; }                                    

    status = (nrf24l01, (pipeAddr + 0x06), payloadSize);
    if(status != HAL_OK){ return status; }                                    

    /* Enable data pipe */
    return NRF24L01_Write_Byte(nrf24l01, EN_RXADDR, 1<<(pipeAddr - 0xa));

}

/* OPEN WRITING PIPE FOR TX DATA */
HAL_StatusTypeDef NRF24L01_Open_Writing_Pipe(NRF24L01_STRUCT *nrf24l01, uint64_t txAddr)
{
    HAL_StatusTypeDef status = NRF24L01_Write(nrf24l01, RX_ADDR_P0, txAddr, 5);  
    if(status != HAL_OK){ return status; }

    return NRF24L01_Write(nrf24l01, TX_ADDR, txAddr, 5);
}

/* READ REGISTER DATA FOR DEBUG */
HAL_StatusTypeDef NRF24L01_Print_Info(NRF24L01_STRUCT *nrf24l01){
    uint8_t data = 0; uint64_t data_long = 0;

    HAL_StatusTypeDef status = NRF24L01_Read_Byte(nrf24l01, NRF_CONFIG, &data);
    if(status != HAL_OK){ return status; }

    status = NRF24L01_Read_Byte(nrf24l01, RF_SETUP, &data);
    if(status != HAL_OK){ return status; }

    status = NRF24L01_Read_Byte(nrf24l01, SETUP_AW, &data);
    if(status != HAL_OK){ return status; }

    status = NRF24L01_Read(nrf24l01, TX_ADDR, &data_long, 5);
    if(status != HAL_OK){ return status; }

    status = NRF24L01_Read_Byte(nrf24l01, EN_AA, &data);
    if(status != HAL_OK){ return status; }

    status = NRF24L01_Read_Byte(nrf24l01, EN_RXADDR, &data);
    if(status != HAL_OK){ return status; }

    status = NRF24L01_Read_Byte(nrf24l01, DYNPD, &data);
    if(status != HAL_OK){ return status; }

    status = NRF24L01_Read_Byte(nrf24l01, RF_CH, &data);
    if(status != HAL_OK){ return status; }

    status = NRF24L01_Read_Byte(nrf24l01, NRF_STATUS, &data);
    if(status != HAL_OK){ return status; }

    status = NRF24L01_Read_Byte(nrf24l01, OBSERVE_TX, &data);
    if(status != HAL_OK){ return status; }

    return HAL_OK;
}
