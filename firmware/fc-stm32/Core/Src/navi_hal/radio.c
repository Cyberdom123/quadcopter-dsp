/**
 * @file radio.c
 * @author Dominik Michalczyk
 *
 * Radio module platform specific implementation.
 */

#include <assert.h>

#include "navi_hal/radio.h"
#include "drivers/nrf24l01.h"
#include "main.h"

enum
{
    PIPE_PAYLOAD_SIZE = 8,
    READING_PIPE_ADDRESS = 0xc2c2c2c2c2LL,
};

typedef struct RADIO_DEVICE
{
    NRF24L01_STRUCT nrf24l01;
    bool initialized;
    bool listening;
    bool telemetry_enabled;
};

static struct RADIO_DEVICE radio = {0};

void RADIO_init(RADIO_receive_callback_t *radio_receive_callback)
{
    assert(radio_receive_callback != NULL);

    NRF24L01_STRUCT *nrf24l01 = &radio.nrf24l01;
    nrf24l01->nrf24l01GpioPort = CE_GPIO_Port;
    nrf24l01->cePin = CE_Pin;
    nrf24l01->csnPin = CSN_Pin;
    nrf24l01->spiHandle = &hspi1;

    NRF24L01_CONFIG nrf24l01_config = NRF24L01_Get_Default_Config();
    nrf24l01_init(nrf24l01, &nrf24l01_config);
}

void RADIO_deinit()
{
}

void RADIO_enable_telemetry()
{
    if (!radio.initialized)
    {
        return;
    }
    NRF24L01_Enable_ACKN_Payload(&radio.nrf24l01);
    NRF24L01_Open_Reading_Pipe(&radio.nrf24l01, RX_ADDR_P1, READING_PIPE_ADDRESS, PIPE_PAYLOAD_SIZE);
    radio.telemetry_enabled = true;
}

void RADIO_start_listening()
{
    if (!radio.initialized)
    {
        return;
    }
    NRF24L01_Start_Listening(&radio.nrf24l01);
    radio.listening = true;
}

void RADIO_write_telemetry_payload(const uint8_t *data, uint8_t len)
{
    if (!radio.telemetry_enabled && !radio.listening)
    {
        return;
    }
    NRF24L01_Write_ACKN_Payload(&radio.nrf24l01, data, len);
}