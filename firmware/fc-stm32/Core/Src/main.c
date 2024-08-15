/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "usbd_cdc_if.h"
#include <drivers/nrf24l01.h>
#include <drivers/ibus.h>
#include <drivers/w25q64.h>
#include <drivers/mpu6050.h>
#include <drivers/bmp280.h>
#include <drivers/motors.h>
#include <dsp/angle_estimation.h>
#include <stabilizer.h>
#include <rc.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TELEMETRY
#define SPI_FLASH
#define NRF
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Declare buffers */
NRF24L01_STRUCT nrf24l01;
MPU6050_STRUCT mpu;
bmp280_dev bmp;
W25Q64_STRUCT w25q64;

uint8_t message[8] = {0};
RC_t rc;
#if defined(TELEMETRY)
Telemetry_t telemetry;
#endif // TELEMETRY

FLOAT_TYPE acc_buff[3];
FLOAT_TYPE gyro_buff[3];

float angles[2] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if(nrf24l01.payloadFlag){
    RC_Connection_Tick();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    NRF24L01_Read_PayloadDMA_Complete(&nrf24l01, message, 8);
    RC_Receive_Message(message, &rc);
    NRF24L01_Start_Listening(&nrf24l01);
  }

}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
  if(mpu.gyro_busy && mpu.acc_busy){
    MPU_read_acc_gyro_DMA_complete(&mpu);
    #if defined(TELEMETRY)
    float angle_change[3];

    Estimate_Angles(angles, angle_change, acc_buff, gyro_buff);
    Stabilize(angles, angle_change, rc.controls_inputs);
    Motors_Switch(rc.power_on);

    telemetry.floatingPoint[0] = radToDeg(angles[0]);
    telemetry.floatingPoint[1] = radToDeg(angles[1]);

    telemetry.floatingPoint[2] = (float) rc.controls_inputs[thrust];
    telemetry.floatingPoint[3] = (float) rc.controls_inputs[pitch];
    telemetry.floatingPoint[4] = (float) rc.controls_inputs[yaw];
    telemetry.floatingPoint[5] = (float) rc.controls_inputs[roll];
    
    #endif // TELEMETRY
    
  }
  else if(mpu.gyro_busy){
    MPU_read_gyro_DMA_complete(&mpu);
  }
  else if(mpu.acc_busy){
    MPU_read_acc_DMA_complete(&mpu);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == NRF_INT_Pin){
    NRF24L01_Stop_Listening(&nrf24l01);
    #if defined(TELEMETRY)
    NRF24L01_Write_ACKN_Payload(&nrf24l01, telemetry.bytes, 24);    
    #endif // TELEMETRY
    
    NRF24L01_Read_PayloadDMA(&nrf24l01, 8);
  }
  if(GPIO_Pin == MPU_INT_Pin){
    MPU_read_acc_gyro_DMA(&mpu);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  
  /* Initialize IO buffers */
  mpu.mpu_acc_buff = acc_buff;
  mpu.mpu_gyro_buff = gyro_buff;

  /* Initialize stabilizer */
  Stabilizer_init();
  const float dt = 0.001f, comp_alpha = 0.001f, iir_tau = 0.04f;
  Estimate_Angles_Init(dt, comp_alpha, iir_tau);

  /* Initialize mpu */
  mpu.hi2c = &hi2c1;
  MPU6050_config mpu_cfg = MPU_get_default_cfg();
  MPU_init(&mpu, &mpu_cfg);
  const uint16_t samples = 8000;
  MPU_measure_gyro_offset(&mpu, samples);
  MPU_measure_acc_offset(&mpu, samples);

  /* Initialize additional */
  #if defined(BAROMETER)
  bmp.hi2c = &hi2c1;
  bmp.defoult_conf = true;
  bmp280_init(&bmp);
  bmp280_burst_read(&bmp);
  #endif // STM32F103xB
  

  /* Initialize W25Q64 */
  #if defined(SPI_FLASH)
  w25q64.cs_gpio = SPI_CS_GPIO_Port; 
  w25q64.cs_pin = SPI_CS_Pin;
  w25q64.hspi = &hspi1;
  W25Q64_Init(&w25q64);
  #endif

  /* Initialize nrf24l01 */
  #if defined(NRF)
  nrf24l01.nrf24l01GpioPort = CE_GPIO_Port;
  nrf24l01.csnPin = CSN_Pin;
  nrf24l01.cePin = CE_Pin;
  nrf24l01.spiHandle = &hspi1;
  #endif
  
  /* Initialize IBUS */
  #if defined(IBUS)
  // TODO
  #endif

  NRF24L01_Init(&nrf24l01, &nrf24l01_default_config);

  #if defined(TELEMETRY)
  NRF24L01_Enable_ACKN_Payload(&nrf24l01);  
  const uint64_t rx_addr_pipe = 0xc2c2c2c2c2LL;
  NRF24L01_Open_Reading_Pipe(&nrf24l01, RX_ADDR_P1, rx_addr_pipe, 8);
  #endif // TELEMETRY
  
  /* Start listening for incoming messages and enable interrupts */
  NRF24L01_Start_Listening(&nrf24l01);
  /* Enable mpu interrupts */
  MPU_clear_int(&mpu);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(RC_Check_Connection()){
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
      Lower_Altitude(&rc); 
    }
  } 
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
