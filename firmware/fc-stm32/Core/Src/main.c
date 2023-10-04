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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "nrf24l01.h"
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
NRF24L01_STRUCT nrf24l01;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);

  /* Initialize nrf24l01 */
  nrf24l01.nrf24l01GpioPort = CE_GPIO_Port;
  nrf24l01.csnPin = CSN_Pin;
  nrf24l01.cePin = CE_Pin;
  nrf24l01.spiHandle = &hspi1;

  HAL_Delay(200);
  HAL_StatusTypeDef status =  NRF24L01_Init(
    &nrf24l01,
    RX_DR_INT_SET,   //Interrupt not reflected on IRQ
    TX_DS_INT_SET,   //Interrupt not reflected on IRQ
    MAX_RT_INT_SET,  //Interrupt not reflected on IRQ
    EN_CRC_SET,       //Enable CRC
    ONE_BYTE_ENCODING,
    PRX,
    FIVE_BYTES_ADDR,
    0xF, 0x7, //500uS delay, 15 re-transmissions
    ONE_MBPS_DATA_RATE,
    RF_POWER_1,
    LNA_HCURR_SET
  );

  //NRF24L01_Open_Writing_Pipe(&nrf24l01, 0xc2c2c2c2c2LL);
  NRF24L01_Open_Reading_Pipe(&nrf24l01, RX_ADDR_P1, 0xc2c2c2c2c2LL, 8);
  
  if(status == HAL_OK){
    HAL_Delay(200);
  }
  NRF24L01_Chanel(&nrf24l01, 44);

  NRF24L01_Get_Info(&nrf24l01);
  NRF24L01_Start_Listening(&nrf24l01);
  //char payload[18] = "hello_from_stm32";

  FLOAT_TYPE acc_data[3];
  FLOAT_TYPE gyro_data[3];

  volatile HAL_StatusTypeDef mpu_status = HAL_OK;

  MPU6050_STRUCT mpu;
  mpu.hi2c = &hi2c1;
  uint8_t who_am_i = 0;


  MPU6050_config mpu_config = MPU_get_default_cfg();

  mpu_status = mpu6050_read_byte(&mpu, 0x75, &who_am_i);
  mpu_status = MPU_init(&mpu, &mpu_config);

  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  // HAL_Delay(1000);

  // TIM2->CCR1 = 30;
  // TIM2->CCR2 = 30;
  // TIM2->CCR3 = 30;
  // TIM2->CCR4 = 30;

  // HAL_Delay(1000);

  // TIM2->CCR1 = 40;
  // TIM2->CCR2 = 40;
  // TIM2->CCR3 = 40;
  // TIM2->CCR4 = 40;

  // HAL_Delay(1000);

  // TIM2->CCR1 = 45;
  // TIM2->CCR2 = 45;
  // TIM2->CCR3 = 45;
  // TIM2->CCR4 = 45;

  // HAL_Delay(1000);

  // TIM2->CCR1 = 50;
  // TIM2->CCR2 = 50;
  // TIM2->CCR3 = 50;
  // TIM2->CCR4 = 50 ;

  // HAL_Delay(1000);

  // TIM2->CCR1 = 55;
  // TIM2->CCR2 = 55;
  // TIM2->CCR3 = 55;
  // TIM2->CCR4 = 55 ;

  // HAL_Delay(1000);

  // TIM2->CCR1 = 53;
  // TIM2->CCR2 = 53;
  // TIM2->CCR3 = 53;
  // TIM2->CCR4 = 53;

  // HAL_Delay(1000);

  // TIM2->CCR1 = 40;
  // TIM2->CCR2 = 40;
  // TIM2->CCR3 = 40;
  // TIM2->CCR4 = 40;

  // HAL_Delay(1000);

  // TIM2->CCR1 = 20;
  // TIM2->CCR2 = 20;
  // TIM2->CCR3 = 20;
  // TIM2->CCR4 = 20;

  // HAL_Delay(2000);
  // HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
  // HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
  // HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
  char msg[8];
  uint8_t power_on = 0;
  uint8_t pwr = 10;
  while (1)
  {
    // NRF24L01_Send(&nrf24l01, payload, 18);

    // NRF24L01_Print_Info(&nrf24l01);
    // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
    // HAL_Delay(500);

    TIM2->CCR1 = pwr;
    TIM2->CCR2 = pwr;
    TIM2->CCR3 = pwr;
    TIM2->CCR4 = pwr;

    if(power_on){
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);   
    }else{
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
    }


    NRF24L01_Get_Info(&nrf24l01);
    HAL_Delay(1);
    if(NRF24L01_Packet_Available(&nrf24l01) == HAL_OK){
      NRF24L01_Read_Payload(&nrf24l01, (uint8_t*) msg, 8);
      
      pwr = 10;
      if(msg[0]-70 > 10){
        pwr = msg[0] - 70;
      }
      
      if(msg[2] == 1 && power_on == 0){
        power_on = 1;
      }

      if(msg[3] == 1 && power_on == 1){
        power_on = 0;
      }

      NRF24L01_Flush_Rx(&nrf24l01);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
      //HAL_Delay(100);


      NRF24L01_Write_Byte(&nrf24l01, NRF_STATUS, (1<<MASK_RX_DR) | (1<<MASK_TX_DS) | (1<<MASK_MAX_RT));
    }else{
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    }
    HAL_Delay(1);

    mpu_status = MPU_read_acc(&mpu, acc_data);
    mpu_status = MPU_read_gyro(&mpu, gyro_data);

  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
