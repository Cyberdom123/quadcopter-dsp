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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "usbd_cdc_if.h"
#include "string.h"
#include <drivers/nrf24l01.h>
#include <drivers/mpu6050.h>
#include <drivers/motors.h>
#include <dsp/filters.h>
#include <dsp/angle_estimation.h>
#include <stabilizer.h>
#include <rc.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TELEMETRY
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
MPU6050_STRUCT mpu;
RC_t rc;

kalman_t kalman_pitch;
kalman_t kalman_roll;
float kalman_angle[2] = {0};

/* Declare buffers */
uint8_t command[8] = {0};
#if defined(TELEMETRY)
Telemetry_t telemetry;
#endif // TELEMETRY

FLOAT_TYPE acc_buff[3];
FLOAT_TYPE gyro_buff[3];

static IIR_filter_t iir;

static float filter_acc_x_in[2]  = {0};
static float filter_acc_x_out[2] = {0};

static float filter_acc_y_in[2]  = {0};
static float filter_acc_y_out[2] = {0};

static float filter_acc_z_in[2]  = {0};
static float filter_acc_z_out[2] = {0};

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
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
    NRF24L01_Read_PayloadDMA_Complete(&nrf24l01, command, 8);
    NRF24L01_Start_Listening(&nrf24l01);
  }

}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
  if(mpu.gyro_busy && mpu.acc_busy){
    MPU_read_acc_gyro_DMA_complete(&mpu);
    #if defined(TELEMETRY)
    float acc_angles[2] = {0};

    // const float dt = 0.001f, alpha = 0.001f; //good for vibrations bad for constant calibration 0.0001
    // float angle_change[3];

    filter_acc_x_in[0] = acc_buff[0];
    Low_Pass_IIR_Filter(&iir, filter_acc_x_out, filter_acc_x_in);
    acc_buff[0] = filter_acc_x_out[0];

    filter_acc_y_in[0] = acc_buff[1];
    Low_Pass_IIR_Filter(&iir, filter_acc_y_out, filter_acc_y_in);
    acc_buff[1] = filter_acc_y_out[0];

    filter_acc_z_in[0] = acc_buff[2];
    Low_Pass_IIR_Filter(&iir, filter_acc_z_out, filter_acc_z_in);
    acc_buff[2] = filter_acc_z_out[0];

    RC_Receive_Message(command, &rc);
    Stabilize(acc_buff, gyro_buff, rc.controls_inputs);
    Motors_Switch(rc.power_on);

    // Calculate_Angles_acc(acc_buff, acc_angles);
    // Calculate_Angular_Velocities(angle_change, angles, gyro_buff);
    // Get_Complementary_Roll_Pitch(angles, acc_angles, angle_change, dt, alpha);

    // Kalman_calculate(&kalman_pitch, &kalman_angle[0], radToDeg(acc_angles[0]), -gyro_buff[0]);
    // Kalman_calculate(&kalman_roll, &kalman_angle[1], radToDeg(acc_angles[1]), gyro_buff[1]);

    // telemetry.floatingPoint[0] = kalman_angle[0];
    // telemetry.floatingPoint[1] = radToDeg(acc_angles[0]);
    // telemetry.floatingPoint[2] = radToDeg(angles[0]);

    // telemetry.floatingPoint[3] = kalman_angle[1];
    // telemetry.floatingPoint[4] = radToDeg(acc_angles[1]);
    // telemetry.floatingPoint[5] = radToDeg(angles[1]);

    telemetry.floatingPoint[0] = (float) rc.controls_inputs[thrust];
    telemetry.floatingPoint[1] = (float) rc.controls_inputs[pitch];
    telemetry.floatingPoint[2] = (float) rc.controls_inputs[yaw];
    telemetry.floatingPoint[3] = (float) rc.controls_inputs[roll];
    telemetry.floatingPoint[4] = (float) rc.power_on;

    // for (size_t i = 0; i < 3; i++)
    // {
    //   telemetry.floatingPoint[i] = acc_buff[i];
    // }
    // for (size_t i = 0; i < 3; i++)
    // {
    //   telemetry.floatingPoint[3+i] = gyro_buff[i];
    // }
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
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  
  /* Initialize IO buffers */
  mpu.mpu_acc_buff = acc_buff;
  mpu.mpu_gyro_buff = gyro_buff;

  /* Initialize stabilizer */
  Stabilizer_init();

  kalman_pitch.sampling_time = 0.001;
  kalman_pitch.angular_velocity_variance = 16;
  kalman_pitch.angle_variance = 9;
  Kalman_init(&kalman_pitch);

  kalman_roll.sampling_time = 0.001;
  kalman_roll.angular_velocity_variance = 16;
  kalman_roll.angle_variance = 9;
  Kalman_init(&kalman_roll);

  iir.samplingTime = 0.001;
  iir.tau = 0.04;
  Low_Pass_IIR_Filter_Init(&iir);

  /* Initialize mpu */
  mpu.hi2c = &hi2c1;
  MPU6050_config mpu_cfg = MPU_get_default_cfg();
  MPU_init(&mpu, &mpu_cfg);
  MPU_measure_gyro_offset(&mpu, 8000);
  MPU_measure_acc_offset(&mpu, 8000);


  /* Initialize nrf24l01 */
  nrf24l01.nrf24l01GpioPort = CE_GPIO_Port;
  nrf24l01.csnPin = CSN_Pin;
  nrf24l01.cePin = CE_Pin;
  nrf24l01.spiHandle = &hspi1;

  NRF24L01_Init(&nrf24l01, &nrf24l01_default_config);

  #if defined(TELEMETRY)
  NRF24L01_Enable_ACKN_Payload(&nrf24l01);  
  uint64_t rx_addr_pipe = 0xc2c2c2c2c2LL;
  NRF24L01_Open_Reading_Pipe(&nrf24l01, RX_ADDR_P1, rx_addr_pipe, 8);
  #endif // TELEMETRY
  
  
  NRF24L01_Get_Info(&nrf24l01);
  NRF24L01_Start_Listening(&nrf24l01);

  HAL_StatusTypeDef status = MPU_clear_int(&mpu);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
