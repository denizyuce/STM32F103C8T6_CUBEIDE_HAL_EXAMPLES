/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#define adxl_address 0x53<<1
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static char* intro="ADXL345 Sensöründen İvme Verilerini Okuma\r\n";
char buffer[50];
uint8_t chipid=0;
uint8_t axis_data[6];
int16_t x_axis ,y_axis ,z_axis;
float xg,yg,zg;
uint8_t sample_tic=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void buffer_clear(char *buffer,uint8_t size){
	for (uint8_t i=0;i<size;i++){
		buffer[i]='\0';
	}
}
void adxl345_write(uint8_t adress,uint8_t value){
	uint8_t data[2];
	data[0]=adress; //Çok baytlı yazmayı aktif ediyoruz.
	data[1]=value;
	HAL_I2C_Master_Transmit (&hi2c1, adxl_address, data, 2, 100);
}
void adxl345_read(uint8_t adress){
	HAL_I2C_Mem_Read (&hi2c1, adxl_address, adress, 1, (uint8_t *)axis_data, 6, 100);
}
void adxl345_read_address (uint8_t adress)
{
	HAL_I2C_Mem_Read (&hi2c1, adxl_address, adress, 1, &chipid, 1, 100);
}
void adxl345_init(void){
	adxl345_read_address (0x00); // read the DEVID
	adxl345_write(0x31, 0x01); //Data format aralığı +/- 2g seçilir
	adxl345_write(0x2d, 0x00); //İlgili register temizlenir.
	adxl345_write(0x2d, 0x08); //Ölçüm modunda ve 8Hz 'de uyanma gerçekleşir.
}
void adxl345_Data_Collect(void){
	 adxl345_read(0x32);

		  x_axis=((axis_data[1]<<8) | axis_data[0]);
		  y_axis=((axis_data[3]<<8) | axis_data[2]);
		  z_axis=((axis_data[5]<<8) | axis_data[4]);

		  xg=x_axis*.0078125;
		  yg=y_axis*.0078125;
		  zg=z_axis*.0078125;
}
void data_push_to_serialport(void){
		  sprintf(buffer,"\r\nxg :%f\r\nyg :%f\r\nzg :%f\r\n",xg,yg,zg);
		  HAL_UART_Transmit(&huart1,(uint8_t*)buffer, sizeof(buffer), 100);
		  buffer_clear(buffer,50);
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  adxl345_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 if (sample_tic==1){
		 HAL_NVIC_DisableIRQ(TIM2_IRQn);
	 	 HAL_UART_Transmit(&huart1, (uint8_t*)intro,42, 100);
		 sprintf(buffer,"\nChip ID :%u\r\n",chipid);
		 HAL_UART_Transmit(&huart1,(uint8_t*)buffer, sizeof(buffer), 100);
		 for(int i=0;i<3;i++){
			 adxl345_Data_Collect();
			 data_push_to_serialport();
		 }
		sample_tic=0;
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
	 }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
