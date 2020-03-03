/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Lib/xpt/XPT2046_touch.h"
//#include "../Lib/ili9341/ili9341.h"
#include "../Lib/ili9341_v2/ILI9341_STM32_Driver.h"
#include "../Lib/ili9341_v2/ILI9341_GFX.h"
#include "../Lib/bme280/bmp280.h"
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
I2C_HandleTypeDef hi2c1;

RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
BMP280_HandleTypedef bmp280;

float pressure, temperature, humidity;

uint16_t size;
uint8_t Data[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_RNG_Init(void);
static void MX_I2C1_Init(void);
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
  MX_RNG_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //V1
//  ILI9341_Init();
//  HAL_Delay(1000);
//  ILI9341_FillScreen(ILI9341_BLACK);
//  ILI9341_WriteString(0, 0, "some text", Font_16x26, ILI9341_WHITE, ILI9341_BLACK);
//  ILI9341_WriteString(0, 0, "some text", Font_16x26, ILI9341_RED, ILI9341_BLACK);
//  ILI9341_WriteString(0, 0, "1", Font_16x26, ILI9341_RED, ILI9341_BLACK);
  //V2
  ILI9341_V2_Init();
  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
  ILI9341_Fill_Screen(BLACK);

  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_0;
  bmp280.i2c = &hi2c1;
  while (!bmp280_init(&bmp280, &bmp280.params)) {
  		size = sprintf((char *)Data, "BMP280 initialization failed\n");
  		ILI9341_Draw_Text(Data, 0, 0, RED, 10, BLACK);
//  		HAL_UART_Transmit(&huart1, Data, size, 1000);
  		HAL_Delay(2000);
  }
  bool bme280p = bmp280.id == BME280_CHIP_ID;
  size = sprintf((char *)Data, "BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
  ILI9341_Draw_Text(Data, 0, 0, RED, 1, BLACK);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  ILI9341_Draw_Hollow_Circle(100, 100, 50, RED);
//  ILI9341_Draw_Hollow_Circle(200, 200, 50, GREEN);
  while (1)
  {

//	  ILI9341_Draw_Hollow_Circle(100, 100, 50, RED);
//	  if (XPT2046_TouchPressed()) {
//  		  uint16_t x = 0;
//  		  uint16_t y = 0;
//  		  XPT2046_TouchGetCoordinates(&x, &y);
//  		  if (x > 0 || y > 0 || x < 0 || y < 0) {
//  			ILI9341_Draw_Hollow_Circle(x, y, 50, BLUE);
//  		  }
//	  }
	  //V1
//	  HAL_Delay(500);
//	  if (XPT2046_TouchPressed()) {
//		  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
//		  uint16_t x = 0;
//		  uint16_t y = 0;
//		  XPT2046_TouchGetCoordinates(&x, &y);
//		  if (x > 0 || y > 0 || x < 0 || y < 0) {
//			  ILI9341_Draw_Hollow_Circle(x, y, 50, BLUE);
//			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
//		  } else {
//			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
//		  }
//		  printf("%d %d", x, y);
//	  } else {
//		  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
//	  }
		HAL_Delay(100);
		while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
			size = sprintf((char *)Data,
					"Temperature/pressure reading failed\n");
			ILI9341_Draw_Text(Data, 0, 0, RED, 1, BLACK);
			HAL_Delay(2000);
		}

		uint8_t temp_s[256];
		uint8_t pressure_s[256];
		uint8_t humidity_s[256];

		sprintf((char *)temp_s,"Temperature: %.2f C", temperature);
		sprintf((char *)pressure_s,"Pressure: %.2f Pa", pressure);
		sprintf((char *)humidity_s,"Humidity: %.2f", humidity);

		ILI9341_Draw_Text(temp_s, 0, 0, RED, 1, BLACK);
		ILI9341_Draw_Text(pressure_s, 0, 20, RED, 1, BLACK);
		ILI9341_Draw_Text(humidity_s, 0, 40, RED, 1, BLACK);
		HAL_Delay(2000);
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(XPT_CS_PIN_GPIO_Port, XPT_CS_PIN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PF9 PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_RS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : XPT_IRQ_PIN_Pin */
  GPIO_InitStruct.Pin = XPT_IRQ_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(XPT_IRQ_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : XPT_CS_PIN_Pin */
  GPIO_InitStruct.Pin = XPT_CS_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(XPT_CS_PIN_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
