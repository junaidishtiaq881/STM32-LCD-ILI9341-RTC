/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stdlib.h"
#include <stdbool.h>
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
//#include "testimg.h"
#include "fonts.h"
#include "UI.h"
#include "RTC.h"
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

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
//
//#define True  1
#define False 0
uint8_t button;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t currentTime;
uint32_t prevTime;
uint32_t counterOutside = 0;
uint32_t counterInside = 0;
bool state = true;
uint8_t seconds;
uint8_t minutes;
uint8_t hours;
uint8_t updated_hours;
uint8_t updated_mins;
uint8_t updated_secs;
uint8_t hours1 ;
uint8_t counter = 0;
int16_t count;

uint16_t F_SET_RTC = 0;
uint16_t F_SHOW_RTC = 0;
uint16_t F_SET_RTC_M = 0;
uint16_t F_SET_RTC_SEC = 0;
uint16_t button_pressed = 0;
int16_t count1 = 0;
uint8_t h_count = 0;
uint32_t currentTime_;
uint32_t prevTime_;

void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)
{

	 currentTime_ = HAL_GetTick();
	counter = __HAL_TIM_GET_COUNTER(htim);
//	count1 = (int16_t )counter;
//
//
	 if( __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) == False && currentTime_ - prevTime_ >= 200){
		 count++;
		 if(count > 1 ){
		  count = 1;
		 }
	 }
	 else{
		 count--;
		 if(count < 0 ){
		    count = 0;
				 }
	 }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 currentTime = HAL_GetTick();
	if ((HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1)== 0)&& currentTime - prevTime >= 500 ) {

		button_pressed = 1;
	//if	(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1)== 0){
      //    hours = pos; //updating hours
      //    }
		 F_SET_RTC = 1;
	     F_SHOW_RTC = 1;
	     F_SET_RTC_M = 1;
	     F_SET_RTC_SEC = 1;
	     prevTime = currentTime;
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
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  DS3231_Init(&hi2c1);

 char TimeBuffer[30];

//   SetSeconds(6);
//   SetMinutes(32);
//   SetHours(12);

   // HAL_Delay(1000);

  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);

  show_menu();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

/***************************** Showing Current Time ******************************/

	  		if (count == 0 && F_SHOW_RTC == 1){
	  			button_pressed = 0;

	  		  while (button_pressed == 0)
	  		 {
	  			  hours = GetHours();
	  			  minutes = GetMinutes();
	  			  seconds  = GetSeconds();

	  				  printf("Current seconds: :%d\n", seconds);
	  				  //minutes = GetMinutes();
	  				  printf("Current minutes: :%d\n", minutes);
	  				// hours = updated_hours; //updating hours

	  				  printf("Hours: %d\n", hours);

	  		        ILI9341_DrawText("Current Time     ", FONT4, 10, 10,BLACK, WHITE);

	  		        sprintf(TimeBuffer,"%02d: %02d: %02d          ",hours, minutes,seconds);
	  		       	ILI9341_DrawText(TimeBuffer, FONT4, 10, 30, BLACK, WHITE);
	  		 }
	  		F_SHOW_RTC = 0;F_SET_RTC = 0;

/******************************* Setting Current Time (Hours)***************************/
	  	   }else if (count == 1 && F_SET_RTC == 1){
	  		 button_pressed = 0;
	  		   while (button_pressed == 0)
	  		   {
	  			 updated_hours = counter / 5;
	  			 hours = updated_hours;

	  	   ILI9341_DrawText("SET RTC TIME     ", FONT4, 10, 10,BLACK, WHITE);

	       sprintf(TimeBuffer,"%02d : %02d : %02d        ",hours, minutes,seconds);

	  	   ILI9341_DrawText(TimeBuffer, FONT4, 10, 30, BLACK, WHITE);

	      ILI9341_DrawText("  ", FONT4, 10, 30, BLACK, WHITE); // setting for hours

	  		 F_SET_RTC = 0;F_SHOW_RTC = 0;
	  	   }

/******************************* Setting Current Time (Minutes)***************************/
	  	    if (count == 1 &&  F_SET_RTC_M == 1){
		  		 button_pressed = 0;
		  		   while (button_pressed == 0)
		  		   {
		  			 updated_mins = counter;
		  			 hours = updated_hours;
		  			 minutes = updated_mins;

		 	   ILI9341_DrawText("SET RTC TIME     ", FONT4, 10, 10,BLACK, WHITE);

		        sprintf(TimeBuffer,"%02d : %02d : %02d        ",hours, minutes,seconds);

		       ILI9341_DrawText(TimeBuffer, FONT4, 10, 30, BLACK, WHITE);

		  		//      ILI9341_DrawText("  ", FONT4, 10, 30, BLACK, WHITE); // setting for hours

	//	  		    updated_hours = hours1;
		  		     ILI9341_DrawText("  ", FONT4, 45, 30, BLACK, WHITE); // setting for minutes
		  		 //    ILI9341_DrawText("  ", FONT4, 92, 30, BLACK, WHITE); // setting for seconds
		  		F_SET_RTC_M = 0;
		  	   }
		  	   }
 /******************************* Setting Current Time (Seconds)***************************/

	  	  	    if (count == 1 && F_SET_RTC == 1 && F_SET_RTC_SEC == 1){
	  		  		 button_pressed = 0;
	  		  		   while (button_pressed == 0)
	  	   {
	  	 			 hours = updated_hours;
	  	 			updated_secs = counter;
	  	 			seconds = updated_secs;


	  		     ILI9341_DrawText("SET RTC TIME     ", FONT4, 10, 10,BLACK, WHITE);

	  	      sprintf(TimeBuffer,"%02d : %02d : %02d        ",hours, minutes,seconds);

	  	      ILI9341_DrawText(TimeBuffer, FONT4, 10, 30, BLACK, WHITE);
	  	    ILI9341_DrawText("  ", FONT4, 90, 30, BLACK, WHITE); // setting for seconds
	  		  F_SET_RTC = 0;	F_SET_RTC_SEC = 0;
	   	   }
	  	}
	  	   }
else {
/****************************************** Displaying MENU AND Highlighting*************************/

	ILI9341_DrawText("1.SHOW RTC TIME", FONT4, 10, 10,count <= 0 ? WHITE : BLACK, count <= 0 ? BLACK : WHITE);
	ILI9341_DrawText("2.SET RTC TIME", FONT4, 10, 30, count > 0 ? WHITE : BLACK, count > 0 ? BLACK : WHITE);
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  /* USER CODE END 3 */
}

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, reset_Pin|dc_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : reset_Pin dc_Pin */
  GPIO_InitStruct.Pin = reset_Pin|dc_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Encoder_SW_Pin */
  GPIO_InitStruct.Pin = Encoder_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Encoder_SW_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
