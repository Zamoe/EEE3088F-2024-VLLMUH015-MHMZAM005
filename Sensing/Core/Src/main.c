/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *Author: Zameer Mahomed
  *This code is to use IR sensor emitters and detectors to sense a wall.
  **************************************
  *Notes and mapping
  *Sensors use ADC pins PA0, PA1 and PA2
  *Emitters use PWM pins on PA8, PA9 and PA10
  *The timers used are tim1 on channels 1,2 and 3
  *
  *Commenting out LCD_switch() switches off the LCD
  *
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
#include "lcd_stm32f0.h"
#include "stdio.h"
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
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
//ADC
uint32_t adc_valueFront =0;
uint32_t adc_valueLeft = 0;
uint32_t adc_valueRight = 0;

uint32_t threshold = 500;
uint32_t brightness;
uint32_t bright_percent = 100;

uint32_t temp_F;
uint32_t temp_L;
uint32_t temp_R;

char LCD_buffer[];

//for debouncing
unsigned long debounceTicks = 0; //time since last press
unsigned long debounceDuration = 20;    // time for debouncing; so far best results with 200
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
void ADCPOLL_Front(void);
void ADCPOLL_Left(void);
void ADCPOLL_Right(void);
void LCD_switch(void);
void emitter_F(int);
void emitter_L(int);
void emitter_R(int);
void debounce_F(void);
void debounce_L(void);
void debounce_R(void);

void ADC1_COMP_IRQHandler(void);
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
  MX_TIM1_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  //Initialise PWM
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

  //LCD
  init_LCD();

  //PWM set brightness
  //set threshold
  //threshold = 0;
  //set emitter brightness from 0 to 47999
  brightness = (float)bright_percent/(float)100*(float)47999;
  //failsafe for if brightness gets set too bright
  if (brightness > 47999){
	  brightness = 47999;
  }
  /*//set PWM brightness
  	//pwm(brightness);
  	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, brightness);
  	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, brightness);
  	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, brightness);
*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	//lcd_command(CLEAR);
    /* USER CODE BEGIN 3 */

	//ADC code
	//change chsek5 to channel in use
	//ADCPOLL_Front();
	//ADCPOLL_Left();
	//ADCPOLL_Right();

	//HAL_GPIO_WritePin(GPIOB, LED7_Pin,SET);
	//HAL_GPIO_WritePin(GPIOB, LED6_Pin,SET);
	//HAL_GPIO_WritePin(GPIOB, LED5_Pin,SET);

	//code to control respective LEDS
	emitter_F(1);
	ADCPOLL_Front();
	if (adc_valueFront >= threshold){
		//debounce_F();
		HAL_GPIO_WritePin(GPIOB, LED6_Pin,SET);
		temp_F = temp_F -1;
	}
	else{
		temp_F = temp_F +1;
		if (temp_F >=3){
		HAL_GPIO_WritePin(GPIOB, LED6_Pin,RESET);
		temp_F=0;
		}
	}
	//HAL_Delay(200);
	//emitter_F(0);

	emitter_L(1);
	ADCPOLL_Left();
	if (adc_valueLeft >= threshold){
			//debounce_L();
			HAL_GPIO_WritePin(GPIOB, LED7_Pin,GPIO_PIN_SET);
			temp_L = temp_F -1;
		}
	else{
			temp_L = temp_L +1;
			if (temp_L >=3){
			HAL_GPIO_WritePin(GPIOB, LED7_Pin,RESET);
			temp_L=0;
			}
		}
	//HAL_Delay(200);
	//emitter_L(0);
	//temp =1;
	//while(temp == 1){
	emitter_R(1);
	ADCPOLL_Right();
	if (adc_valueRight >= threshold){
			//debounce_R();
			HAL_GPIO_WritePin(GPIOB, LED5_Pin,SET);
			temp_R = temp_F -1;
		}
	else{
			temp_R = temp_R +1;
			if (temp_R >=3){
			HAL_GPIO_WritePin(GPIOB, LED5_Pin,RESET);
			temp_R=0;
			}
		}
	//temp = 0;
	//}
	HAL_Delay(200);
	//emitter_R(0);


	//delay for no current
	//HAL_Delay(20);
	//HAL_GPIO_WritePin(LED6_GPIO_B, LED6_Pin,RESET);
	//HAL_GPIO_WritePin(LED5_GPIO_B, LED5_Pin,SET);

	//HAL_GPIO_TogglePin(GPIOB, LED7_Pin);

	//LCD printing comment out if polling too slow
	LCD_switch();

  }
  /* USER CODE END 3 */
}
//function to turn on emitter
void emitter_F(int input){
	if (input == 1 ){
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, brightness);
	}
	else {
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
	}
}
void emitter_L(int input){
	if (input == 1 ){
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, brightness);
	}
	else {
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
	}
}
void emitter_R(int input){
	if (input == 1 ){
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, brightness);
	}
	else {
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
	}
}
//function for LCD to remove if polling speed too low
void LCD_switch(void){
	lcd_command(CLEAR);
	sprintf(LCD_buffer, "F:%d", adc_valueFront); // Convert integer to string
	lcd_putstring(LCD_buffer);
	sprintf(LCD_buffer, " L:%d", adc_valueLeft); // Convert integer to string
	lcd_putstring(LCD_buffer);
	sprintf(LCD_buffer, " B:%d", bright_percent); // Convert integer to string
	lcd_putstring(LCD_buffer);
	lcd_command(LINE_TWO);
	sprintf(LCD_buffer, "R:%d", adc_valueRight); // Convert integer to string
	lcd_putstring(LCD_buffer);
	sprintf(LCD_buffer, " T:%d", threshold); // Convert integer to string
	lcd_putstring(LCD_buffer);
	HAL_Delay(250);
}
// function for adc
void ADCPOLL_Front(void){
	//ADC code
		//change chsek5 to channel in use
		ADC1 -> CHSELR = ADC_CHSELR_CHSEL0;
		HAL_ADC_Start(&hadc);
		HAL_Delay(10);
		while(HAL_ADC_PollForConversion(&hadc,HAL_MAX_DELAY)!= HAL_OK);
		HAL_ADC_Stop(&hadc);
		adc_valueFront = HAL_ADC_GetValue(&hadc);
		HAL_ADC_IRQHandler(&hadc); //Clear flags
}

void ADCPOLL_Left(void){
	//ADC code
		//change chsek5 to channel in use
		ADC1 -> CHSELR = ADC_CHSELR_CHSEL1;
		HAL_ADC_Start(&hadc);
		HAL_Delay(10);
		while(HAL_ADC_PollForConversion(&hadc,HAL_MAX_DELAY)!= HAL_OK);
		HAL_ADC_Stop(&hadc);
		adc_valueLeft = HAL_ADC_GetValue(&hadc);
		HAL_ADC_IRQHandler(&hadc); //Clear flags
}
void ADCPOLL_Right(void){
	//ADC code
		//change chsek5 to channel in use
		ADC1 -> CHSELR = ADC_CHSELR_CHSEL2;
		HAL_ADC_Start(&hadc);
		HAL_Delay(10);
		while(HAL_ADC_PollForConversion(&hadc,HAL_MAX_DELAY)!= HAL_OK);
		HAL_ADC_Stop(&hadc);
		adc_valueRight = HAL_ADC_GetValue(&hadc);
		HAL_ADC_IRQHandler(&hadc); //Clear flags
		//uint32_t val;  //poll ADC call ADC1_COMP_IRQHandler -> sets adc_val to adc val
		//HAL_ADC_Start(&hadc); //start ADC
		//HAL_ADC_PollForConversion(&hadc, 1); //poll at 1ms
		//adc_valueRight  = HAL_ADC_GetValue(&hadc); 	//get ADC value
		//return val;
}
void debounce_F(void)
{
	unsigned long currentTick = HAL_GetTick();
	// ensures that unwanted press noise within durationDuration are not regeistered
	if ((currentTick - debounceTicks) > debounceDuration){
		HAL_GPIO_WritePin(GPIOB, LED6_Pin,SET);

	}
	debounceTicks = currentTick; //set time since last tick

}

void debounce_L(void)
{
	unsigned long currentTick = HAL_GetTick();
	// ensures that unwanted press noise within durationDuration are not regeistered
	if ((currentTick - debounceTicks) > debounceDuration){
		HAL_GPIO_WritePin(GPIOB, LED7_Pin,SET);

	}
	debounceTicks = currentTick; //set time since last tick

}

void debounce_R(void)
{
	unsigned long currentTick = HAL_GetTick();
	// ensures that unwanted press noise within durationDuration are not regeistered
	if ((currentTick - debounceTicks) > debounceDuration){
		HAL_GPIO_WritePin(GPIOB, LED5_Pin,SET);

	}
	debounceTicks = currentTick; //set time since last tick

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
