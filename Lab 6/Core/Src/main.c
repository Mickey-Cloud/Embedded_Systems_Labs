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
volatile int redThreshold = 200;
volatile int blueThreshold = 150;
volatile int greenThreshold = 100;
volatile int orangeThreshold = 50;
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setupLights(void){
	//setup leds 6789
	GPIOC->MODER |= (1<<12);
	GPIOC->MODER &= ~(1<<13);
	GPIOC->MODER |= (1<<14);
	GPIOC->MODER &= ~(1<<15);
	GPIOC->MODER |= (1<<16);
	GPIOC->MODER &= ~(1<<17);
	GPIOC->MODER |= (1<<18);
	GPIOC->MODER &= ~(1<<19);

	GPIOC->OTYPER &= ~(1<<6);
	GPIOC->OTYPER &= ~(1<<7);
	GPIOC->OTYPER &= ~(1<<8);
	GPIOC->OTYPER &= ~(1<<9);
	
	GPIOC->OSPEEDR &= ~(1<<12);
	GPIOC->OSPEEDR &= ~(1<<14);
	GPIOC->OSPEEDR &= ~(1<<16);
	GPIOC->OSPEEDR &= ~(1<<18);

	GPIOC->PUPDR &= ~(1<<12);
	GPIOC->PUPDR &= ~(1<<13);
	GPIOC->PUPDR &= ~(1<<14);
	GPIOC->PUPDR &= ~(1<<15);
	GPIOC->PUPDR &= ~(1<<16);
	GPIOC->PUPDR &= ~(1<<17);
	GPIOC->PUPDR &= ~(1<<18);
	GPIOC->PUPDR &= ~(1<<19);
}
void lightsThresholds(int voltage){
	if(voltage > redThreshold){
		GPIOC->ODR |= (1<<6);
	}else{
		GPIOC->ODR &= ~(1<<6);
	}
	if (voltage > blueThreshold){
		GPIOC->ODR |= (1<<7);
	}else{
		GPIOC->ODR &= ~(1<<7);
	}
	if(voltage > greenThreshold){
		GPIOC->ODR |= (1<<9);
	}else{
		GPIOC->ODR &= ~(1<<9);
	}
	if(voltage > orangeThreshold){
		GPIOC->ODR |= (1<<8);
	}else{
		GPIOC->ODR &= ~(1<<8);
	}
	
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  
  HAL_Init();
  SystemClock_Config();
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	
	setupLights();

  //pin pc0
	GPIOC->MODER |= (1 << 0);
	GPIOC->MODER |= (1 << 1);
  GPIOC->PUPDR &= ~(1 << 0);
	GPIOC->PUPDR &= ~(1 << 1);
	
	//pin pA04
	GPIOA->MODER |= (1<<8);
	GPIOA->MODER |= (1<<9);
	GPIOA->PUPDR &= ~(1<<8);
	GPIOA->PUPDR &= ~(1<<9);
  
	
	ADC1->CFGR1 |= (1<<4);
	ADC1->CFGR1 &= ~(1<<3);
	//
	ADC1->CFGR1 |= (1<<13);
	//Hardware triggers disabled
	ADC1->CFGR1 &= ~(1<<11);
	ADC1->CFGR1 &= ~(1<<10);
	
	//set chanel 10
	ADC1->CHSELR |=(1<<10);
	
	//calibrate the ADC
	ADC1->CR |= (1<<31);
	while(ADC1->CR & 1<<31){
	}
	//enable
	ADC1->CR |= (1<<0);
	//start
	ADC1->CR |= (1<<2);
	
	//setup the DAC
	//software trigger mode
	DAC->CR |= (1<<5);
	DAC->CR |= (1<<4);
	DAC->CR |= (1<<3);
	//enable DAC
	DAC->CR |= (1<<0);
	
	const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
	
	
	
	int dcValue = 0;
	int index = 0;
  while (1)
  {
		if(index >= 31){
			index = 0;
		}else{
			index = index+1;
		}
		
		DAC->DHR8R1 = sine_table[index];
		
		dcValue = ADC1->DR;
		lightsThresholds(dcValue);
    
		HAL_Delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
