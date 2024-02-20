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

volatile char savedChar;
volatile char newData;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
void USART3_4_IRQHandler(void){
	savedChar = USART3->RDR;
	newData = 1;
}


void transmitCharacter(char c){
	while(1){
		if((USART3->ISR & (1<<7)) == (1<<7)){
			break;
		}
	}
    USART3->TDR = c;
}

void transmitString(const char* word){
	int i = 0;
	while (word[i] != '\0') { 
        transmitCharacter(word[i]); // Transmit the current character
        i++; // Move to the next character in the array
    }
}

void cmdLogic(char c){
	switch(c){
		case 'r':
		case 'R':
			switch(savedChar){
				case '0':
					GPIOC->ODR &= ~(1<<6);
				  transmitString("Red Light turned Off.\n\r");
					break;
				case '1':
					GPIOC->ODR |= (1<<6);
				  transmitString("Red Light turned on.\n\r");
					break;
				case '2':
					GPIOC->ODR ^= (1<<6);
				  transmitString("Red Light toggled.\n\r");
					break;
				default:
					transmitString("Error - command not number 0,1, or 2.\n\r");
			}
			break;
		case 'g':
		case 'G':
			switch(savedChar){
				case '0':
					GPIOC->ODR &= ~(1<<9);
				  transmitString("Green Light turned Off.\n\r");
					break;
				case '1':
					GPIOC->ODR |= (1<<9);
				  transmitString("Green Light turned on.\n\r");
					break;
				case '2':
					GPIOC->ODR ^= (1<<9);
				  transmitString("Green Light toggled.\n\r");
					break;
				default:
					transmitString("Error - command not number 0,1, or 2. \n\r");
			}
			break;
		case 'b':
		case 'B':
			switch(savedChar){
				case '0':
					GPIOC->ODR &= ~(1<<7);
				  transmitString("Blue Light turned Off.\n\r");
					break;
				case '1':
					GPIOC->ODR |= (1<<7);
				  transmitString("Blue Light turned on.\n\r");
					break;
				case '2':
					GPIOC->ODR ^= (1<<7);
				  transmitString("Blue Light toggled.\n\r");
					break;
				default:
					transmitString("Error - command not number 0, 1, or 2.\n\r");
			}
			break;
		case 'o':
		case 'O':
			switch(savedChar){
				case '0':
					GPIOC->ODR &= ~(1<<8);
				  transmitString("Orange Light turned Off.\n\r");
					break;
				case '1':
					GPIOC->ODR |= (1<<8);
				  transmitString("Orange Light turned on.\n\r");
					break;
				case '2':
					GPIOC->ODR ^= (1<<8);
				  transmitString("Orange Light toggled.\n\r");
					break;
				default:
					transmitString("Error - command not number 0, 1, or 2.\n\r");
			}
			break;
		default:
			transmitString("Something went really bad we should never reach here!!\n\r");
	}
}

void lightLogic(void){
	char color;
	switch(savedChar){
		case 'r':
		case 'R':
		case 'g':
		case 'G':
		case 'b':
		case 'B':
		case 'o':
		case 'O':
			transmitString("CMD?\n\r");
			color = savedChar;
			//wait until the RDR is done transmitting
			while(1){
			if(newData == 1){
				newData = 0;
				break;
				}
			}
			cmdLogic(color);
			break;
		default:
			transmitString("Error - Input not color (r,g,b,o)!\n\r");
			break;
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

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	//setup all the LED's
	//setup leds 67
	GPIOC->MODER |= (1<<12);
	GPIOC->MODER &= ~(1<<13);
	GPIOC->MODER |= (1<<14);
	GPIOC->MODER &= ~(1<<15);

	GPIOC->OTYPER &= ~(1<<6);
	GPIOC->OTYPER &= ~(1<<7);
	
	GPIOC->OSPEEDR &= ~(1<<12);
	GPIOC->OSPEEDR &= ~(1<<14);

	GPIOC->PUPDR &= ~(1<<12);
	GPIOC->PUPDR &= ~(1<<13);
	GPIOC->PUPDR &= ~(1<<14);
	GPIOC->PUPDR &= ~(1<<15);
	//setup the LED's	8 9
	GPIOC->MODER |= (1<<16);
	GPIOC->MODER &= ~(1<<17);
	GPIOC->MODER |= (1<<18);
	GPIOC->MODER &= ~(1<<19);

	GPIOC->OTYPER &= ~(1<<8);
	GPIOC->OTYPER &= ~(1<<9);
	
	GPIOC->OSPEEDR &= ~(1<<16);
	GPIOC->OSPEEDR &= ~(1<<18);

	GPIOC->PUPDR &= ~(1<<16);
	GPIOC->PUPDR &= ~(1<<17);
	GPIOC->PUPDR &= ~(1<<18);
	GPIOC->PUPDR &= ~(1<<19);
	
	//set pb10 & 11 to alternate function mode
	GPIOB->MODER &= ~(1<<20);
	GPIOB->MODER |= (1<<21);
	GPIOB->MODER &= ~(1<<22);
	GPIOB->MODER |=(1<<23);
	
	GPIOB->AFR[1] |= 1<<10;
	GPIOB->AFR[1] |= 1<<14;
	
	USART3->BRR = HAL_RCC_GetHCLKFreq()/9600;
	USART3->CR1 |= (1<<2);
	USART3->CR1 |= (1<<3);
	USART3->CR1 |= (1<<5);
	USART3->CR1 |= (1<<0);
	
	newData=0;
	
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 2);
	
  while (1)
  {
		
		while(1){
			if(newData){
				newData = 0;
				break;
			}
		}
		lightLogic();
		/*
		if(USART3->ISR & (1<<5) ){
			
			GPIOC->ODR ^= (1<<8);
		}
		while(~(USART3->ISR & (1<<5))){
			GPIOC->ODR ^= (1<<8);
		}*/
    
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
