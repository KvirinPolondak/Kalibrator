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
#include "i2c.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VALID_COMMAND_LENGTH 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t command_buffer[50] = {'0', '0', '0', '0', '0', '0', '0', '0', '0', '0'};
volatile int command_buffer_pointer = 0;
volatile int command_received = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void STATUS_LED_On(void);
void STATUS_LED_Off(void);
void STATUS_LED_Toggle(void);
int Command_Callback(uint8_t *cmd_in, uint32_t length);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart1, command_buffer, 1);

  /* Signal to show we're online */
	STATUS_LED_On();
	HAL_Delay(100);
	STATUS_LED_Off();
	HAL_Delay(100);
	STATUS_LED_On();
	HAL_Delay(100);
	STATUS_LED_Off();
	HAL_Delay(100);

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  	/* Check if command was received */
  	if (command_received) {
  		Command_Callback(command_buffer, command_buffer_pointer);

  		/* reset the flag */
  		command_received = 0;

  		/* reset the buffer */
  		command_buffer_pointer = 0;
  		memset(command_buffer, '0', VALID_COMMAND_LENGTH);
  	}

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Turn on Status LED */
void STATUS_LED_On(void) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	return;
}


/* Turn off Status LED */
void STATUS_LED_Off(void) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	return;
}


/* Toggle Status LED */
void STATUS_LED_Toggle(void) {
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	return;
}


/* Status LED blink Success */
void STATUS_LED_Blink_Success(void) {
	STATUS_LED_On();
	HAL_Delay(100);
	STATUS_LED_Off();
	HAL_Delay(100);
	STATUS_LED_On();
	HAL_Delay(100);
	STATUS_LED_Off();
	HAL_Delay(100);
}


/* Status LED blink Failure */
void STATUS_LED_Blink_Failure(void) {
	STATUS_LED_On();
	HAL_Delay(100);
	STATUS_LED_Off();
	HAL_Delay(100);
	STATUS_LED_On();
	HAL_Delay(100);
	STATUS_LED_Off();
	HAL_Delay(100);
	STATUS_LED_On();
	HAL_Delay(100);
	STATUS_LED_Off();
	HAL_Delay(100);
	STATUS_LED_On();
	HAL_Delay(100);
	STATUS_LED_Off();
	HAL_Delay(100);
}


/* This Callback function is called when data from USB is received */
void CDC_ReceiveCallback_FS(uint8_t* Buf, uint32_t Len) {
	/* Set the flag only if we recieved valid number of chars */
	if (Len == VALID_COMMAND_LENGTH) {
		memcpy(command_buffer, Buf, Len);
		command_buffer_pointer = Len;
		command_received = 1;
	}
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		command_buffer_pointer++;

		/* If the beginning of the command buffer is valid */
		if (command_buffer[0] == 'L') {
			/* Don't overrun buffer */
			if (command_buffer_pointer < VALID_COMMAND_LENGTH) {

				HAL_UART_Receive_IT(huart, command_buffer + command_buffer_pointer, 1);

			/* If the buffer is full */
			} else {
				/* set the flag */
				command_received = 1;

				HAL_UART_Receive_IT(huart, command_buffer, 1);
			}
		/* if the beginning of the command buffer is invalid */
		} else {
			/* clear the buffer */
			command_buffer_pointer = 0;
			HAL_UART_Receive_IT(huart, command_buffer, 1);
		}
	}

	return;
}


int Command_extract_info(uint8_t *string, uint32_t length, uint32_t *led_id, uint32_t *led_val) {
		if (length < 10) {
			return -1; /* Failure: command too short */
		}

		/* Check if command has defined structure */
		if (string[0] != 'L' 		  ||
				string[1] != 'E' 			||
				string[2] != 'D' 			||
				string[3] != '_' 		 	||
				! isdigit(string[4]) 	||
				! isdigit(string[5]) 	||
				string[6] != '_' 		 	||
				! isdigit(string[7]) 	||
				! isdigit(string[8]) 	||
				! isdigit(string[9])
			) {

			return -1;	/* Failure: wrong command */
		}

		/* String to led id (int) */
		*led_id = (string[4] - '0') * 10 + (string[5] - '0');

		/* String to led current (int)*/
		*led_val = 	100 * (string[7] - '0')  +
								10  * (string[8] - '0')  +
								1   * (string[9] - '0');

		/* Set upper bound on LED current value */
		if (*led_val > 255) {
			*led_val = 255;
		}

		/* Set upper bound on LEDs */
		if (*led_id >= 20) {

			return -1; /* Failure: not so many LEDs available */
		}


		/* return Success */
		return 0;
}



int Command_Callback(uint8_t *cmd_in, uint32_t length) {
	uint32_t led_id;
	uint32_t led_val;
	uint8_t dac_data[3];

	/* Command format = LED_XX_ZZZ */

	if (Command_extract_info(cmd_in, length, &led_id, &led_val) != 0) {
		/* Failure: can't extract led_id and led_val from input command */
		STATUS_LED_Blink_Failure();
		return -1;
	}

	/* DAC data setup */
	dac_data[0] = 0b00010000;				/* control byte */
	dac_data[1] = led_val;					/* msb byte */
	dac_data[2] = 0b00000000;				/* lsb byte (dont care byte) */

	/* Based on LED Id send command to certain DAC (one of four available) */
	switch (led_id) {
	case 0:
		dac_data[0] = 0b00010000;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001100<<1, dac_data, 3, 100);
		break;
	case 1:
		dac_data[0] = 0b00010010;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001100<<1, dac_data, 3, 100);
		break;
	case 2:
		dac_data[0] = 0b00010100;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001100<<1, dac_data, 3, 100);
		break;
	case 3:
		dac_data[0] = 0b00010110;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001100<<1, dac_data, 3, 100);
		break;
	case 4:
		dac_data[0] = 0b00010000;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001101<<1, dac_data, 3, 100);
		break;
	case 5:
		dac_data[0] = 0b00010010;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001101<<1, dac_data, 3, 100);
		break;
	case 6:
		dac_data[0] = 0b00010100;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001101<<1, dac_data, 3, 100);
		break;
	case 7:
		dac_data[0] = 0b00010110;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001101<<1, dac_data, 3, 100);
		break;
	case 8:
		dac_data[0] = 0b00010000;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001110<<1, dac_data, 3, 100);
		break;
	case 9:
		dac_data[0] = 0b00010010;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001110<<1, dac_data, 3, 100);
		break;
	case 10:
		dac_data[0] = 0b00010100;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001110<<1, dac_data, 3, 100);
		break;
	case 11:
		dac_data[0] = 0b00010110;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001110<<1, dac_data, 3, 100);
		break;
	case 12:
		dac_data[0] = 0b00010000;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001111<<1, dac_data, 3, 100);
		break;
	case 13:
		dac_data[0] = 0b00010010;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001111<<1, dac_data, 3, 100);
		break;
	case 14:
		dac_data[0] = 0b00010100;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001111<<1, dac_data, 3, 100);
		break;
	case 15:
		dac_data[0] = 0b00010110;
		HAL_I2C_Master_Transmit(&hi2c1, 0b01001111<<1, dac_data, 3, 100);
		break;
	case 16:
		dac_data[0] = 0b00010000;
		HAL_I2C_Master_Transmit(&hi2c2, 0b01001100<<1, dac_data, 3, 100);
		break;
	case 17:
		dac_data[0] = 0b00010010;
		HAL_I2C_Master_Transmit(&hi2c2, 0b01001100<<1, dac_data, 3, 100);
		break;
	case 18:
		dac_data[0] = 0b00010100;
		HAL_I2C_Master_Transmit(&hi2c2, 0b01001100<<1, dac_data, 3, 100);
		break;
	case 19:
		dac_data[0] = 0b00010110;
		HAL_I2C_Master_Transmit(&hi2c2, 0b01001100<<1, dac_data, 3, 100);
		break;
	}

	/* Return Success */
	STATUS_LED_Blink_Success();
	return 0;
}




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
