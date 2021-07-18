/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal.h"

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

char transmit_data[25] = "";
char command_vel[20] = "";
uint8_t rx_char = 0x00;
int rx_index = 0;
int ready_flag = 0;

unsigned char Re_buf[30], counter = 0;
unsigned char RX[2];
uint8_t Rig[3];
float YAW;
int16_t DATA[7];

float KP = 0.01;
float KI = .002;
float KD = 0;

float max_sum = 1;
float min_sum = 0;

float max_controller = 1;
float min_controller = 0;

float max_d_error = 1;
float min_d_error = 0;

float MR_setpoint = 50;
float ML_setpoint = 50;

float MR_setpoint_cm = 10;
float ML_setpoint_cm = 10;	

int MR_direction = 1;
int ML_direction = 1;

float MR_rpm;
float ML_rpm;

float MR_speed;
float ML_speed;

uint16_t MR_count = 0;
uint16_t ML_count = 0;

uint16_t MR_last_count = 0;
uint16_t ML_last_count = 0;

uint8_t MR_flag_encoder = 0;
uint8_t ML_flag_encoder = 0;

float MR_error = 0;
float ML_error = 0;

float MR_last_error = 0;
float ML_last_error = 0;

float MR_sum_error = 0;
float ML_sum_error = 0;


float MR_d_error;
float ML_d_error;

float MR_controller;
float ML_controller;

int serial_timer_counter = 0;

int a= 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_setpoint();
void start_IMU();
void start_lcd();

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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT( & htim8);
  HAL_TIM_Base_Start_IT( & htim1);
  HAL_TIM_Base_Start_IT( & htim3);

  HAL_TIM_PWM_Start( & htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start( & htim4, TIM_CHANNEL_2);

  // __HAL_UART_ENABLE_IT( & huart2, UART_IT_RXNE);
  //__HAL_UART_ENABLE_IT( & huart6, UART_IT_RXNE);
  //HAL_UART_Receive_IT( & huart6, & rx_char, 1);
  //HAL_UART_Receive_IT( & huart2, RX, 1);
  start_lcd();
	clear();
	HAL_Delay(3000);
	print("    Az Mehr");
	setCursor(0,1);
	print(" Aghaz Mikonim");
  start_IMU();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (MR_direction == -1) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    }
    if (ML_direction == -1) {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
    }
		
		if(serial_timer_counter >= 5){
			sprintf(transmit_data, "%.1f %.1f %.1f\n", MR_speed, ML_speed, YAW);
			MR_speed = -MR_rpm * MR_direction *.3516;
			ML_speed = -ML_rpm * ML_direction * .3516;
			HAL_UART_Transmit( & huart6, transmit_data, strlen(transmit_data), strlen(transmit_data));
			serial_timer_counter = 0;
			
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
  if (htim == & htim8) {
		serial_timer_counter++;
    MR_count = TIM1 -> CNT;
    ML_count = TIM3 -> CNT;
    
		MR_rpm = MR_count - MR_last_count + MR_flag_encoder * MR_direction* 65535;
    ML_rpm = ML_count - ML_last_count + ML_flag_encoder * ML_direction* 65535;
		
		MR_last_count = MR_count;
    ML_last_count = ML_count;
		
		MR_flag_encoder = 0;
    ML_flag_encoder = 0;
		
				
    if (MR_rpm > 0) {
      MR_rpm = -MR_rpm;
    }
		
    if (ML_rpm > 0) {
      ML_rpm = -ML_rpm;
    }
		
    MR_rpm = (MR_rpm * 300) / 4000.0;
    ML_rpm = (ML_rpm * 300) / 4000.0;

		
		//P
		
    MR_error = MR_setpoint + MR_rpm;
		ML_error = ML_setpoint + ML_rpm;
		
		//I
    MR_sum_error = MR_sum_error + KI * MR_error;
		ML_sum_error = ML_sum_error + KI * ML_error;
		
    if (MR_sum_error > max_sum) {
      MR_sum_error = max_sum;
    }
    if (MR_sum_error < min_sum) {
      MR_sum_error = min_sum;
    }
		
		if (ML_sum_error > max_sum) {
      ML_sum_error = max_sum;
    }
    if (ML_sum_error < min_sum) {
      ML_sum_error = min_sum;
    }
		
    MR_d_error = KD * (MR_error - MR_last_error);
		ML_d_error = KD * (ML_error - ML_last_error);
		
    MR_last_error = MR_error;
    if (MR_d_error > max_d_error) {
      MR_d_error = max_d_error;
    }
    if (MR_d_error < -max_d_error) {
      MR_d_error = -max_d_error;
    }
		
		ML_last_error = ML_error;
    if (ML_d_error > max_d_error) {
      ML_d_error = max_d_error;
    }
    if (ML_d_error < -max_d_error) {
      ML_d_error = -max_d_error;
    }
		
    MR_controller = KP * MR_error + MR_sum_error + MR_d_error;
		ML_controller = KP * ML_error + ML_sum_error + ML_d_error;
		
    if (MR_controller > max_controller) {
      MR_controller = max_controller;
    }
    if (MR_controller < min_controller) {
      MR_controller = min_controller;
    }
		
		if (ML_controller > max_controller) {
      ML_controller = max_controller;
    }
    if (ML_controller < min_controller) {
      ML_controller = min_controller;
    }
		
    __HAL_TIM_SET_COMPARE( & htim4, TIM_CHANNEL_2, MR_controller * 7200);
    __HAL_TIM_SET_COMPARE( & htim4, TIM_CHANNEL_4, ML_controller * 7200);

  }
	
  if (htim == & htim1) {
    MR_flag_encoder = 1;
  }
	
  if (htim == & htim3) {
    ML_flag_encoder = 1;
		a ++;
  }

}

void HAL_USART_RxCpltCallback(UART_HandleTypeDef * huart) {

  if (huart == & huart6) {
    command_vel[rx_index] = rx_char;
    rx_index++;
    if (rx_index == 8) {
      set_setpoint();
      rx_index = 0;
    }
    HAL_UART_Receive_IT( & huart6, & rx_char, 1);
  }

  if (huart == & huart2) {
    HAL_UART_Receive_IT( & huart2, RX, 1);
    Re_buf[counter] = RX[0];
    if (counter == 0 && Re_buf[0] != 0x5A)
      return;
    counter++;
    if (counter == 20) {
      counter = 0;
      DATA[0] = (Re_buf[4] << 8) | Re_buf[5];
      YAW = (((float) DATA[0] / 100)*360)/320.0;
      HAL_UART_Transmit( & huart2, Rig, 3, 3);
    }

  }
}

void set_setpoint() {
  MR_setpoint_cm = (command_vel[1] - '0') * 10 + (command_vel[2] - '0') + (command_vel[3] - '0') * .1;
  ML_setpoint_cm = (command_vel[5] - '0') * 10 + (command_vel[6] - '0') + (command_vel[7] - '0') * .1;
	MR_setpoint	= MR_setpoint_cm / .3516;
	ML_setpoint  = ML_setpoint_cm / .3516;
  
	if (command_vel[0] == '-') {
    MR_direction = -1;
  } else {
    MR_direction = 1;
  }
  if (command_vel[4] == '-') {
    ML_direction = -1;
  } else {
    ML_direction = 1;
  }
}
void start_IMU() {
  Rig[0] = 170;
  Rig[1] = 56;
  Rig[2] = 226;
  HAL_Delay(2000);
  HAL_UART_Transmit( & huart2, Rig, 3, 3);
}

void start_lcd() {
  LiquidCrystal(GPIOE, GPIO_PIN_0, GPIO_PIN_6, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5);
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
