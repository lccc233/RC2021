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
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "callback.h"
#include "pid.h"
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
#define drawer_kp 0.02
#define drawer_ki 0.1
#define drawer_kd 3
#define drawer_out 3000 
#define drawer_iout 800
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t pwm_val_1=0;
uint16_t pwm_val_2=0;
void roll(void);
void grab(void);
void ring(void);
uint8_t QRCode[1];
uint8_t cam_info[3];
uint8_t mode[1];
int QRok=0;
uint8_t led_counter=0;
int get=0;
int light;
uint8_t spi_buff[6]={0xaa,0xbb,0xcc,0xdd,0xee,0xff};

int drawer_distance_set=0;
pid_type_def drawer_pid;
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
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	float pid[3]={drawer_kp,drawer_ki,drawer_kp};
	can_init();//start 
	PID_init(&drawer_pid,PID_POSITION,pid,drawer_out,drawer_iout);//pid init
	
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);//264--500
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 450);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 140);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 175);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, light);
	HAL_UART_Receive_IT(&huart1, QRCode, sizeof(QRCode));
	HAL_UART_Receive_IT(&huart6, cam_info, sizeof(cam_info));
	//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm_val);
	//HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_SPI_Transmit_DMA(&hspi2,spi_buff,4);
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, light);//20
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm_val_1);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm_val_2);
		if(get==1)grab();
		get=0;
		if(mode[0]!=cam_info[1])
			HAL_UART_Transmit(&huart6,mode,sizeof(mode),1000);
		else 
		{
			if(mode[0]==1)
			{
				if(cam_info[2]==1||cam_info[2]==2)
				{
					roll();
				}
			}
			else if(mode[0]==2)
			{
				if(cam_info[2]==1)
				{
					grab();
				}
				else if(cam_info[2]==2)
				{
					ring();
				}
				else if(cam_info[2]==3)
				{
					if(QRok==1)grab();
					HAL_Delay(1000);
				}
			}
		}
		HAL_Delay(100);
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == (&htim2))
	{
		switch(led_counter)
		{
			case 0:
				HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
				break;
			case 1:
				HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
				break;
			default:
				HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_RESET);
				led_counter = 0;
		}
		led_counter++;
	}
}
void roll()
{
	long i;
	for(i=450000;i>=290000;--i)__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, i/1000);
	HAL_Delay(150);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 450);
}

void grab()
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 230);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 208);
	HAL_Delay(300);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 175);
	HAL_Delay(300);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,106);
	HAL_Delay(300);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 440);
	HAL_Delay(2000);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 140);
	HAL_Delay(100);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 250);
}

void ring()
{
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart1)
	{
		if(QRCode[0]=='R')
		{
			QRok=1;
		}
		HAL_UART_Receive_IT(&huart1, QRCode, sizeof(QRCode));
	}
	if(huart==&huart6)
	{
		HAL_UART_Receive_IT(&huart6, cam_info, sizeof(cam_info));
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
