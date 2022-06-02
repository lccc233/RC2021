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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
#include "stdlib.h"
#include "math.h"
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
#define drawer_iout 300

#define M2006_S_KP 8.0f
#define M2006_S_KI 0.0f
#define M2006_S_KD 0.0f
#define M2006_S_OUT	4000.0f
#define M2006_S_IOUT 2000.0f

#define M2006_P_KP 0.5f
#define M2006_P_KI 0.0f
#define M2006_P_KD 1.0f
#define M2006_P_OUT	8000.0f
#define M2006_P_IOUT 6000.0f



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t indata[1];
uint8_t SBUS_in[25];
int num=0;
uint16_t ch[16];
void SBUS_intoCH(void){
ch[0]  = ((SBUS_in[1]|SBUS_in[2]<<8)                      & 0x07FF);
ch[1]  = ((SBUS_in[2]>>3 |SBUS_in[3]<<5)                 & 0x07FF);
ch[2]  = ((SBUS_in[3]>>6 |SBUS_in[4]<<2 |SBUS_in[5]<<10)  & 0x07FF);
ch[3]  = ((SBUS_in[5]>>1 |SBUS_in[6]<<7)                 & 0x07FF);
ch[4]  = ((SBUS_in[6]>>4 |SBUS_in[7]<<4)                 & 0x07FF);
ch[5]  = ((SBUS_in[7]>>7 |SBUS_in[8]<<1 |SBUS_in[9]<<9)   & 0x07FF);
ch[6]  = ((SBUS_in[9]>>2 |SBUS_in[10]<<6)                & 0x07FF);
ch[7]  = ((SBUS_in[10]>>5|SBUS_in[11]<<3)                & 0x07FF);
ch[8]  = ((SBUS_in[12]   |SBUS_in[13]<<8)                & 0x07FF);
ch[9]  = ((SBUS_in[13]>>3|SBUS_in[14]<<5)                & 0x07FF);
ch[10] = ((SBUS_in[14]>>6|SBUS_in[15]<<2|SBUS_in[16]<<10) & 0x07FF);
ch[11] = ((SBUS_in[16]>>1|SBUS_in[17]<<7)                & 0x07FF);
ch[12] = ((SBUS_in[17]>>4|SBUS_in[18]<<4)                & 0x07FF);
ch[13] = ((SBUS_in[18]>>7|SBUS_in[19]<<1|SBUS_in[20]<<9)  & 0x07FF);
ch[14] = ((SBUS_in[20]>>2|SBUS_in[21]<<6)                & 0x07FF);
ch[15] = ((SBUS_in[21]>>5|SBUS_in[22]<<3)                & 0x07FF);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern pid_type_def m2006_speed;
extern pid_type_def m2006_pos;
int16_t com1,com2;
uint16_t pwm_val_1=0;
uint16_t pwm_val_2=0;
uint16_t pwm_val_3=0;
uint16_t rabbit_1=0;
uint16_t rabbit_2=0;
void roll(void);
void grab(void);
void ring(void);
void send_to_hcc(void);
void win(void);
uint8_t QRCode[1];
uint8_t cam_info[5];
uint8_t mode[1];
int QRok=0;
uint8_t led_counter=0;
int get=0;
int cam_py=0;
int mode_f=0;

extern int16_t communicate[3];
uint8_t spi_tx_buff[3]={0xAA,0x11,0xAA};
uint8_t spi_rx_buff[3]={0};

int drawer_distance_set=0;
pid_type_def drawer_pid,cam_pid;
void controller()
{
	if(ch[1]<600)
		{
			rabbit_1=150;
			rabbit_2=380;
		}
		else if(ch[1]>1600)
		{
			rabbit_1=350;
			rabbit_2=230;
		}
		else
		{
			rabbit_1=(ch[1]-600)/5+150;
			rabbit_2=380-(ch[1]-600)*3/20;
		}
		if(ch[4]>1000)
		{
			grab();
			while(ch[4]>1000)
			{
				HAL_Delay(1);
			}
		}
		if(ch[5]>1000)
		{
			roll();
			while(ch[5]>1000)
			{
				HAL_Delay(1);
			}
		}
		if(ch[2]>=600&&ch[2]<=1600)
		{
			drawer_distance_set=(ch[2]-600)*320;
		}
		else if(ch[2]<600)
		{
			drawer_distance_set=0;
		}
		else if(ch[2]>1600)
		{
			drawer_distance_set=320000;
		}
}
void discode()
{
	for(int i=0;i<3;++i)
	{
		if((spi_rx_buff[i]&0xf0)==0xf0)
		{
		
		}
		else if((spi_rx_buff[i]&0xf0)==0x90)
		{
		
		}
		else if((spi_rx_buff[i]&0xf0)==0x60)
		{
		
		}
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_CAN1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
	{
		float pid[3]={M2006_P_KP, M2006_P_KI, M2006_P_KD};
		PID_init(&m2006_pos, PID_POSITION, pid, M2006_P_OUT, M2006_P_IOUT);
	}
	{
		float pid[3]={M2006_S_KP, M2006_S_KI, M2006_S_KD};
		PID_init(&m2006_speed, PID_POSITION, pid, M2006_S_OUT, M2006_S_IOUT);
	}	
	can_filter_init();//start 	
//	float pid_cam[3]={1,0,0};
//	PID_init(&drawer_pid,PID_POSITION,pid,drawer_out,drawer_iout);//pid init
//	PID_init(&cam_pid,PID_POSITION,pid_cam,300000,100000);//pid init
	
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//210-430
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);//100--200
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);//231->1 -- 460->2
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 450);
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 140);
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 175);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 200);
	HAL_UART_Receive_IT(&huart1, QRCode, sizeof(QRCode));
	HAL_UART_Receive_IT(&huart6, cam_info, sizeof(cam_info));
	HAL_UART_Receive_IT(&huart3,SBUS_in, 25);
	//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm_val);
	//HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	while(1){
//		CAN_CMD_Communicate(123, 123, 123);
//		HAL_Delay(10);
//	}
//	while(1){
//	
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm_val_1);
//	HAL_Delay(10);
//	}



  while (1)
  {	
//		HAL_Delay(100);
		if(communicate[0]==1)mode[0]=2;
		else if(communicate[0]==2)mode[0]=1;
		else if(communicate[0]==0)mode[0]=0;
		else if(communicate[0]==3)win();
		else if(communicate[0]==4){
			mode[0]=1;
			mode_f=1;
		}
		if(mode[0]==0){drawer_distance_set=0;__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 370);__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 420);
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 425);}
		HAL_Delay(10);
		//send_to_hcc();
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm_val_1);
		
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm_val_2);
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm_val_3);
//		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, rabbit_1);
//		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, rabbit_2);
		if(get==1)roll();
		get=0;
		if(ch[0])controller();
		else{
			if(mode[0]!=cam_info[1])
				HAL_UART_Transmit(&huart6,mode,sizeof(mode),1000);
			else 
			{
				if(mode[0]==1)
				{
					if(mode_f==0){
						__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 231);
						__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 420);
						__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 380);
						drawer_distance_set=300000;
						if(cam_info[2]==1){
							if(cam_info[3]<50){
								CAN_CMD_Communicate(1, 0, 0);
								int ax=0;
//								while(cam_info[4]-120>20||cam_info[4]-120<-20){
//									drawer_distance_set+=(cam_info[4]-120)*100;
//									HAL_Delay(10);
//									ax++;
//									if(ax>100)break;
//								}
								__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 190);
								HAL_Delay(2000);
								while(abs(drawer_distance_set-motor_measure_chassis[0].code)>1000){
									HAL_Delay(10);
								}
								drawer_distance_set=1000;
								HAL_Delay(800);
								__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 380);
								HAL_Delay(200);
								drawer_distance_set=300000;
								CAN_CMD_Communicate(0, 0, 0);
							}
						}
					}
					else{
						drawer_distance_set=220000;
						__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 230);
						__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 320);
						__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 325);
						if(cam_info[2]==1||cam_info[2]==2)
						{
							if(cam_info[3]<50){
								if(cam_info[2]==1){
									CAN_CMD_Communicate(0, 0, 0);
								}
								if(cam_info[2]==2){
									CAN_CMD_Communicate(0, 1, 0);
								}
								roll();
							}
						}
					}
				}
				else if(mode[0]==2)
				{
					drawer_distance_set=270000;
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 231);
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 420);
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 425);
					if(cam_info[2]==1)
					{
						if(cam_info[3]<50){
							CAN_CMD_Communicate(1, 0, 0);
							int ax=0;
							while(!communicate[1]){
								ax++;
								HAL_Delay(1);if(ax>2000)break;
							}
							ax=0;
//							while(cam_info[4]-120>20||cam_info[4]-120<-20){
//								drawer_distance_set+=(cam_info[4]-120)*100;
//								HAL_Delay(10);
//								if(drawer_distance_set>320000)drawer_distance_set=320000;
//								ax++;
//								if(ax>100)break;
//							}
							grab();
							CAN_CMD_Communicate(0, 0, 0);
						}
						//HAL_Delay(200);
					}
					else if(cam_info[2]==2)
					{
						if(cam_info[3]<50)
						ring();
					}
//					else if(cam_info[2]==3)
//					{
//						if(QRok==1)grab();
//						else{   
//							CAN_CMD_Communicate(1, 0, 0);
//							int ax=0;
//							while(!communicate[1]){
//								ax++;
//								HAL_Delay(1);if(ax>2000)break;
//							}
//							ax=1000;
//							while(ax--){
//								HAL_Delay(1);
//								if(QRok==1)grab();
//								if(cam_info[2]==1)grab();
//								break;
//							}
//							CAN_CMD_Communicate(0, 0, 0);
//						}
//						QRok=0;
//						//HAL_Delay(1000);
//					}
				}
			}
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
		switch(led_counter%2)
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
				//led_counter = 0;
		}
		led_counter++;
		//drawer_distance_set=PID_calc(&cam_pid,cam_py,0);
	}
}
void roll()
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 255);
	HAL_Delay(120);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 325);
	HAL_Delay(800);
}

void grab()
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 330);
	HAL_Delay(300);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 100);
	HAL_Delay(100);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 230);
	HAL_Delay(400);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 231);
	HAL_Delay(600);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 190);
	HAL_Delay(300);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 460);
	HAL_Delay(500);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 140);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 425);
	HAL_Delay(100);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 200);
	HAL_Delay(500);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 231);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 420);
	HAL_Delay(300);
}
int r_f=0;
void ring()
{
	r_f=1;
	CAN_CMD_Communicate(1, 0, 0);
	int ax=0;
	while(!communicate[1]){
			ax++;
			HAL_Delay(1);if(ax>2000)break;
	}
	HAL_Delay(500);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 330);
	HAL_Delay(300);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 100);
	HAL_Delay(100);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 230);
	HAL_Delay(400);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 231);
	HAL_Delay(600);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 190);
	HAL_Delay(300);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 300);
	CAN_CMD_Communicate(1, 2, 0);
	while(communicate[1]!=2){
			ax++;
			HAL_Delay(1);if(ax>2000)break;
	}
	HAL_Delay(100);
	drawer_distance_set=0;
	HAL_Delay(500);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 220);
	HAL_Delay(800);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 425);
	HAL_Delay(300);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 330);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 420);
	drawer_distance_set=300000;
	
	HAL_Delay(500);
	CAN_CMD_Communicate(0, 0, 0);
}
void win()
{
	drawer_distance_set=0;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 231);
	CAN_CMD_Communicate(1, 0, 0);
	int ax=0;
	while(communicate[1]!=2){
		ax++;
		HAL_Delay(1);if(ax>2000)break;
	}
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 190);
	HAL_Delay(100);
	CAN_CMD_Communicate(2, 0, 0);
	HAL_Delay(500);
	drawer_distance_set=240000;
	HAL_Delay(500);
	CAN_CMD_Communicate(3, 0, 0);
	HAL_Delay(1300);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 400);
	HAL_Delay(200);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 300);
	CAN_CMD_Communicate(0, 0, 0);
	HAL_Delay(1000);
	communicate[0]=0;
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
		cam_py=cam_info[4]-30;
	}
	if(huart==&huart3)
	{
		SBUS_intoCH();
		HAL_UART_Receive_IT(&huart3,SBUS_in, 25);
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

