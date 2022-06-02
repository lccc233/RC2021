/*
 * @Author: AFShk
 * @Date: 2022-04-17 09:48:27
 * @LastEditors: AFShk
 * @LastEditTime: 2022-04-22 14:41:08
 * @FilePath: \example\BSP\bsp_can.c
 * @Description: 
 * 
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved. 
 */
#include "can.h"
#include "bsp_can.h"
#include "pid.h"
extern CAN_HandleTypeDef hcan1;
extern pid_type_def M2006_SPEED;
int16_t motor[4];
extern int drawer_distance_set;

pid_type_def m2006_speed;
pid_type_def m2006_pos;

CAN_TxHeaderTypeDef  chassis_tx_message;
uint8_t              chassis_can_send_data[8];
CAN_TxHeaderTypeDef  m2006_tx_message;
uint8_t              m2006_can_send_data[8];
CAN_TxHeaderTypeDef  gimbal_tx_message;
uint8_t              gimbal_can_send_data[8];
CAN_TxHeaderTypeDef  communicate_tx_message;
uint8_t              communicate_can_send_data[8];

uint8_t safe_flag=1;
uint8_t rx_data2[8];
uint8_t rx_data1[8];
int16_t communicate[3];
motor_measure_t motor_measure_chassis[4];
motor_measure_t motor_measure_m2006[4];
motor_measure_t motor_measure_gimbal[3];
motor_measure_t motor_measure_typedfcommunicate[3];

void can_filter_init(void)
{
	//3508 chassis fifo0 bank 0-3;
	uint32_t CAN_ID=0x204;
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = ((CAN_ID<<21)&0xffff0000)>>16;
	can_filter_st.FilterIdLow = ((CAN_ID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0xFFFF;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

//	CAN_ID=0x203;//0x202
//	can_filter_st.FilterIdHigh = ((CAN_ID<<21)&0xffff0000)>>16;
//	can_filter_st.FilterIdLow = ((CAN_ID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
//	can_filter_st.FilterBank = 1;
//	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
//	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

//	CAN_ID=0x202;//0x203
//	can_filter_st.FilterIdHigh = ((CAN_ID<<21)&0xffff0000)>>16;
//	can_filter_st.FilterIdLow = ((CAN_ID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
//	can_filter_st.FilterBank = 2;
//	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
//	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

//	CAN_ID=0x201;//0x204
//	can_filter_st.FilterIdHigh = ((CAN_ID<<21)&0xffff0000)>>16;
//	can_filter_st.FilterIdLow = ((CAN_ID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
//	can_filter_st.FilterBank = 3;
//	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
//	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

//    //other gimbal fifo0 bank 4-10;
//	CAN_ID=0x205;//0x205
//	can_filter_st.FilterIdHigh = ((CAN_ID<<21)&0xffff0000)>>16;
//	can_filter_st.FilterIdLow = ((CAN_ID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
//	can_filter_st.FilterBank = 4;
//	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
//	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

//	CAN_ID++;//0x206
//	can_filter_st.FilterIdHigh = ((CAN_ID<<21)&0xffff0000)>>16;
//	can_filter_st.FilterIdLow = ((CAN_ID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
//	can_filter_st.FilterBank = 5;
//	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
//	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

//	CAN_ID++;//0x207
//	can_filter_st.FilterIdHigh = ((CAN_ID<<21)&0xffff0000)>>16;
//	can_filter_st.FilterIdLow = ((CAN_ID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
//	can_filter_st.FilterBank = 6;
//	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
//	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

//	CAN_ID++;//0x208
//	can_filter_st.FilterIdHigh = ((CAN_ID<<21)&0xffff0000)>>16;
//	can_filter_st.FilterIdLow = ((CAN_ID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
//	can_filter_st.FilterBank = 7;
//	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
//	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

//	CAN_ID++;//0x209
//	can_filter_st.FilterIdHigh = ((CAN_ID<<21)&0xffff0000)>>16;
//	can_filter_st.FilterIdLow = ((CAN_ID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
//	can_filter_st.FilterBank = 8;
//	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
//	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

//	CAN_ID++;//0x20A
//	can_filter_st.FilterIdHigh = ((CAN_ID<<21)&0xffff0000)>>16;
//	can_filter_st.FilterIdLow = ((CAN_ID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
//	can_filter_st.FilterBank = 9;
//	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
//	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

//	CAN_ID++;//0x20B
//	can_filter_st.FilterIdHigh = ((CAN_ID<<21)&0xffff0000)>>16;
//	can_filter_st.FilterIdLow = ((CAN_ID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
//	can_filter_st.FilterBank = 10;
//	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
//	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

//	CAN_ID++;//0x20C
//	can_filter_st.FilterIdHigh = ((CAN_ID<<21)&0xffff0000)>>16;
//	can_filter_st.FilterIdLow = ((CAN_ID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
//	can_filter_st.FilterBank = 11;
//	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
//	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

//	CAN_ID++;//0x20D
//	can_filter_st.FilterIdHigh = ((CAN_ID<<21)&0xffff0000)>>16;
//	can_filter_st.FilterIdLow = ((CAN_ID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
//	can_filter_st.FilterBank = 12;
//	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
//	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

//	CAN_ID++;//0x20E
//	can_filter_st.FilterIdHigh = ((CAN_ID<<21)&0xffff0000)>>16;
//	can_filter_st.FilterIdLow = ((CAN_ID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
//	can_filter_st.FilterBank = 13;
//	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
//	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

//	can_filter_st.FilterActivation = ENABLE;
//	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
//	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
//	can_filter_st.FilterIdHigh = 0x0000;
//	can_filter_st.FilterIdLow = 0x0000;
//	can_filter_st.FilterMaskIdHigh = 0x0000;
//	can_filter_st.FilterMaskIdLow = 0x0000;
//	can_filter_st.FilterBank = 14;
//	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
//	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
//	HAL_CAN_Start(&hcan2);
//	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan==&hcan1)
	{
		CAN_RxHeaderTypeDef rx_header;
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data1);
		switch (rx_header.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			{
				static uint8_t i = 0;
				i = rx_header.StdId - CAN_3508_M1_ID;
				get_motor_measure(&motor_measure_chassis[i], rx_data1);
				//电机编码器控制
			    //记录编码器数据
				int16_t temp1=motor_measure_chassis[i].ecd-motor_measure_chassis[i].last_ecd;
				int16_t temp2=temp1+(temp1<0?8192:-8192);
				if(i==0||i==2)
					motor_measure_chassis[i].code+=abs(temp2)<abs(temp1)?temp2:temp1;
				else if(i==1||i==3)
					motor_measure_chassis[i].code-=abs(temp2)<abs(temp1)?temp2:temp1;
				if(i==0){
					if(safe_flag==1){
						PID_calc(&m2006_pos, motor_measure_chassis[0].code, drawer_distance_set);
						PID_calc(&m2006_speed, motor_measure_chassis[0].speed_rpm, m2006_pos.out);
						CAN_CMD_Chassis(m2006_speed.out, 0, 0, 0);
					}
					else if(safe_flag==2)
						CAN_CMD_Chassis(500, 0, 0, 0);
					else{
						motor_measure_chassis[0].code=0;
						CAN_CMD_Chassis(0, 0, 0, 0);
					}
				}	
				break;
		  }
			case CAN_2006_M1_ID:
			case CAN_2006_M2_ID:
			case CAN_2006_M3_ID:
			case CAN_2006_M4_ID:
			{
				static uint8_t i = 0;
				i = rx_header.StdId - CAN_2006_M1_ID;
				get_motor_measure(&motor_measure_m2006[i], rx_data1);
				
				int16_t temp1=motor_measure_m2006[i].ecd-motor_measure_m2006[i].last_ecd;
				int16_t temp2=temp1+(temp1<0?8192:-8192);
				motor_measure_m2006[i].code+=abs(temp2)<abs(temp1)?temp2:temp1;			
				break;
	
			}
			case CAN_COMMUNICATE_M1_ID:
			case CAN_COMMUNICATE_M3_ID:
			{
				communicate[0]=(rx_data1[0]<<8)|rx_data1[1];
				communicate[1]=(rx_data1[2]<<8)|rx_data1[3];
				communicate[2]=(rx_data1[4]<<8)|rx_data1[5];
			}
			default:
			{
				break;
			}
		}
  }
//	else if(hcan==&hcan2){
//		CAN_RxHeaderTypeDef rx_header;
//		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data1);	
//		switch (rx_header.StdId)
//		{
//			default:
//			{
//				break;
//			}
//		}
//	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan==&hcan1){
		CAN_RxHeaderTypeDef rx_header;
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data2);	
		switch (rx_header.StdId)
		{
//			case CAN_2006_M1_ID:
//			case CAN_2006_M2_ID:
//			case CAN_2006_M3_ID:
//			case CAN_2006_M4_ID:
//			{
//				static uint8_t i = 0;
//				i = rx_header.StdId - CAN_2006_M1_ID;
//				get_motor_measure(&motor_measure_m2006[i], rx_data2);
//				
//				int16_t temp1=motor_measure_m2006[i].ecd-motor_measure_m2006[i].last_ecd;
//				int16_t temp2=temp1+(temp1<0?8192:-8192);
//				motor_measure_m2006[i].code+=abs(temp2)<abs(temp1)?temp2:temp1;
//				break;
//			}
//			case CAN_6020_M1_ID:
//			case CAN_6020_M2_ID:
//			case CAN_6020_M3_ID:
//			{
//				static uint8_t i = 0;
//				i = rx_header.StdId - CAN_6020_M1_ID;
//				get_motor_measure(&motor_measure_gimbal[i], rx_data2);
//				
//				int16_t temp1=motor_measure_gimbal[i].ecd-motor_measure_gimbal[i].last_ecd;
//				int16_t temp2=temp1+(temp1<0?8192:-8192);
//				motor_measure_gimbal[i].code+=abs(temp2)<abs(temp1)?temp2:temp1;
//				break;
//			}
//		  case CAN_COMMUNICATE_M1_ID:
//			case CAN_COMMUNICATE_M2_ID:
//			case CAN_COMMUNICATE_M3_ID:
//			{
//				static uint8_t i = 0;
//				i = rx_header.StdId - CAN_COMMUNICATE_M1_ID;
//				get_motor_measure(&motor_measure_typedfcommunicate[i], rx_data2);
//				break;
//			}
			default:
			{
				break;
			}
		}
	}
//	else if(hcan==&hcan2){
//		CAN_RxHeaderTypeDef rx_header;
//		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data2);	
//		switch (rx_header.StdId)
//		{
//			default:
//			{
//				break;
//			}
//		}
//	}
}

void CAN_CMD_Chassis(int16_t M1 ,int16_t M2 ,int16_t M3 ,int16_t M4)
{
	uint32_t send_mail_box;
	chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = M1 >> 8;
	chassis_can_send_data[1] = M1;
	chassis_can_send_data[2] = M2 >> 8;
	chassis_can_send_data[3] = M2;
	chassis_can_send_data[4] = M3 >> 8;
	chassis_can_send_data[5] = M3;
	chassis_can_send_data[6] = M4 >> 8;
	chassis_can_send_data[7] = M4;
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_CMD_M2006(int16_t M1 ,int16_t M2 ,int16_t M3 ,int16_t M4)
{
	uint32_t send_mail_box;
	m2006_tx_message.StdId = CAN_M2006_ALL_ID;
	m2006_tx_message.IDE = CAN_ID_STD;
	m2006_tx_message.RTR = CAN_RTR_DATA;
	m2006_tx_message.DLC = 0x08;
	m2006_can_send_data[0] = M1 >> 8;
	m2006_can_send_data[1] = M1;
	m2006_can_send_data[2] = M2 >> 8;
	m2006_can_send_data[3] = M2;
	m2006_can_send_data[4] = M3 >> 8;
	m2006_can_send_data[5] = M3;
	m2006_can_send_data[6] = M4 >> 8;
	m2006_can_send_data[7] = M4;
	HAL_CAN_AddTxMessage(&M2006_CAN, &m2006_tx_message, m2006_can_send_data, &send_mail_box);
}

void CAN_CMD_Gimbal(int16_t M1 ,int16_t M2 ,int16_t M3)
{
	uint32_t send_mail_box;
	gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x06;
	gimbal_can_send_data[0] = M1 >> 8;
	gimbal_can_send_data[1] = M1;
	gimbal_can_send_data[2] = M2 >> 8;
	gimbal_can_send_data[3] = M2;
	gimbal_can_send_data[4] = M3 >> 8;
	gimbal_can_send_data[5] = M3;
	HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void CAN_CMD_Communicate(int16_t M1 ,int16_t M2 ,int16_t M3)
{
	uint32_t send_mail_box;
	communicate_tx_message.StdId = CAN_COMMUNICATE_M3_ID;
	communicate_tx_message.IDE = CAN_ID_STD;
	communicate_tx_message.RTR = CAN_RTR_DATA;
	communicate_tx_message.DLC = 0x06;
	communicate_can_send_data[0] = M1 >> 8;
	communicate_can_send_data[1] = M1;
	communicate_can_send_data[2] = M2 >> 8;
	communicate_can_send_data[3] = M2;
	communicate_can_send_data[4] = M3 >> 8;
	communicate_can_send_data[5] = M3;
	HAL_CAN_AddTxMessage(&COMMUNICATE_CAN, &communicate_tx_message, communicate_can_send_data, &send_mail_box);
}

