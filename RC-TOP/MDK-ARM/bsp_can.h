/*
 * @Author: AFShk
 * @Date: 2022-04-17 09:48:40
 * @LastEditors: AFShk
 * @LastEditTime: 2022-04-17 14:19:52
 * @FilePath: \example\BSP\bsp_can.h
 * @Description: 
 * 
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved. 
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"
#include "math.h"
#include "stdlib.h"

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	  int code;
} motor_measure_t;

extern motor_measure_t motor_measure_chassis[4];
extern motor_measure_t motor_measure_m2006[4];
extern motor_measure_t motor_measure_gimbal[3];
extern motor_measure_t motor_measure_typedfcommunicate[3];

#define CAN_CHASSIS_ALL_ID 0x200
#define CAN_3508_M1_ID 0x201
#define CAN_3508_M2_ID 0x202
#define CAN_3508_M3_ID 0x203
#define CAN_3508_M4_ID 0x204
#define CHASSIS_CAN hcan1

#define CAN_M2006_ALL_ID 0x1FF
#define CAN_2006_M1_ID 0x205
#define CAN_2006_M2_ID 0x206
#define CAN_2006_M3_ID 0x207
#define CAN_2006_M4_ID 0x208
#define M2006_CAN hcan1

#define CAN_GIMBAL_ALL_ID 0x2FF
#define CAN_6020_M1_ID 0x209
#define CAN_6020_M2_ID 0x20A
#define CAN_6020_M3_ID 0x20B
#define GIMBAL_CAN hcan1

#define COMMUNICATE_ALL_ID 0x2FF
#define CAN_COMMUNICATE_M1_ID 0x20C
#define CAN_COMMUNICATE_M2_ID 0x20D
#define CAN_COMMUNICATE_M3_ID 0x20E
#define COMMUNICATE_CAN hcan1

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

extern void can_filter_init(void);
extern motor_measure_t motor_measure_chassis[4];
extern motor_measure_t motor_measure_gimbal[3];
extern void CAN_CMD_Chassis(int16_t M1 ,int16_t M2 ,int16_t M3 ,int16_t M4);
extern void CAN_CMD_M2006(int16_t M1 ,int16_t M2 ,int16_t M3 ,int16_t M4);		
extern void CAN_CMD_Gimbal(int16_t M1 ,int16_t M2 ,int16_t M3);
extern void CAN_CMD_Communicate(int16_t M1 ,int16_t M2 ,int16_t M3);
#endif
