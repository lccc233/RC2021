#include "callback.h"
#include "can.h"
#include "pid.h"

motor_measure_t drawer;
int drawer_delta,drawer_change;
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];


extern int drawer_distance_set;
extern pid_type_def drawer_pid;
uint8_t cnt=0;

//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

		
void can_init()
{
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//can enable
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
	
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	  if(rx_header.StdId==0x201)
		{
			get_motor_measure(&drawer,rx_data);
			{
				if((drawer.last_ecd>7000)&&(drawer.ecd<2000))
						drawer_delta=(8192-drawer.last_ecd)+(drawer.ecd);
				else if((drawer.ecd>7000)&&(drawer.last_ecd<2000))
						drawer_delta=-((8192-drawer.ecd)+drawer.last_ecd);
				else
						drawer_delta=drawer.ecd-drawer.last_ecd;
  			drawer_change+=drawer_delta;
		   	PID_calc(&drawer_pid,drawer_change,drawer_distance_set);
			  can_cmd_chassis(drawer_pid.out);	
			}
			
		}
}

void can_cmd_chassis(int16_t motor1)
{
	  uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = 0>> 8;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0>> 8;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0>> 8;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}















