/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_rng.h"
#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                 									\
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
		
		
static motor_measure_t motor_shoot[2];
static motor_measure_t motor_missile;
static motor_measure_t motor_yaw;
		
static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              shoot_can_send_data[8];
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &hcan1)
    { 
			switch (rx_header.StdId)
			{
					case CAN_PULL_SPRING_ID:
					{
							get_motor_measure(&motor_shoot[rx_header.StdId - CAN_PULL_SPRING_ID], rx_data);
							detect_hook(CAN_PULL_SPRING_MOTOR_TOE );
							break;
					}
					case CAN_RELOAD_ID:
					{
							get_motor_measure(&motor_shoot[rx_header.StdId - CAN_PULL_SPRING_ID], rx_data);
							detect_hook(CAN_RELOAD_MOTOR_TOE );
							break;
					}
					case CAN_MISSILE_SHOOT_MOTOR_ID:
					{
							get_motor_measure(&motor_missile, rx_data);
							detect_hook(MISSILE_SHOOT_MOTOR_TOE);
							break;
					}
					case CAN_YAW_MOTOR_ID:
					{
							get_motor_measure(&motor_yaw, rx_data);
							detect_hook(CAN_YAW_MOTOR_TOE);
							break;
					}
					default:
					{
							break;
					}
			}
    }
		else if (hcan == &hcan2)
    {
        switch (rx_header.StdId)
    {
			default:
        {
            break;
        }
		}
		}		
}

void CAN_cmd_shoot(int16_t pull_spring,int16_t reload ,int16_t shoot, int16_t yaw)
{
   uint32_t send_mail_box;
   shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
   shoot_tx_message.IDE = CAN_ID_STD;
   shoot_tx_message.RTR = CAN_RTR_DATA;
   shoot_tx_message.DLC = 0x08;
   shoot_can_send_data[0] = (pull_spring >> 8);
   shoot_can_send_data[1] = pull_spring;
   shoot_can_send_data[2] = (reload >> 8);
   shoot_can_send_data[3] = reload;
   shoot_can_send_data[4] = (shoot >> 8);
   shoot_can_send_data[5] = shoot;
   shoot_can_send_data[6] = (yaw >> 8);
   shoot_can_send_data[7] = yaw;
   HAL_CAN_AddTxMessage(&hcan1, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}


const motor_measure_t *get_shoot_motor_measure_point(uint8_t j)
{
    return &motor_shoot[(j & 0x01)];
}

const motor_measure_t *get_missile_shoot_motor_measure_point(void)
{
    return &motor_missile;
}

const motor_measure_t *get_yaw_motor_measure_point(void)
{
    return &motor_yaw;
}
