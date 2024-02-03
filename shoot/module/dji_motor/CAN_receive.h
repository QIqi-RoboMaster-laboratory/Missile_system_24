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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define SHOOT_CAN hcan1

/* CAN send and receive ID */
typedef enum
{
    CAN_SHOOT_ALL_ID = 0x1FF,
    CAN_PULL_SPRING_ID = 0x205,
    CAN_RELOAD_ID = 0x206,
		CAN_MISSILE_SHOOT_MOTOR_ID = 0x207,
		CAN_YAW_MOTOR_ID = 0X208,
} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct
{
	float POS_GAOL;//目标位置
	float POS_ABS;//绝对位置0
	float POS_OFFSET;
	float eer;
	float eer_eer;
}ANGLE_TypeDef;


extern void CAN_cmd_shoot(int16_t missile_shoot1,int16_t missile_shoot2 ,int16_t shoot, int16_t yaw);

extern const motor_measure_t *get_missile_shoot_motor_measure_point(void);

extern const motor_measure_t *get_shoot_motor_measure_point(uint8_t j);

extern const motor_measure_t *get_yaw_motor_measure_point(void);

#endif
