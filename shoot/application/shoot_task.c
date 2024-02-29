/**
 * @file shoot_task.c
 * @date 2024-01-10
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "shoot_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_laser.h"
#include "bsp_missile_shoot.h"
#include "arm_math.h"
#include "bsp_servo_pwm.h"
#include "user_lib.h"
#include "referee.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "pid.h"
#include "stm32.h"
#include "tim.h"
#include "gpio.h"
/*----------------------------------宏定义---------------------------*/
#define shoot_laser_on() laser_on()                                              // 激光开启宏定义
#define shoot_laser_off() laser_off()                                            // 激光关闭宏定义
#define PHOTOLECTRIC_DOOR_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin) // 光电门IO
#define missile_shoot_motor(speed) trigger_control.trigger_set_speed = speed           // 开启拨弹电机
/*----------------------------------内部函数---------------------------*/
/**
 * @brief          射击模式设置
 * @param[in]      void
 * @retval         返回无
 */
static void Shoot_Set_Mode(void);

/**
 * @brief          发射数据更新
 * @param[in]      void
 * @retval         返回空
 */
static void Shoot_Feedback_Update(void);

/**
 * @brief        	 射击位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         返回空
 */
static void missile_angle_control_loop(Shoot_Motor_t *trigger_move_control_loop);

/**
 * @brief        	 弹簧位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         返回空
 */
static void missile_spring_angle_control_loop(Shoot_Motor_t *missile_move_control_loop);

/**
 * @brief        	 换弹速度控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         返回空
 */
static void missile_reload_speed_control_loop(Shoot_Motor_t *trigger_move_control_loop);

/**
 * @brief        	 换弹位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         返回空
 */
static void missile_reload_angle_control_loop(Shoot_Motor_t *missile_move_control_loop);

/**
 * @brief        	 yaw位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         返回空
 */
static void missile_yaw_angle_control_loop(Shoot_Motor_t *missile_move_control_loop);

/**
 * @brief          计算位置返回值
 * @param[in]      motor_angle_calc：要计算位置的结构体
 * @retval         返回空
 */
static void Motor_Angle_Cal(Shoot_Motor_t *motor_angle_calc);

/**
 * @brief          计算速度返回值
 * @param[in]      motor_speed_clac：要计算速度的结构体
 * @retval         返回空
 */
static void Motor_Speed_Cal(Shoot_Motor_t *motor_speed_clac);

/**
 * @brief          计算电流返回值
 * @param[in]      motor_current_calc：要计算电流的结构体
 * @retval         返回空
 */
static void Motor_Current_Cal(Shoot_Motor_t *motor_current_calc);

/**
 * @brief          判断电机堵转
 * @param[in]      motor_current_calc：要判断堵转的结构体
 * @retval         返回空
 */
static void Motor_Block(Shoot_Motor_t*blocking_motor);

/**
 * @brief          设置发射控制模式
 * @param[in]      void
 * @retval         返回空
 */
static void shoot_set_control_mode(missile_shoot_move_t *missile_shoot_set_control);

/**
 * @brief          射击模式切换数据过渡
 * @param[in]      void
 * @retval         返回空
 */
static void shoot_mode_change_transit(void);

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
void shoot_init(void);

/**
 * @brief          舵机控制
 * @param[in]      void
 * @retval         返回空
 */
void SERIO_Control(void);
/*----------------------------------内部变量---------------------------*/
fp32 missile_shoot;
int a;
fp32 motor_last_angle = 0;
fp32 sum = 0;
int b = 0;
int turnback_flag = 0;
int shoot_ready_flag = 0;
fp32 rc_speedset;
#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2000
int PWM=1000;
int turn_shoot_flag = 0;
int shoot_cnt = 1;
int turn_spring_flag = 0;
int spring_cnt = 1;
int turn_reload_flag = 0;
int reload_cnt = 1;
int turn_reload_cnt = 0;
int turn_yaw_flag = 0;
int yaw_cnt = 1;
uint8_t cnt = 1;
int B_flag=0;
/*----------------------------------结构体------------------------------*/
Shoot_Motor_t missile_shoot_motor; 
Shoot_Motor_t pull_spring_motor; 
Shoot_Motor_t reload_motor; 
Shoot_Motor_t yaw_motor;
missile_shoot_move_t missile_shoot_move;       // 发射控制
/*----------------------------------外部变量---------------------------*/
extern ExtY_stm32 stm32_Y_shoot;
extern ext_power_heat_data_t power_heat_data_t; // 机器人当前的功率状态，主要判断枪口热量
/*---------------------------------------------------------------------*/
// 控制模式
shoot_mode_e shoot_mode = SHOOT_STOP;                             // 此次射击模式
shoot_mode_e last_shoot_mode = SHOOT_STOP;                        // 上次射击模式
shoot_control_mode_e shoot_control_mode = SHOOT_STOP_CONTROL;     // 射击控制模式
shoot_init_state_e shoot_init_state = SHOOT_INIT_UNFINISH;        // 射击初始化枚举体
shoot_motor_control_mode_e missile_shoot_motor_mode = SHOOT_MOTOR_STOP;    

/**
 * @brief          射击任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void shoot_task(void const *pvParameters)
{
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    // 射击初始化
    shoot_init();
    while (1)
    {
				SERIO_Control();
        // 设置发射模式
        Shoot_Set_Mode();
        // 模式切换数据过渡,主要PID清除，防止数据积累引发电机反转
        shoot_mode_change_transit();
        // 发射数据更新
        Shoot_Feedback_Update();
        // 射击控制循环
        shoot_control_loop();
        // 发送控制电流
				if(toe_is_error(DBUS_TOE))
				{
						CAN_cmd_shoot(0,0,0,0);
				}
				else
				{
						CAN_cmd_shoot(pull_spring_motor.give_current, reload_motor.give_current, missile_shoot_motor.give_current, yaw_motor.give_current);
        }
					vTaskDelay(SHOOT_TASK_DELAY_TIME);
    }
}

/**
 * @brief          舵机控制
 * @param[in]      void
 * @retval         返回空
 */
void SERIO_Control(void)
{
		if(shoot_control_mode == SHOOT_RC_CONTROL)
		{
		if(missile_shoot_move.shoot_rc->rc.ch[3] >= 531)
		{
			PWM = missile_shoot_move.shoot_rc->rc.ch[3]*1.9;
		}
		else
		{
			PWM = 1000;
		}
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,PWM);
		if(PWM == 1000)
		{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
		}
		}
		if(shoot_control_mode == SHOOT_OUTPOST || shoot_control_mode == SHOOT_BASE)
		{
		if(shoot_ready_flag == 1)
		{
		PWM = 1130;
		}
		else if(shoot_ready_flag == 2)
		{
		PWM = 1320;
		}
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,PWM);
		}
}

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
void shoot_init(void)
{
    missile_shoot_move.laster_add = 0;
    missile_shoot_motor.move_flag = 1;
    // 初始化PID
    stm32_shoot_pid_init();
    // 拨弹盘pid
    static const fp32 missile_shoot_speed_pid[3] = {15, 0, 10};
		static const fp32 missile_shoot_angle_pid[3] = {10, 0, 10};
		static const fp32 pull_spring_speed_pid[3] = {15, 0, 2};
		static const fp32 pull_spring_angle_pid[3] = {10, 0, 0.5};
		static const fp32 reload_speed_pid[3] = {60, 0, 10};
		static const fp32 reload_angle_pid[3] = {60, 0, 10};
		static const fp32 yaw_speed_pid[3] = {30, 0, 2};
		static const fp32 yaw_angle_pid[3] = {20, 0, 1};
		PID_Init(&missile_shoot_motor.motor_pid_angle, PID_POSITION, missile_shoot_angle_pid, MISSILE_READY_ANGLE_PID_MAX_OUT, MISSILE_READY_ANGLE_PID_MAX_IOUT);
    PID_Init(&missile_shoot_motor.motor_pid, PID_POSITION, missile_shoot_speed_pid, MISSILE_READY_SPEED_PID_MAX_OUT, MISSILE_READY_SPEED_PID_MAX_IOUT);
		PID_Init(&pull_spring_motor.motor_pid_angle, PID_POSITION, pull_spring_angle_pid, SPRING_READY_ANGLE_PID_MAX_OUT, SPRING_READY_ANGLE_PID_MAX_IOUT);
		PID_Init(&pull_spring_motor.motor_pid, PID_POSITION, pull_spring_speed_pid, SPRING_READY_SPEED_PID_MAX_OUT, SPRING_READY_SPEED_PID_MAX_IOUT);
		PID_Init(&reload_motor.motor_pid_angle, PID_POSITION, reload_angle_pid, RELOAD_READY_ANGLE_PID_MAX_OUT, RELOAD_READY_ANGLE_PID_MAX_IOUT);
		PID_Init(&reload_motor.motor_pid, PID_POSITION, reload_speed_pid, RELOAD_READY_SPEED_PID_MAX_OUT, RELOAD_READY_SPEED_PID_MAX_IOUT);
		PID_Init(&yaw_motor.motor_pid_angle, PID_POSITION, yaw_angle_pid, MYAW_READY_ANGLE_PID_MAX_OUT, MYAW_READY_ANGLE_PID_MAX_IOUT);
		PID_Init(&yaw_motor.motor_pid, PID_POSITION, yaw_speed_pid, MYAW_READY_SPEED_PID_MAX_OUT, MYAW_READY_SPEED_PID_MAX_IOUT);
    // 数据指针获取
    missile_shoot_move.shoot_rc = get_remote_control_point();
    reload_motor.shoot_motor_measure = get_shoot_motor_measure_point(1);
    reload_motor.blocking_angle_set = 0;
		reload_motor.set_angle = 0;
    pull_spring_motor.shoot_motor_measure = get_shoot_motor_measure_point(0);
		pull_spring_motor.blocking_angle_set = 0;
		pull_spring_motor.set_angle = 0;
    missile_shoot_motor.shoot_motor_measure = get_missile_shoot_motor_measure_point();
		missile_shoot_motor.blocking_angle_set = 0;
		missile_shoot_motor.set_angle = 0;
		yaw_motor.shoot_motor_measure = get_yaw_motor_measure_point();
		yaw_motor.blocking_angle_set = 0;
		yaw_motor.set_angle = 0;	
		 
    Shoot_Feedback_Update();
}

/**
 * @brief          射击模式切换数据过渡
 * @param[in]      void
 * @retval         返回空
 */
static void shoot_mode_change_transit(void)
{
    if (last_shoot_mode != shoot_mode)
    {
        // 模式发生切换,pid清除
        stm32_step_shoot_pid_clear();
    }
}

/**
 * @brief          发射数据更新
 * @param[in]      void
 * @retval         返回空
 */
static void Shoot_Feedback_Update(void)
{    
		Motor_Speed_Cal(&missile_shoot_motor);
		Motor_Angle_Cal(&missile_shoot_motor);
		Motor_Current_Cal(&missile_shoot_motor);
		Motor_Block(&missile_shoot_motor);
	
		Motor_Speed_Cal(&reload_motor);
		Motor_Angle_Cal(&reload_motor);
		Motor_Current_Cal(&reload_motor);
		Motor_Block(&reload_motor);
		
		Motor_Speed_Cal(&pull_spring_motor);
		Motor_Angle_Cal(&pull_spring_motor);
		Motor_Current_Cal(&pull_spring_motor);
		Motor_Block(&pull_spring_motor);
		
		Motor_Speed_Cal(&yaw_motor);
		Motor_Angle_Cal(&yaw_motor);
		Motor_Current_Cal(&yaw_motor);
		Motor_Block(&yaw_motor);
	
}

/**
 * @brief          计算速度返回值
 * @param[in]      motor_speed_clac：要计算速度的结构体
 * @retval         返回空
 */
static void Motor_Speed_Cal(Shoot_Motor_t *motor_speed_clac)
{
	motor_speed_clac->speed = motor_speed_clac->shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED * 10;
}

/**
 * @brief          计算位置返回值
 * @param[in]      motor_angle_calc：要计算位置的结构体
 * @retval         返回空
 */
static void Motor_Angle_Cal(Shoot_Motor_t *motor_angle_calc)
{
	static float pos, pos_old;
	motor_angle_calc->ANGLE_rev.eer = motor_angle_calc->shoot_motor_measure->ecd -motor_angle_calc->shoot_motor_measure->last_ecd;
		if(motor_angle_calc->ANGLE_rev.eer < -4096)
		{
			motor_angle_calc->ANGLE_rev.eer += 8192;
		}
		else if(motor_angle_calc->ANGLE_rev.eer > 4096)
		{
			motor_angle_calc->ANGLE_rev.eer -= 8192;
		}
		motor_angle_calc->angle_sum += motor_angle_calc->ANGLE_rev.eer;
		motor_angle_calc->angle_ref = motor_angle_calc->angle_sum*360.0/19.0/8192.0/10;
		motor_angle_calc->reload_angle_ref = motor_angle_calc->angle_sum*360.0/8192.0/10;
}

/**
 * @brief          计算电流返回值
 * @param[in]      motor_current_calc：要计算电流的结构体
 * @retval         返回空
 */
static void Motor_Current_Cal(Shoot_Motor_t *motor_current_calc)
{
	motor_current_calc->current_cal = motor_current_calc->shoot_motor_measure->given_current;
}

/**
 * @brief          判断电机堵转
 * @param[in]      motor_current_calc：要判断堵转的结构体
 * @retval         返回空
 */
static void Motor_Block(Shoot_Motor_t*blocking_motor)
{
	if(abs(blocking_motor->give_current) == 10000)
	{
		blocking_motor->blocking_time++;
	}
	else
	{
		blocking_motor->blocking_time = 0;
	}
	if(blocking_motor->blocking_time >=700)
	{
		blocking_motor->block_flag = 1;
	}
	else
	{
		blocking_motor->block_flag = 0;
	}
}

/**
 * @brief          设置发射控制模式
 * @param[in]      void
 * @retval         返回空
 */
static void shoot_set_control_mode(missile_shoot_move_t *missile_shoot_set_control)
{

    // 运行模式

    // 判断初始化是否完成
    if (shoot_control_mode == SHOOT_INIT_CONTROL)
    {
        static uint32_t init_time = 0;
        // 判断拨杆是否拨到下档
        if (switch_is_down(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
        {
            // 拨到下档停止初始化
            init_time = 0;
        }
        else
        {
            // 判断是否初始化完成
            if (shoot_init_state == SHOOT_INIT_UNFINISH)
            {
                // 初始化未完成

                // 判断初始化时间是否过长
                if (init_time >= SHOOT_TASK_S_TO_MS(SHOOT_TASK_MAX_INIT_TIME))
                {
                    // 初始化时间过长不进行初始化，进入其他模式
                    init_time = 0;
                }
                else
                {
                        // 初始化模式保持原状，初始化时间增加
                        init_time++;
                        return;
								}
							}
            else
            {
                // 进入其他模式
                init_time = 0;
            }
        }
    }
    // 根据遥控器开关设置发射控制模式
    if (switch_is_up(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // 击打前哨站模式
        shoot_control_mode = SHOOT_OUTPOST;
    }
    else if (switch_is_mid(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // 遥控器控制模式
        shoot_control_mode = SHOOT_RC_CONTROL;
    }
    else if (switch_is_down(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // 击打基地模式
        shoot_control_mode = SHOOT_BASE;
    }
    else if (toe_is_error(DBUS_TOE))
    {
        // 遥控器报错处理
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }
    else
    {
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }

    
    // 判断进入初始化模式
    static shoot_control_mode_e last_shoot_control_mode = SHOOT_STOP_CONTROL;
    if (shoot_control_mode != SHOOT_STOP_CONTROL && last_shoot_control_mode == SHOOT_STOP_CONTROL)
    {
        // 进入初始化模式
        //        shoot_control_mode = SHOOT_INIT_CONTROL;
    }
    last_shoot_control_mode = shoot_control_mode;
}

/**
 * @brief          射击模式设置
 * @param[in]      void
 * @retval         返回无
 */
static void Shoot_Set_Mode(void)
{

    // 设置发射控制模式
    shoot_set_control_mode(&missile_shoot_move);
		
    // 保留上次射击模式
    last_shoot_mode = shoot_mode;

    // 更新当前射击模式
	
	
	
}
/**
 * @brief          拨弹轮循环
 * @param[in]      void
 * @retval         返回无
 */
void shoot_control_loop(void)
{
	a=PHOTOLECTRIC_DOOR_PIN;
	if(shoot_control_mode == SHOOT_OUTPOST)//击打前哨站
	{

	}
	
	if(shoot_control_mode == SHOOT_RC_CONTROL)//遥控器控制
	{		
		    if (abs(missile_shoot_move.shoot_rc->rc.ch[4]) >= 1000 && turn_shoot_flag == 0)
    {
					missile_shoot_motor.set_angle -= 0.5;
//					if(missile_shoot_motor.set_angle >= 200*shoot_cnt)
//					{
//					turn_shoot_flag = 1;
//					shoot_cnt += 1;
//						if(shoot_cnt == 0)
//						{
//							shoot_cnt += 1;
//						}
//					}
    }
		else if (abs(missile_shoot_move.shoot_rc->rc.ch[4]) == 660 && turn_shoot_flag == 0)
    {
					missile_shoot_motor.set_angle += 0.5;
//					if(missile_shoot_motor.set_angle <= 200*shoot_cnt)
//					{	
//					turn_shoot_flag = 1;
//					shoot_cnt -= 1;
//						if(shoot_cnt == 0)
//						{
//							shoot_cnt -= 1;
//						}
//					} 
		}

				if (missile_shoot_move.shoot_rc->rc.ch[2] ==660 && turn_spring_flag == 0)
    {
					pull_spring_motor.set_angle += 0.03;	
//					turn_spring_flag = 1;
    }
		else if (missile_shoot_move.shoot_rc->rc.ch[2] == -660 && turn_spring_flag == 0)
    {
					pull_spring_motor.set_angle -= 0.01;		
//					turn_spring_flag = 1;
    }

				if (missile_shoot_move.shoot_rc->rc.ch[1] > 400 && turn_reload_flag == 0)
    {
					reload_motor.set_angle += 60;
//			if(PHOTOLECTRIC_DOOR_PIN == 1)
//			{
//				reload_motor.set_angle += 1;
//			}
//					if(reload_motor.set_angle >= 40*reload_cnt)
//					{

					turn_reload_flag = 1;

//					reload_cnt += 1;
//						if(reload_cnt == 0)
//						{
//							reload_cnt += 1;
//						}
//					}

    }
		else if (missile_shoot_move.shoot_rc->rc.ch[1] < -400 && turn_reload_flag == 0)
    {
					reload_motor.set_angle -= 60;	
//						if(PHOTOLECTRIC_DOOR_PIN == 1)
//			{
//				reload_motor.set_angle -= 2;
//			}
//					if(reload_motor.set_angle <= 40*reload_cnt)
//					{	
					turn_reload_flag = 1;
//					reload_cnt -= 1;
//						if(reload_cnt == 0)
//						{
//							reload_cnt -= 1;
//						}
//					} 
    }
		
		if(reload_motor.block_flag == 1)
		{
				reload_motor.set_angle = reload_motor.reload_angle_ref;
		}
		
				if (missile_shoot_move.shoot_rc->rc.ch[0] > 400 && turn_yaw_flag == 0)
    {
					yaw_motor.set_angle += 0.01;

    }
		else if (missile_shoot_move.shoot_rc->rc.ch[0] < -400 && turn_yaw_flag == 0)
    {
					yaw_motor.set_angle -= 0.01;
	
    }
		
//				if (missile_shoot_move.shoot_rc->rc.ch[1] > 200 && turn_flag == 0)
//    {
//					reload_motor.set_angle += 15000;
//					turn_flag = 1;
//    }
//		else if (missile_shoot_move.shoot_rc->rc.ch[1] < -200 && turn_flag == 0)
//    {
//					reload_motor.set_angle -= 15000;	
//					turn_flag = 1;
//    }

		if (missile_shoot_move.shoot_rc->rc.ch[4] == 0)
		{		
			missile_shoot_motor.set_speed = 0;
			missile_shoot_motor.set_angle += 0;
			turn_shoot_flag = 0;
		}
		
		if (missile_shoot_move.shoot_rc->rc.ch[1] == 0)
		{		
			reload_motor.set_speed = 0;
			reload_motor.set_angle += 0;
			turn_reload_flag = 0;
		}

		if (missile_shoot_move.shoot_rc->rc.ch[2] == 0)
		{		
			pull_spring_motor.set_speed = 0;
			pull_spring_motor.set_angle += 0;
			turn_spring_flag = 0;
		}
		
		if (missile_shoot_move.shoot_rc->rc.ch[0] == 0)
		{		
			yaw_motor.set_speed = 0;
			yaw_motor.set_angle += 0;
			turn_yaw_flag = 0;
		}

    // pid计算
    missile_angle_control_loop(&missile_shoot_motor); // 发射控制
		missile_spring_angle_control_loop(&pull_spring_motor); // 弹簧控制
		missile_reload_angle_control_loop(&reload_motor); // 换弹盘控制
		missile_yaw_angle_control_loop(&yaw_motor); // yaw轴控制
	}
	if(shoot_control_mode == SHOOT_BASE)
	{
		if (missile_shoot_move.shoot_rc->rc.ch[1] > 400 && turn_reload_flag == 0)
    {
					reload_motor.set_angle += 1;
//			if(PHOTOLECTRIC_DOOR_PIN == 1)
//			{
//				reload_motor.set_angle += 1;
//			}
//					if(reload_motor.set_angle >= 40*reload_cnt)
//					{

					turn_reload_flag = 1;

//					reload_cnt += 1;
//						if(reload_cnt == 0)
//						{
//							reload_cnt += 1;
//						}
//					}

    }
		else if (missile_shoot_move.shoot_rc->rc.ch[1] < -400 && turn_reload_flag == 0)
    {
					reload_motor.set_angle -= 1;	
//						if(PHOTOLECTRIC_DOOR_PIN == 1)
//			{
//				reload_motor.set_angle -= 2;
//			}
//					if(reload_motor.set_angle <= 40*reload_cnt)
//					{	
					turn_reload_flag = 1;
//					reload_cnt -= 1;
//						if(reload_cnt == 0)
//						{
//							reload_cnt -= 1;
//						}
//					} 
    }
		if (missile_shoot_move.shoot_rc->rc.ch[1] == 0)
		{		
			reload_motor.set_speed = 0;
			reload_motor.set_angle += 0;
			turn_reload_flag = 0;
		}
		missile_reload_angle_control_loop(&reload_motor); // 换弹盘控制
	}
}

/**
 * @brief          发射位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         返回空
 */
static void missile_angle_control_loop(Shoot_Motor_t *missile_move_control_loop)
{
    missile_move_control_loop->motor_pid.max_out = MISSILE_SPEED_PID_MAX_OUT;
    missile_move_control_loop->motor_pid.max_iout = MISSILE_SPEED_PID_MAX_IOUT;	
		missile_move_control_loop->motor_pid_angle.max_out = MISSILE_ANGLE_PID_MAX_OUT;
    missile_move_control_loop->motor_pid_angle.max_iout = MISSILE_ANGLE_PID_MAX_IOUT;	
		missile_move_control_loop->given_angle = PID_Calc(&missile_move_control_loop->motor_pid_angle, missile_move_control_loop->angle_ref, missile_move_control_loop->set_angle);
		PID_Calc(&missile_move_control_loop->motor_pid, missile_move_control_loop->speed, missile_move_control_loop->given_angle);
    missile_move_control_loop->give_current = (missile_move_control_loop->motor_pid.out);
}

/**
 * @brief          弹簧位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         返回空
 */
static void missile_spring_angle_control_loop(Shoot_Motor_t *missile_move_control_loop)
{
    missile_move_control_loop->motor_pid.max_out = SPRING_SPEED_PID_MAX_OUT;
    missile_move_control_loop->motor_pid.max_iout = SPRING_SPEED_PID_MAX_IOUT;	
		missile_move_control_loop->motor_pid_angle.max_out = SPRING_ANGLE_PID_MAX_OUT;
    missile_move_control_loop->motor_pid_angle.max_iout = SPRING_ANGLE_PID_MAX_IOUT;	
		missile_move_control_loop->given_angle = PID_Calc(&missile_move_control_loop->motor_pid_angle, missile_move_control_loop->angle_ref, missile_move_control_loop->set_angle);
		PID_Calc(&missile_move_control_loop->motor_pid, missile_move_control_loop->speed, missile_move_control_loop->given_angle);
    missile_move_control_loop->give_current = (missile_move_control_loop->motor_pid.out);
}

/**
 * @brief          换弹速度控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         返回空
 */
static void missile_reload_speed_control_loop(Shoot_Motor_t *missile_move_control_loop)
{
    missile_move_control_loop->motor_pid.max_out = RELOAD_SPEED_PID_MAX_OUT;
    missile_move_control_loop->motor_pid.max_iout = RELOAD_SPEED_PID_MAX_IOUT;
    PID_Calc(&missile_move_control_loop->motor_pid, missile_move_control_loop->speed, missile_move_control_loop->set_speed);
    missile_move_control_loop->give_current = (int16_t)(missile_move_control_loop->motor_pid.out);
}

/**
 * @brief          换弹位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         返回空
 */
static void missile_reload_angle_control_loop(Shoot_Motor_t *missile_move_control_loop)
{
    missile_move_control_loop->motor_pid.max_out = RELOAD_SPEED_PID_MAX_OUT;
    missile_move_control_loop->motor_pid.max_iout = RELOAD_SPEED_PID_MAX_IOUT;	
		missile_move_control_loop->motor_pid_angle.max_out = RELOAD_ANGLE_PID_MAX_OUT;
    missile_move_control_loop->motor_pid_angle.max_iout = RELOAD_ANGLE_PID_MAX_IOUT;	
		missile_move_control_loop->given_angle = PID_Calc(&missile_move_control_loop->motor_pid_angle, missile_move_control_loop->reload_angle_ref, missile_move_control_loop->set_angle);
		PID_Calc(&missile_move_control_loop->motor_pid, missile_move_control_loop->speed, missile_move_control_loop->given_angle);
    missile_move_control_loop->give_current = (missile_move_control_loop->motor_pid.out);
}

/**
 * @brief          yaw位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         返回空  
 */
static void missile_yaw_angle_control_loop(Shoot_Motor_t *missile_move_control_loop)
{
    missile_move_control_loop->motor_pid.max_out = MYAW_SPEED_PID_MAX_OUT;
    missile_move_control_loop->motor_pid.max_iout = MYAW_SPEED_PID_MAX_IOUT;	
		missile_move_control_loop->motor_pid_angle.max_out = MYAW_ANGLE_PID_MAX_OUT;
    missile_move_control_loop->motor_pid_angle.max_iout = MYAW_ANGLE_PID_MAX_IOUT;	
		missile_move_control_loop->given_angle = PID_Calc(&missile_move_control_loop->motor_pid_angle, missile_move_control_loop->angle_ref, missile_move_control_loop->set_angle);
		PID_Calc(&missile_move_control_loop->motor_pid, missile_move_control_loop->speed, missile_move_control_loop->given_angle);
    missile_move_control_loop->give_current = (missile_move_control_loop->motor_pid.out);
}



