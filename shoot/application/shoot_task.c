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
/*----------------------------------�궨��---------------------------*/
#define shoot_laser_on() laser_on()                                              // ���⿪���궨��
#define shoot_laser_off() laser_off()                                            // ����رպ궨��
#define PHOTOLECTRIC_DOOR_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin) // �����IO
#define missile_shoot_motor(speed) trigger_control.trigger_set_speed = speed           // �����������
/*----------------------------------�ڲ�����---------------------------*/
/**
 * @brief          ���ģʽ����
 * @param[in]      void
 * @retval         ������
 */
static void Shoot_Set_Mode(void);

/**
 * @brief          �������ݸ���
 * @param[in]      void
 * @retval         ���ؿ�
 */
static void Shoot_Feedback_Update(void);

/**
 * @brief        	 ���λ�ÿ���ѭ��
 * @param[in]      trigger_move_control_loop��Ҫ����λ�õĽṹ��
 * @retval         ���ؿ�
 */
static void missile_angle_control_loop(Shoot_Motor_t *trigger_move_control_loop);

/**
 * @brief        	 ����λ�ÿ���ѭ��
 * @param[in]      trigger_move_control_loop��Ҫ����λ�õĽṹ��
 * @retval         ���ؿ�
 */
static void missile_spring_angle_control_loop(Shoot_Motor_t *missile_move_control_loop);

/**
 * @brief        	 �����ٶȿ���ѭ��
 * @param[in]      trigger_move_control_loop��Ҫ����λ�õĽṹ��
 * @retval         ���ؿ�
 */
static void missile_reload_speed_control_loop(Shoot_Motor_t *trigger_move_control_loop);

/**
 * @brief        	 ����λ�ÿ���ѭ��
 * @param[in]      trigger_move_control_loop��Ҫ����λ�õĽṹ��
 * @retval         ���ؿ�
 */
static void missile_reload_angle_control_loop(Shoot_Motor_t *missile_move_control_loop);

/**
 * @brief        	 yawλ�ÿ���ѭ��
 * @param[in]      trigger_move_control_loop��Ҫ����λ�õĽṹ��
 * @retval         ���ؿ�
 */
static void missile_yaw_angle_control_loop(Shoot_Motor_t *missile_move_control_loop);

/**
 * @brief          ����λ�÷���ֵ
 * @param[in]      motor_angle_calc��Ҫ����λ�õĽṹ��
 * @retval         ���ؿ�
 */
static void Motor_Angle_Cal(Shoot_Motor_t *motor_angle_calc);

/**
 * @brief          �����ٶȷ���ֵ
 * @param[in]      motor_speed_clac��Ҫ�����ٶȵĽṹ��
 * @retval         ���ؿ�
 */
static void Motor_Speed_Cal(Shoot_Motor_t *motor_speed_clac);

/**
 * @brief          �����������ֵ
 * @param[in]      motor_current_calc��Ҫ��������Ľṹ��
 * @retval         ���ؿ�
 */
static void Motor_Current_Cal(Shoot_Motor_t *motor_current_calc);

/**
 * @brief          �жϵ����ת
 * @param[in]      motor_current_calc��Ҫ�ж϶�ת�Ľṹ��
 * @retval         ���ؿ�
 */
static void Motor_Block(Shoot_Motor_t*blocking_motor);

/**
 * @brief          ���÷������ģʽ
 * @param[in]      void
 * @retval         ���ؿ�
 */
static void shoot_set_control_mode(missile_shoot_move_t *missile_shoot_set_control);

/**
 * @brief          ���ģʽ�л����ݹ���
 * @param[in]      void
 * @retval         ���ؿ�
 */
static void shoot_mode_change_transit(void);

/**
 * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
 * @param[in]      void
 * @retval         ���ؿ�
 */
void shoot_init(void);

/**
 * @brief          �������
 * @param[in]      void
 * @retval         ���ؿ�
 */
void SERIO_Control(void);
/*----------------------------------�ڲ�����---------------------------*/
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
/*----------------------------------�ṹ��------------------------------*/
Shoot_Motor_t missile_shoot_motor; 
Shoot_Motor_t pull_spring_motor; 
Shoot_Motor_t reload_motor; 
Shoot_Motor_t yaw_motor;
missile_shoot_move_t missile_shoot_move;       // �������
/*----------------------------------�ⲿ����---------------------------*/
extern ExtY_stm32 stm32_Y_shoot;
extern ext_power_heat_data_t power_heat_data_t; // �����˵�ǰ�Ĺ���״̬����Ҫ�ж�ǹ������
/*---------------------------------------------------------------------*/
// ����ģʽ
shoot_mode_e shoot_mode = SHOOT_STOP;                             // �˴����ģʽ
shoot_mode_e last_shoot_mode = SHOOT_STOP;                        // �ϴ����ģʽ
shoot_control_mode_e shoot_control_mode = SHOOT_STOP_CONTROL;     // �������ģʽ
shoot_init_state_e shoot_init_state = SHOOT_INIT_UNFINISH;        // �����ʼ��ö����
shoot_motor_control_mode_e missile_shoot_motor_mode = SHOOT_MOTOR_STOP;    

/**
 * @brief          ������񣬼�� GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: ��
 * @retval         none
 */
void shoot_task(void const *pvParameters)
{
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    // �����ʼ��
    shoot_init();
    while (1)
    {
				SERIO_Control();
        // ���÷���ģʽ
        Shoot_Set_Mode();
        // ģʽ�л����ݹ���,��ҪPID�������ֹ���ݻ������������ת
        shoot_mode_change_transit();
        // �������ݸ���
        Shoot_Feedback_Update();
        // �������ѭ��
        shoot_control_loop();
        // ���Ϳ��Ƶ���
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
 * @brief          �������
 * @param[in]      void
 * @retval         ���ؿ�
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
 * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
 * @param[in]      void
 * @retval         ���ؿ�
 */
void shoot_init(void)
{
    missile_shoot_move.laster_add = 0;
    missile_shoot_motor.move_flag = 1;
    // ��ʼ��PID
    stm32_shoot_pid_init();
    // ������pid
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
    // ����ָ���ȡ
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
 * @brief          ���ģʽ�л����ݹ���
 * @param[in]      void
 * @retval         ���ؿ�
 */
static void shoot_mode_change_transit(void)
{
    if (last_shoot_mode != shoot_mode)
    {
        // ģʽ�����л�,pid���
        stm32_step_shoot_pid_clear();
    }
}

/**
 * @brief          �������ݸ���
 * @param[in]      void
 * @retval         ���ؿ�
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
 * @brief          �����ٶȷ���ֵ
 * @param[in]      motor_speed_clac��Ҫ�����ٶȵĽṹ��
 * @retval         ���ؿ�
 */
static void Motor_Speed_Cal(Shoot_Motor_t *motor_speed_clac)
{
	motor_speed_clac->speed = motor_speed_clac->shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED * 10;
}

/**
 * @brief          ����λ�÷���ֵ
 * @param[in]      motor_angle_calc��Ҫ����λ�õĽṹ��
 * @retval         ���ؿ�
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
 * @brief          �����������ֵ
 * @param[in]      motor_current_calc��Ҫ��������Ľṹ��
 * @retval         ���ؿ�
 */
static void Motor_Current_Cal(Shoot_Motor_t *motor_current_calc)
{
	motor_current_calc->current_cal = motor_current_calc->shoot_motor_measure->given_current;
}

/**
 * @brief          �жϵ����ת
 * @param[in]      motor_current_calc��Ҫ�ж϶�ת�Ľṹ��
 * @retval         ���ؿ�
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
 * @brief          ���÷������ģʽ
 * @param[in]      void
 * @retval         ���ؿ�
 */
static void shoot_set_control_mode(missile_shoot_move_t *missile_shoot_set_control)
{

    // ����ģʽ

    // �жϳ�ʼ���Ƿ����
    if (shoot_control_mode == SHOOT_INIT_CONTROL)
    {
        static uint32_t init_time = 0;
        // �жϲ����Ƿ񲦵��µ�
        if (switch_is_down(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
        {
            // �����µ�ֹͣ��ʼ��
            init_time = 0;
        }
        else
        {
            // �ж��Ƿ��ʼ�����
            if (shoot_init_state == SHOOT_INIT_UNFINISH)
            {
                // ��ʼ��δ���

                // �жϳ�ʼ��ʱ���Ƿ����
                if (init_time >= SHOOT_TASK_S_TO_MS(SHOOT_TASK_MAX_INIT_TIME))
                {
                    // ��ʼ��ʱ����������г�ʼ������������ģʽ
                    init_time = 0;
                }
                else
                {
                        // ��ʼ��ģʽ����ԭ״����ʼ��ʱ������
                        init_time++;
                        return;
								}
							}
            else
            {
                // ��������ģʽ
                init_time = 0;
            }
        }
    }
    // ����ң�����������÷������ģʽ
    if (switch_is_up(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // ����ǰ��վģʽ
        shoot_control_mode = SHOOT_OUTPOST;
    }
    else if (switch_is_mid(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // ң��������ģʽ
        shoot_control_mode = SHOOT_RC_CONTROL;
    }
    else if (switch_is_down(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // �������ģʽ
        shoot_control_mode = SHOOT_BASE;
    }
    else if (toe_is_error(DBUS_TOE))
    {
        // ң����������
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }
    else
    {
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }

    
    // �жϽ����ʼ��ģʽ
    static shoot_control_mode_e last_shoot_control_mode = SHOOT_STOP_CONTROL;
    if (shoot_control_mode != SHOOT_STOP_CONTROL && last_shoot_control_mode == SHOOT_STOP_CONTROL)
    {
        // �����ʼ��ģʽ
        //        shoot_control_mode = SHOOT_INIT_CONTROL;
    }
    last_shoot_control_mode = shoot_control_mode;
}

/**
 * @brief          ���ģʽ����
 * @param[in]      void
 * @retval         ������
 */
static void Shoot_Set_Mode(void)
{

    // ���÷������ģʽ
    shoot_set_control_mode(&missile_shoot_move);
		
    // �����ϴ����ģʽ
    last_shoot_mode = shoot_mode;

    // ���µ�ǰ���ģʽ
	
	
	
}
/**
 * @brief          ������ѭ��
 * @param[in]      void
 * @retval         ������
 */
void shoot_control_loop(void)
{
	a=PHOTOLECTRIC_DOOR_PIN;
	if(shoot_control_mode == SHOOT_OUTPOST)//����ǰ��վ
	{

	}
	
	if(shoot_control_mode == SHOOT_RC_CONTROL)//ң��������
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

    // pid����
    missile_angle_control_loop(&missile_shoot_motor); // �������
		missile_spring_angle_control_loop(&pull_spring_motor); // ���ɿ���
		missile_reload_angle_control_loop(&reload_motor); // �����̿���
		missile_yaw_angle_control_loop(&yaw_motor); // yaw�����
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
		missile_reload_angle_control_loop(&reload_motor); // �����̿���
	}
}

/**
 * @brief          ����λ�ÿ���ѭ��
 * @param[in]      trigger_move_control_loop��Ҫ����λ�õĽṹ��
 * @retval         ���ؿ�
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
 * @brief          ����λ�ÿ���ѭ��
 * @param[in]      trigger_move_control_loop��Ҫ����λ�õĽṹ��
 * @retval         ���ؿ�
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
 * @brief          �����ٶȿ���ѭ��
 * @param[in]      trigger_move_control_loop��Ҫ����λ�õĽṹ��
 * @retval         ���ؿ�
 */
static void missile_reload_speed_control_loop(Shoot_Motor_t *missile_move_control_loop)
{
    missile_move_control_loop->motor_pid.max_out = RELOAD_SPEED_PID_MAX_OUT;
    missile_move_control_loop->motor_pid.max_iout = RELOAD_SPEED_PID_MAX_IOUT;
    PID_Calc(&missile_move_control_loop->motor_pid, missile_move_control_loop->speed, missile_move_control_loop->set_speed);
    missile_move_control_loop->give_current = (int16_t)(missile_move_control_loop->motor_pid.out);
}

/**
 * @brief          ����λ�ÿ���ѭ��
 * @param[in]      trigger_move_control_loop��Ҫ����λ�õĽṹ��
 * @retval         ���ؿ�
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
 * @brief          yawλ�ÿ���ѭ��
 * @param[in]      trigger_move_control_loop��Ҫ����λ�õĽṹ��
 * @retval         ���ؿ�  
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



