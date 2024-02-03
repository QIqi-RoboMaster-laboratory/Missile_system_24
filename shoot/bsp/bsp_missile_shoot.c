#include "bsp_missile_shoot.h"
#include "main.h"
extern TIM_HandleTypeDef htim1;
void missile_shoot_off(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, missile_shoot_OFF);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, missile_shoot_OFF);
}
void missile_shoot1_on(uint16_t cmd)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, cmd);
}
void missile_shoot2_on(uint16_t cmd)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, cmd);
}


