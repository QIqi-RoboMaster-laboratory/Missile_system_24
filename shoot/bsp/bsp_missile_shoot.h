#ifndef BSP_missile_shoot_H
#define BSP_missile_shoot_H
#include "struct_typedef.h"

#define missile_shoot_UP 1400
#define missile_shoot_DOWN 1320
#define missile_shoot_OFF 1000

extern void missile_shoot_off(void);
extern void missile_shoot1_on(uint16_t cmd);
extern void missile_shoot2_on(uint16_t cmd);
#endif
