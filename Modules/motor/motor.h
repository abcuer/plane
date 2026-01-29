#ifndef _MOTOR_H
#define _MOTOR_H

#include "stm32f1xx_hal.h"

void Motor_Init(void);
void MotorSetPWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

#endif