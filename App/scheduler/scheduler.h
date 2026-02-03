#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "stm32f1xx_hal.h"

typedef struct
{
	uint8_t check_flag;
	uint16_t err_flag;
	int16_t cnt_2ms;
	int16_t cnt_4ms;
	int16_t cnt_6ms;
	int16_t cnt_10ms;
	int16_t cnt_20ms;
	int16_t cnt_50ms;
	int16_t cnt_1000ms;
}loop_t;

void main_loop(void);
void Loop_check(void);
void Duty_2ms();
void Duty_4ms();
void Duty_6ms();
void Duty_10ms();
void Duty_20ms();
void Duty_50ms();
void Duty_1000ms();
#endif