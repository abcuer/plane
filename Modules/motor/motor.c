#include "motor.h"
#include "tim.h"

void Motor_Init(void) 
{
    // 启动定时器 PWM 输出
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // M1
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // M2
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // M3
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // M4
}

static void Update_Motor_PWM(uint8_t id, uint16_t pulse) 
{
    if(pulse < 1000) pulse = 1000;
    if(pulse > 2000) pulse = 2000;

    switch(id) 
    {
        case 0: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse); break; // M1
        case 1: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse); break; // M2
        case 2: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse); break; // M3
        case 3: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse); break; // M4
    }
}

void MotorSetPWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    Update_Motor_PWM(0, m1);
    Update_Motor_PWM(1, m2);
    Update_Motor_PWM(2, m3);
    Update_Motor_PWM(3, m4);
}