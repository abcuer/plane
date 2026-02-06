#include "bsp_delay.h"
#include "scheduler.h"
#include "stm32f1xx_hal.h"

static uint32_t usTicks = 0;
volatile uint32_t SysTick_count = 0; 

void Delay_Init(void) 
{
    usTicks = SystemCoreClock / 1000000;
}

uint32_t GetSysTime_us(void) 
{
    uint32_t ms, cycle_cnt;
    uint32_t load = SysTick->LOAD; 
    do {
        ms = SysTick_count;
        cycle_cnt = SysTick->VAL;
    } while (ms != SysTick_count); 
    return (ms * 1000) + (load - cycle_cnt) / usTicks;
}

void delay_us(uint32_t nus)
{
    uint32_t t0 = GetSysTime_us();
    while((GetSysTime_us() - t0) < nus);
}

void delay_ms(uint32_t nms)
{
    delay_us(nms * 1000);
}

void HAL_SYSTICK_Callback(void) 
{
    SysTick_count++;
    static uint8_t cnt = 0;
    if(++cnt >= 2) {
        cnt = 0;
        Loop_check(); 
    }
}

// void delay_us(uint32_t us)
// {
//     uint32_t ticks_per_us = SystemCoreClock / 1000000;
//     uint32_t wait_ticks = us * ticks_per_us;
//     uint32_t start_tick = SysTick->VAL;
//     uint32_t current_tick;

//     // SysTick 是向下计数的
//     while (1) {
//         current_tick = SysTick->VAL;
//         uint32_t elapsed;
        
//         if (current_tick <= start_tick) {
//             elapsed = start_tick - current_tick;
//         } else {
//             // 处理了跨越 1ms 重装载边界的情况
//             elapsed = SysTick->LOAD - current_tick + start_tick;
//         }

//         if (elapsed >= wait_ticks) break;
//     }
// }

// void delay_us(uint32_t us)
// {
// 	SysTick->LOAD = SYS_CLK * us;			//设置定时器重装值
//     SysTick->VAL  = 0;
//     SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
//                     SysTick_CTRL_ENABLE_Msk;
//     while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
//     SysTick->CTRL = 0;    // 彻底关闭
// }

// void delay_ms(uint32_t ms)
// {
//     for(uint32_t i=0;i<ms;i++) delay_us(1000);
// }