#include "headfile.h"
#include "bsp_delay.h"
#include "led.h"

/**
 * @brief 系统初始化函数，初始化所有模块和外设
 * @param 无
 * @retval 无
 */
void System_Init(void)
{
    LedDevice_Init();
    if(BMI088_Init() == 0) 
    {
        SetLedMode(rLEDL, LED_ON); // 左红灯表示BMI088初始化成功
        delay_ms(500);
    }

    // NRF24L01_init();

    // SPL06_Init();
    // Flow_Init();    
//    Motor_Init();
    delay_ms(3000);
}
