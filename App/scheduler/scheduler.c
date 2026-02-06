#include "scheduler.h"
#include "headfile.h"
#include "led.h"

loop_t loop; 
//u32 time[10],time_sum;
int32_t time[10],time_sum;
 
//2msÖ´ÐÐÒ»´Î
//ÔÚdelay.cµ÷ÓÃ£¬2msÒ»´Î
void Loop_check()
{
	loop.cnt_2ms++;
	loop.cnt_4ms++;
	loop.cnt_6ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;
	loop.cnt_1000ms++;

	// if( loop.check_flag >= 1)
	// {
	// 	loop.err_flag ++;// 2ms 
	// }
	// else
	// {
	// 	loop.check_flag += 1;   //¸Ã±êÖ¾Î»ÔÚÑ­»·ºóÃæÇå0
	// }
	
}

void main_loop()
{
	// if( loop.check_flag >= 1 )
	// {
		
		if( loop.cnt_2ms >= 1 )
		{
			loop.cnt_2ms = 0;
            SetLedMode(bLEDL, LED_ON);
            SetLedMode(bLEDR, LED_OFF);
			Duty_2ms();	 					//ÖÜÆÚ2msµÄÈÎÎñ
		}
		// if( loop.cnt_4ms >= 2 )
		// {
		// 	loop.cnt_4ms = 0;
		// 	Duty_4ms();						//ÖÜÆÚ4msµÄÈÎÎñ
		// }
		if( loop.cnt_6ms >= 3 )
		{
            SetLedMode(bLEDR, LED_ON);
            SetLedMode(bLEDL, LED_OFF);
			loop.cnt_6ms = 0;
			Duty_6ms();						//ÖÜÆÚ6msµÄÈÎÎñ
		}
		// if( loop.cnt_10ms >= 5 )
		// {
		// 	loop.cnt_10ms = 0;
		// 	Duty_10ms();					//ÖÜÆÚ10msµÄÈÎÎñ
		// } 
		// if( loop.cnt_20ms >= 10 )
		// {
		// 	loop.cnt_20ms = 0;
		// 	Duty_20ms();					//ÖÜÆÚ20msµÄÈÎÎñ
		// }
		// if( loop.cnt_50ms >= 25 )
		// {
		// 	loop.cnt_50ms = 0;
		// 	Duty_50ms();					//ÖÜÆÚ50msµÄÈÎÎñ
		// }
		// if( loop.cnt_1000ms >= 500)
		// {
		// 	loop.cnt_1000ms = 0;
		// 	Duty_1000ms();				//ÖÜÆÚ1sµÄÈÎÎñ
		// }
		// loop.check_flag = 0;		//Ñ­»·ÔËÐÐÍê±Ï±êÖ¾
	// }
}

void Duty_2ms()
{
    time[0] = GetSysTime_us();

    BMI088_GetData(&bmi);

    time[0] = GetSysTime_us() - time[0];
}

void Duty_6ms()
{
    time[1] = GetSysTime_us();

    IMU_GetAngle(&bmi, 0.006f); // 6ms调用一次

    time[1] = GetSysTime_us() - time[1];
}