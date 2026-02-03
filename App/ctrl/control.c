#include "control.h"
#include "pid.h"

IMU_t bmi;   

PID_t pitch_pid = 
{
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    .tar = 0.0f,
    .deadband = 0.5f,
    .max_out = 2000.0f, // PWM 最大值 2500
    .mode = POSITION_PID,
};

PID_t roll_pid =
{
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    .max_out = 19000.0f,
    .mode = POSITION_PID,
};

void PIDParam_Init(void)
{
    PID_Init(&pitch_pid);
    PID_Init(&roll_pid);
}
/*
void DistPidCtrl(void)
{
	dist.tar = dist_pid.tar;
	dist.now = distance;
	PidCalculate(&dist);
	speed_pid.tar = dist.out;
}

*/
void PitchPidCtrl(void)
{
    pitch_pid.now = bmi.pitch;
    PidCalculate(&pitch_pid);
    
}