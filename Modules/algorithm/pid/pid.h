#ifndef __PID_H_
#define __PID_H_

#include "stdint.h"

/* PID 模式枚举 */
enum
{
    POSITION_PID = 0,  // 位置式
    DELTA_PID          // 增量式
};

/* PID 结构体定义 */
typedef struct
{
    // 控制参数
    float p;
    float i;
    float d;
    uint8_t mode;

    // 目标与当前值
    float tar;   
    float now;
    float error[3]; // error[0]:当前误差, error[1]:上一次, error[2]:上上次

    // 输出分量
    float pout;
    float iout;
    float dout;
    float out;          // 最终总输出
    float max_out;      // 总输出限幅：对应 PWM 的最大值
    float max_iout;     // 积分限幅（抗饱和）
    float deadband;     // 死区：误差在这个范围内时，PID 不起作用，防止电机震荡
    float d_filter_hz;  // 微分项低通滤波频率
    
} PID_t;

/* 函数声明 */
/**
 * @brief PID 初始化
 * @param pid: 结构体指针
 * @param mode: 位置式或增量式
 * @param p/i/d: 三项系数
 * @param max_i: 积分限幅值
 * @param max_o: 总输出限幅值
 */
void PID_Init(PID_t *pid);
void PidCalculate(PID_t *pid);
void PidReset(PID_t *pid);

#endif