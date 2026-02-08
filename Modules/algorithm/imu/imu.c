#include "imu.h"
#include "math.h"
#include "stm32f1xx_hal.h"

#define sampleFreq	125.0f			// sample frequency in Hz
/*采样周期的一半，用于求解四元数微分方程时计算角增量
请确定自己的姿态调用周期: 8ms,即上面的sampleFreq: 125Hz*/
#define halfT 0.004f

//这里的Kp,Ki是用于控制加速度计修正陀螺仪积分姿态的速度
#define Kp 2.0f  		//2.0f
#define Ki 0.0016f  	//0.002f

float gx=0, gy=0, gz=0;				//由角速度计算的角速率
float ax=0, ay=0, az=0;				//由加速度计算的加速度

float acc_sp[3];						//积分速度
float acc_sp_RHRH[3];				//处理后速度

float bd_gx=0, bd_gy=0, bd_gz=0;
float bd_ax=0, bd_ay=0, bd_az=0;

//初始姿态四元数(地理坐标系)，q(q0,q1,q2,q3)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;    //最优估计四元数
float q0_yaw = 1.0f, q1_yaw = 0.0f, q2_yaw = 0.0f, q3_yaw = 0.0f;    //弥补Mahony算法在无地磁情况解算Yaw轴满足不了大扰动要求的现象
//定义姿态解算误差的积分
//当前加计测得的重力加速度在三轴(x,y,z)上的分量,与当前姿态计算得来的重力在三轴上的分量的误差的积分
float xErrorInt = 0.0f, yErrorInt = 0.0f, zErrorInt = 0.0f;

float gz_offset = 0.0f; // Z轴零偏偏移量
uint8_t is_calibrated = 0;

// 在系统启动后调用此函数一次，期间保持板子绝对静止
void IMU_Check_Offset(void) 
{
    float sum_gz = 0;
    const int sample_count = 500;
    
    for(int i = 0; i < sample_count; i++) {
        BMI088_GetData(&bmi); // 获取原始数据
        sum_gz += (float)bmi.gyro[2] * 0.0609756f; // 累加 Z 轴度/秒
        HAL_Delay(2); // 采样间隔
    }
    gz_offset = sum_gz / (float)sample_count; // 计算平均偏置
    is_calibrated = 1;
}
// 90秒偏14度
/*
 * 姿态融合
 * 单位: m/s^2   rad/s
*/
static void ImuUpdate(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // 1. 规范化加速度计值
    norm = sqrt(ax * ax + ay * ay + az * az); 
    if(norm < 0.1f) return;
    ax /= norm; ay /= norm; az /= norm;
        
    // 2. 估计重力投影
    vx = 2.0f * (q1*q3 - q0*q2);                                            
    vy = 2.0f * (q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3 ;
        
    // 3. 计算误差 (这里 ez 在无磁力计时理论上接近 0)
    ex = (ay * vz - az * vy);      
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
    
    // 4. PI 修正 (增加对 ez 的观察)
    xErrorInt += ex * Ki;            
    yErrorInt += ey * Ki;
    zErrorInt += ez * Ki;
    
    gx += Kp * ex + xErrorInt;                    
    gy += Kp * ey + yErrorInt;
    gz += Kp * ez + zErrorInt;            
                
    // 5. 四元数更新 (严格使用 halfT)
    // 这里的 gx, gy, gz 必须是 rad/s
    float q0_n = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
    float q1_n = q1 + ( q0*gx + q2*gz - q3*gy) * halfT;
    float q2_n = q2 + ( q0*gy - q1*gz + q3*gx) * halfT;
    float q3_n = q3 + ( q0*gz + q1*gy - q2*gx) * halfT;
    
    // 6. 归一化
    norm = sqrt(q0_n*q0_n + q1_n*q1_n + q2_n*q2_n + q3_n*q3_n);
    q0 = q0_n / norm; q1 = q1_n / norm; q2 = q2_n / norm; q3 = q3_n / norm;
    
    // 7. 提取角度
    bmi.pitch = atan2(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * 57.29578f - 3.3f;
    bmi.roll  = asin(2.0f * (q0*q2 - q1*q3)) * 57.29578f - 45.3f;
    // Yaw 轴直接提取
    bmi.yaw   = atan2(2.0f * (q1*q2 + q0*q3), 1.0f - 2.0f * (q2*q2 + q3*q3)) * 57.29578f;
}

void IMU_GetAngle(uint8_t on)
{
    BMI088_GetData(&bmi);

	//陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据，单位为：°/s
	//陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
	//陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
	//陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

	//加速度计量程为:±2g        获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
	//加速度计量程为:±4g        获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
	//加速度计量程为:±8g        获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
	//加速度计量程为:±16g       获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
	
	ax = (float)bmi.acc[0] * 0.000183105f; 
	ay = (float)bmi.acc[1] * 0.000183105f;
	az = (float)bmi.acc[2] * 0.000183105f;
	   
	bd_gx = (float)bmi.gyro[0] * 0.0609756f;
	bd_gy = (float)bmi.gyro[1] * 0.0609756f;
	bd_gz = (float)bmi.gyro[2] * 0.0609756f;
	
	bd_ax = 1.0064f*ax - 717.6192f;
	bd_ay = 1.0104f*ay + 39.3216f;
	bd_az = 0.9762f*az + 704.5120f;

	// 1. 计算带单位的陀螺仪数据 (°/s)
    float raw_gz = (float)bmi.gyro[2] * 0.0609756f;
    
    // 2. 减去静态零偏
    bd_gz = raw_gz - gz_offset;
	if(fabs(bd_gz) < 0.05f) bd_gz = 0.0f;
	
	LPF_1_(1.8f, 0.002f, bd_gx*0.0174533f, gx);
	LPF_1_(1.8f, 0.002f, bd_gy*0.0174533f, gy);
	
	acc_sp[0] += (az-0.995f)/3.0f;
	acc_sp[1] += ax * 5.2f;
	acc_sp[2] -= ay * 5.2f;	
	
	if(on == 1 && is_calibrated)
		ImuUpdate(bd_gx*0.0174533f, bd_gy*0.0174533f, bd_gz*0.0174533f, bd_ax, bd_ay, bd_az);
}
