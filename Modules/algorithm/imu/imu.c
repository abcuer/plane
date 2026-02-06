#include "imu.h"
#include "math.h"

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

/*
 * 姿态融合
 * 单位: m/s^2   rad/s
*/
static void ImuUpdate(float gx, float gy, float gz, float ax, float ay, float az)//g表陀螺仪，a表加计
{
 	float norm;
	
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q1q1 = q1 * q1;
	float q1q3 = q1 * q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;	
	float vx, vy, vz;
	float ex, ey, ez;
	
	float q0_yawq0_yaw = q0_yaw * q0_yaw;
	float q1_yawq1_yaw = q1_yaw * q1_yaw;
	float q2_yawq2_yaw = q2_yaw * q2_yaw;
	float q3_yawq3_yaw = q3_yaw * q3_yaw;
	float q1_yawq2_yaw = q1_yaw * q2_yaw;
	float q0_yawq3_yaw = q0_yaw * q3_yaw;
	
	//**************************Yaw轴计算******************************
	//Yaw轴四元素的微分方程，先单独解出yaw的姿态
	q0_yaw = q0_yaw + (-q1_yaw * gx - q2_yaw * gy - q3_yaw * gz) * halfT;	//halfT采样时间的一半
	q1_yaw = q1_yaw + (q0_yaw * gx + q2_yaw * gz - q3_yaw * gy) * halfT;
	q2_yaw = q2_yaw + (q0_yaw * gy - q1_yaw * gz + q3_yaw * gx) * halfT;
	q3_yaw = q3_yaw + (q0_yaw * gz + q1_yaw * gy - q2_yaw * gx) * halfT;
	
	//规范化Yaw轴四元数
	norm = sqrt(q0_yawq0_yaw + q1_yawq1_yaw + q2_yawq2_yaw + q3_yawq3_yaw);
	q0_yaw = q0_yaw / norm;
	q1_yaw = q1_yaw / norm;
	q2_yaw = q2_yaw / norm;
	q3_yaw = q3_yaw / norm;
	
	if(ax * ay * az	== 0)//如果加速度数据无效，或者自由坠落，不结算
	return ;
	
	//规范化加速度计值
	norm = sqrt(ax * ax + ay * ay + az * az); 
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;
		
	//估计重力方向和流量/变迁，重力加速度在机体系的投影
	vx = 2 * (q1q3 - q0q2);											
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;
		
	//向量外积再相减得到差分就是误差
	ex = (ay * vz - az * vy) ;      
	ey = (az * vx - ax * vz) ;
	ez = (ax * vy - ay * vx) ;
	
	//对误差进行PI计算
	xErrorInt = xErrorInt + ex * Ki;			
	yErrorInt = yErrorInt + ey * Ki;
	zErrorInt = zErrorInt + ez * Ki;
	
	//校正陀螺仪
	gx = gx + Kp * ex + xErrorInt;					
	gy = gy + Kp * ey + yErrorInt;
	gz = gz + Kp * ez + zErrorInt;			
				
	//四元素的微分方程
	q0 = q0 + (-q1 * gx - q2	*	gy - q3	*	gz)	*	halfT;
	q1 = q1 + (q0	*	gx + q2	*	gz - q3	*	gy)	*	halfT;
	q2 = q2 + (q0	*	gy - q1	*	gz + q3	*	gx)	*	halfT;
	q3 = q3 + (q0	*	gz + q1	*	gy - q2	*	gx)	*	halfT;
	
	//规范化Pitch、Roll轴四元数
	norm = sqrt(q0q0 + q1q1 + q2q2 + q3q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	
	//求解欧拉角
	bmi.pitch = atan2(2 * q2q3 + 2 * q0q1, -2 * q1q1 - 2 * q2q2 + 1) * 57.3f;
	bmi.roll = asin(-2 * q1q3 + 2 * q0q2) * 57.3f;
	bmi.yaw = atan2(2 * q1_yawq2_yaw + 2 * q0_yawq3_yaw, -2 * q2_yawq2_yaw - 2 * q3_yawq3_yaw + 1)	* 57.3f;
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
	
	LPF_1_(1.8f,0.002f ,bd_gx*0.0174533f, gx);
	LPF_1_(1.8f,0.002f ,bd_gy*0.0174533f, gy);
	
	acc_sp[0] += (az-0.995f)/3.0f;
	acc_sp[1] += ax * 5.2f;
	acc_sp[2] -= ay * 5.2f;	
	
	if(on == 1)
		ImuUpdate(bd_gx*0.0174533f, bd_gy*0.0174533f, bd_gz*0.0174533f, bd_ax, bd_ay, bd_az);
}
