#ifndef _IMU_H
#define _IMU_H

#include "bmi088.h"

#define DEG2RAD  0.017453293f //¶È×ª»¡¶È¦Ð/180 
#define RAD2DEG  57.29578f    //»¡¶È×ª¶È180/¦Ð 

typedef struct {
	float yaw;
	float pitch;
	float roll;
	float yaw_mag; //µ¥¶ÀÓÉ´ÅÁ¦¼ÆµÄ³öÀ´µÄ½Ç¶È
	float Cos_Roll;
	float Sin_Roll;
	float Cos_Pitch;
	float Sin_Pitch;
	float Cos_Yaw;
	float Sin_Yaw;
}_st_IMU;

typedef volatile struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;

typedef struct V{
            float x;
            float y;
            float z;
} MPUDA;

void IMU_GetAngle(IMU_t *imu, float dt); 
// extern float yaw_control;
// extern float Yaw_Correct;
// extern _st_IMU IMU;
// extern float GetAccz(void);
// extern void GetAngle(const _st_Mpu *pMpu,_st_AngE *pAngE, float dt);
// extern	 MPUDA  Gravity, Acc, Gyro,AccGravity;

#endif