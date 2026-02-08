#ifndef _IMU_H
#define _IMU_H

#include "bmi088.h"
#include "control.h"

#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *3.14f *(t) ) ) ) *( (in) - (out) ))
void IMU_GetAngle(uint8_t on);
void IMU_Check_Offset(void);

#endif