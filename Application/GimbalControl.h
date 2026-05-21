#ifndef GIMBALCONTEOL_H
#define GIMBALCONTEOL_H

#include "M3508_Driver.h"

extern M3508_HandleTypeDef Yaw_Gimbal_motorHandle;
extern void Gimbal_Init(void);
extern void YAW_GimbalControl(fp32 Yaw_Speed);
#endif
