#ifndef SHOOT_CONTROL_H
#define SHOOT_CONTROL_H

#include "M3508_Driver.h"
#include "M2006_Driver.h"

extern M2006_HandleTypeDef Trigger_motor[2];
extern M3508_HandleTypeDef FrictionWheel_motorHandle[4];

extern void Shoot_Init(void);
extern void Shoot_FrictionWheelControl(fp32 setSpeed_UL, fp32 setSpeed_UR, fp32 setSpeed_DL,fp32 setSpeed_DR);
extern void Shoot_TriggerWheelControl(fp32 setSpeed_X, fp32 setSpeed_Y);

#endif
