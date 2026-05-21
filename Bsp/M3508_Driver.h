#ifndef M3508_DRIVER_H
#define M3508_DRIVER_H

#include "main.h"
#include "pid.h"
#include "CAN_receive.h"

/***************位置PID**********************/
//				摩擦轮
#define FRICTION_SPEEDPID_POSITION_KP   30.0f
#define FRICTION_SPEEDPID_POSITION_KI   10.0f
#define FRICTION_SPEEDPID_POSITION_KD   5.0f

//			Yaw轴M33508
#define YAW_SPEEDPID_POSITION_KP   3.8f
#define YAW_SPEEDPID_POSITION_KI   0.4f
#define YAW_SPEEDPID_POSITION_KD   0.01f
	
/***************增量PID***********************/
//				摩擦轮
#define FRICTION_SPEEDPID_DETAL_KP   3.8f
#define FRICTION_SPEEDPID_DETAL_KI   0.4f
#define FRICTION_SPEEDPID_DETAL_KD  0.01f

//			Yaw轴M33508
#define YAW_SPEEDPID_DETAL_KP   3.8f
#define YAW_SPEEDPID_DETAL_KI   0.4f
#define YAW_SPEEDPID_DETAL_KD  0.01f


typedef struct{
	
	uint8_t M3508_ID;//电机id
	
	const motor_measure_t *Basic_Data;//电流，速度，温度
	
	pid_type_def  PID_Speed_Control_Data;//速度环pid相关参数
	
	pid_type_def  PID_Positon_Control_Data;//位置环pid相关参数
	
}M3508_HandleTypeDef;

extern void M3508_Init(M3508_HandleTypeDef* device,uint8_t M3508_ID,enum PID_MODE Mode, uint8_t ifWheel);           
extern void M3508_GetBasicData(M3508_HandleTypeDef* device,uint8_t ifWheel);
extern int16_t M3508_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);
extern int16_t M3508_POSITION_LoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);

#endif
