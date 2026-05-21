#ifndef M2006_DRIVER_H
#define M2006_DRIVER_H

#include "main.h"
#include "pid.h"
#include "CAN_receive.h"
#include "math.h"

#define M2006_REDUCTION_RATIO 36.000000 //减速比36:1

/*速度PID参数*/
#define UP2006_SPEEDPID_KP 8.0f
#define UP2006_SPEEDPID_KI 0.005f
#define UP2006_SPEEDPID_KD 0.001f

/*位置PID参数*/
#define UP2006_POSITIONPID_KP 3.0f
#define UP2006_POSITIONPID_KI 0.005f
#define UP2006_POSITIONPID_KD 0.0005f

/*增量式PID参数*/
#define UP2006_SPEEDPID_DELTA_KP 4.0f
#define UP2006_SPEEDPID_DELTA_KI 0.4f
#define UP2006_SPEEDPID_DELTA_KD 2.0f

#define UP2006_POSITIONPID_DELTA_KP 0.0f
#define UP2006_POSITIONPID_DELTA_KI 0.0f
#define UP2006_POSITIONPID_DELTA_KD 0.0f

typedef struct{
	
	uint8_t M2006_ID;//电机id
	
	int  Expand_ecd;//电机减速箱旋转轴 绝对编码器值
	
	int  Cylinder_Number;//旋转圈数
	
	const motor_measure_t *Basic_Data;//电流，速度，温度
	
    pid_type_def  PID_Position_Control_Data;//位置环pid相关参数
	
	pid_type_def  PID_Speed_Control_Data;//速度环pid相关参数
	
}M2006_HandleTypeDef;


/**
  * @func			void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode)
  * @brief          2006电机相关参数初始化
  * @param[in]      device:2006电机句柄
  * @param[in]      M2006_ID:初始化电机id  7
  * @param[in]      PID_MODE:pid的控制模式(增量式，位置式) PID_DETAIL or PID_POSITION
  * @retval         none
  */
extern void M2006_Init(M2006_HandleTypeDef* device,uint8_t M2006_ID,enum PID_MODE PID_Mode);

/**
  * @func			void M2006_GetBasicData(M2006_HandleTypeDef* device)
  * @brief          获取2006电机相关参数
  * @param[in]      device:2006电机句柄
  * @retval         none
  */
extern void M2006_GetBasicData(M2006_HandleTypeDef* device);

/**
  * @func			int16_t M2006_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006位置环pid控制函数
  * @param[in]      pid:2006电机位置环pid句柄
  * @param[in]      ref:2006电机实际角度
  * @param[in]      set:2006电机目标角度
  * @retval         2006位置环pid输出值
  */
extern int16_t M2006_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);
/**
  * @func			int16_t M2006_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006速度环pid控制函数
  * @param[in]      pid:2006电机速度环pid句柄
  * @param[in]      ref:2006电机实际转速
  * @param[in]      set:2006电机目标转速
  * @retval         2006速度环pid输出值
  */
extern int16_t M2006_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);


#endif
