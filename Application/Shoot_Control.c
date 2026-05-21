#include "Shoot_Control.h"

/*拨盘电机句柄*/
M2006_HandleTypeDef Trigger_motor[2];
/*摩擦轮电机句柄 */
M3508_HandleTypeDef FrictionWheel_motorHandle[4];

/**
  * @func			void Shoot_Init(void)
  * @brief          射击驱动初始化
  * @param[in]      none
  * @retval         none
  */
void Shoot_Init(void)
{
	/*摩擦轮电机初始化.拨弹电机初始化*/
   
    M3508_Init(&FrictionWheel_motorHandle[0],0x01, PID_POSITION, 0);
    M3508_Init(&FrictionWheel_motorHandle[1],0x02, PID_POSITION, 0);
    M3508_Init(&FrictionWheel_motorHandle[2],0x03, PID_POSITION, 0);
    M3508_Init(&FrictionWheel_motorHandle[3],0x04, PID_POSITION, 0);
	
    
    M2006_Init(&Trigger_motor[0],0x06, PID_DELTA);
    M2006_Init(&Trigger_motor[1],0x07, PID_DELTA);
    
}

int16_t Set_Shoot_Speed[4] = {0}; 

void Shoot_FrictionWheelControl(fp32 setSpeed_UL, fp32 setSpeed_UR, fp32 setSpeed_DL,fp32 setSpeed_DR)
{
    /*读取摩擦轮*/
    M3508_GetBasicData(&FrictionWheel_motorHandle[0], 0);
    M3508_GetBasicData(&FrictionWheel_motorHandle[1], 0);
    M3508_GetBasicData(&FrictionWheel_motorHandle[2], 0);
    M3508_GetBasicData(&FrictionWheel_motorHandle[3], 0);

    Set_Shoot_Speed[0] = M3508_SpeedLoopPIDController(&(FrictionWheel_motorHandle[0].PID_Speed_Control_Data), FrictionWheel_motorHandle[0].Basic_Data->speed_rpm, setSpeed_UL);
    Set_Shoot_Speed[1] = M3508_SpeedLoopPIDController(&(FrictionWheel_motorHandle[1].PID_Speed_Control_Data), FrictionWheel_motorHandle[1].Basic_Data->speed_rpm, setSpeed_UR);
    Set_Shoot_Speed[2] = M3508_SpeedLoopPIDController(&(FrictionWheel_motorHandle[2].PID_Speed_Control_Data), FrictionWheel_motorHandle[2].Basic_Data->speed_rpm, setSpeed_DL);
    Set_Shoot_Speed[3] = M3508_SpeedLoopPIDController(&(FrictionWheel_motorHandle[3].PID_Speed_Control_Data), FrictionWheel_motorHandle[3].Basic_Data->speed_rpm, setSpeed_DR);

    CAN_cmd_Shoot(Set_Shoot_Speed[0], Set_Shoot_Speed[1], Set_Shoot_Speed[2], Set_Shoot_Speed[3]);
}

int16_t Set_Trigger_Speed[2] = {0}; 

void Shoot_TriggerWheelControl(fp32 setSpeed_X, fp32 setSpeed_Y)
{
    M2006_GetBasicData(&Trigger_motor[0]);
    M2006_GetBasicData(&Trigger_motor[1]);
    
    Set_Trigger_Speed[0] = M2006_SpeedLoopPIDController(&(Trigger_motor[0].PID_Speed_Control_Data),Trigger_motor[0].Basic_Data->speed_rpm,setSpeed_X);
    Set_Trigger_Speed[1] = M2006_SpeedLoopPIDController(&(Trigger_motor[1].PID_Speed_Control_Data),Trigger_motor[1].Basic_Data->speed_rpm,setSpeed_Y);
    
    CAN_cmd_gimbal_Frie(Set_Trigger_Speed[0], Set_Trigger_Speed[1]);
}


