#include "GimbalControl.h"

M3508_HandleTypeDef Yaw_Gimbal_motorHandle;

void Gimbal_Init(void)
{
    M3508_Init(&Yaw_Gimbal_motorHandle,0x05, PID_POSITION, 1); 
}

void YAW_GimbalControl(fp32 Yaw_Speed)
{
    /*¶ÁÈ¡Ä¦²ÁÂÖ*/
    M3508_GetBasicData(&Yaw_Gimbal_motorHandle, 1);

    int16_t Set_Yaw_Speed = M3508_SpeedLoopPIDController(&(Yaw_Gimbal_motorHandle.PID_Speed_Control_Data), Yaw_Gimbal_motorHandle.Basic_Data->speed_rpm, Yaw_Speed);
    
    CAN_cmd_gimbal(Set_Yaw_Speed);
    
}