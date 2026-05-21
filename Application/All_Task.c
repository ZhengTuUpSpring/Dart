#include "All_Task.h"
#define FrictionWheel_Speed 6000.0f

void Shoot_Task(void *argument)
{
    for(;;)
    {
        if(rc_ctrl.rc.s[1] == 1)
        {
            Shoot_FrictionWheelControl(-FrictionWheel_Speed, FrictionWheel_Speed, FrictionWheel_Speed, -FrictionWheel_Speed);
        }
        else if(rc_ctrl.rc.s[1] == 3)
        {
            Shoot_FrictionWheelControl(0, 0, 0, 0);
        }

        else
        {
            CAN_cmd_Shoot(0, 0, 0, 0);
        }
        
        osDelay(5);
    }
}

#define Gimbal_Speed_K 12.0f


void Gimbal_Task(void *argument)
{
    for(;;)
    {
//		if(rc_ctrl.rc.s[0] == 1)
//		{
//			YAW_GimbalControl(6000);
//		}
//		
//		else
//		{
			YAW_GimbalControl(rc_ctrl.rc.ch[2] * Gimbal_Speed_K);
//		}
        
        osDelay(5);
    }
}

#define Trigger_Speed_X_K 10.0f
#define Trigger_Speed_Y_K 14.0f

void Trigger_X_Y_Task(void *argument)
{
    for(;;)
    {
//		if(rc_ctrl.rc.s[0] == 1)
//		{
//			Shoot_TriggerWheelControl(rc_ctrl.rc.ch[0] * Trigger_Speed_K, 6000);
//		}
//		
//		else
//		{
			Shoot_TriggerWheelControl(rc_ctrl.rc.ch[0] * Trigger_Speed_X_K, rc_ctrl.rc.ch[1] * Trigger_Speed_Y_K);
//		}
        osDelay(5);
    }
}

int speed = 0;
void VOFA_Task(void *argument)
{
    for(;;)
    {
//		if(rc_ctrl.rc.s[0] == 1)
//		{
//			speed = 6000;
//		}
//		
//		else
//		{
//			speed = 0;
//		}
//        printf("%d, %d\n",Trigger_motor[1].Basic_Data->speed_rpm, speed);
        osDelay(10);
    }
}