#include "M2006_Driver.h"

int16_t int16_abs(int16_t value);
/**
  * @func			void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode)
  * @brief          2006电机相关参数初始化
  * @param[in]      device:2006电机句柄
  * @param[in]      M2006_ID:初始化电机id  7
  * @param[in]      PID_MODE:pid的控制模式(增量式，位置式) PID_DETAIL or PID_POSITION
  * @retval         none
  */
void M2006_Init(M2006_HandleTypeDef* device,uint8_t M2006_ID,enum PID_MODE PID_Mode)
{
	/*初始化电机相关参数*/
	device->M2006_ID = M2006_ID;
	device->Expand_ecd = 0;
	device->Cylinder_Number = 0;
	
	/*初始化pid相关参数*/
	fp32 PID_Position[3] = {0.0f};
	fp32 PID_Speed[3]    = {0.0f};
    
	if(PID_Mode == PID_POSITION)//位置式
	{
		switch(0x200 + M2006_ID)
		{
            
			case CAN_TRIGGER_MOTOR_X_ID:
			PID_Position[0] = UP2006_POSITIONPID_KP;
			PID_Position[1] = UP2006_POSITIONPID_KI;
			PID_Position[2] = UP2006_POSITIONPID_KD;
            
			PID_Speed[0] = UP2006_SPEEDPID_KP;
			PID_Speed[1] = UP2006_SPEEDPID_KI;
			PID_Speed[2] = UP2006_SPEEDPID_KD;
			break;
            
			case CAN_TRIGGER_MOTOR_Y_ID:
			PID_Position[0] = UP2006_POSITIONPID_KP;
			PID_Position[1] = UP2006_POSITIONPID_KI;
			PID_Position[2] = UP2006_POSITIONPID_KD;
            
			PID_Speed[0] = UP2006_SPEEDPID_KP;
			PID_Speed[1] = UP2006_SPEEDPID_KI;
			PID_Speed[2] = UP2006_SPEEDPID_KD;
			break;
            
			default : break;
		}
	}
	else if(PID_Mode == PID_DELTA)//增量式
	{
		switch(0x200 + M2006_ID)
		{          
			case CAN_TRIGGER_MOTOR_X_ID:
            PID_Position[0] = UP2006_POSITIONPID_DELTA_KP;
            PID_Position[1] = UP2006_POSITIONPID_DELTA_KI;
            PID_Position[2] = UP2006_POSITIONPID_DELTA_KD;
            
			PID_Speed[0] = UP2006_SPEEDPID_DELTA_KP;
			PID_Speed[1] = UP2006_SPEEDPID_DELTA_KI;
			PID_Speed[2] = UP2006_SPEEDPID_DELTA_KD;
			break;
            
            case CAN_TRIGGER_MOTOR_Y_ID:
            PID_Position[0] = UP2006_POSITIONPID_DELTA_KP;
            PID_Position[1] = UP2006_POSITIONPID_DELTA_KI;
            PID_Position[2] = UP2006_POSITIONPID_DELTA_KD;
            
			PID_Speed[0] = UP2006_SPEEDPID_DELTA_KP;
			PID_Speed[1] = UP2006_SPEEDPID_DELTA_KI;
			PID_Speed[2] = UP2006_SPEEDPID_DELTA_KD;
			break;
            
			default : break;
		}		
	}
    PID_init(&(device->PID_Position_Control_Data),PID_Mode, PID_Position, 16384,16384);//位置环
    PID_init(&(device->PID_Speed_Control_Data),   PID_Mode, PID_Speed,    16384,16384);//速度环;
}

/**
  * @func			void M2006_GetBasicData(M2006_HandleTypeDef* device)
  * @brief          获取2006电机相关参数
  * @param[in]      device:2006电机句柄
  * @retval         none
  */
void M2006_GetBasicData(M2006_HandleTypeDef* device)
{
	if(device == NULL)
	{
		return;
	}
	switch(device->M2006_ID + 0x200)
	{
        case CAN_TRIGGER_MOTOR_X_ID:
            device->Basic_Data = get_trigger_X_motor_measure_point();
        break;
        
        case CAN_TRIGGER_MOTOR_Y_ID:
            device->Basic_Data = get_trigger_Y_motor_measure_point();
        break;
		default : break;
			
	}
}

int16_t M2006_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
{
	if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
	    pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
	else
	{
		return 0;
	}
}

/**
  * @func			int16_t M2006_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006速度环pid控制函数
  * @param[in]      pid:2006电机速度环pid句柄
  * @param[in]      ref:2006电机实际转速
  * @param[in]      set:2006电机目标转速
  * @retval         2006速度环pid输出值
  */
int16_t M2006_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
{
	if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
	else
	{
		return 0;
	}
}

