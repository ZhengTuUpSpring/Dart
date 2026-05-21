/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "can.h"

#define Shoot_CAN hcan1
#define GIMBAL_CAN hcan1
#define GIMBAL_Trigger_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
    //can1 shoot
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
    
    //can1 云台电机
    CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_YAW_M3508_ID = 0x205,
    
    //can2 发射 拨弹 电机
    CAN_SHOOT_ALL_ID = 0x1FF,
    CAN_TRIGGER_MOTOR_X_ID = 0x206,
    CAN_TRIGGER_MOTOR_Y_ID = 0x207,   

} can_msg_id_e;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;


/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_Shoot(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          发送电机控制电流(0x205,0x206)
  * @param[in]      YAW_M3508: (0x205) 6020电机控制电流, 范围 [-30000,30000]0]
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t YAW_M3508);

/**
  * @brief          发送电机控制电流(0x206,0x207,0x208)00]
  * @param[in]      trigger_X:  (0x207) 3508电机控制电流, 范围 [-30000,30000]
  * @param[in]      trigger_Y: (0x208) 3508电机控制电流, 范围 [-30000,30000]
  * @retval         none
  */
extern void CAN_cmd_gimbal_Frie(int16_t trigger_X, int16_t trigger_Y);


/**
  * @brief          返回摩擦轮电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);


/**
  * @brief          返回yaw M3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_X_motor_measure_point(void);

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_Y_motor_measure_point(void);

#endif
