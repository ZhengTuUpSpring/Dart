/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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

#include "CAN_receive.h"
#include "main.h"
#include "string.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//��ȡ�������
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }


/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
�������, 0:Ħ����1 3508���,   1Ħ����2 3508���,     2:Ħ����3 3508���, 3:Ħ����4 3508���;
          4:yaw��̨��� M3508���;  5:X���� 2006��� ; 6:Y�� 2006���;
*/
static motor_measure_t motor_chassis[7];


/**
  * @brief          hal��CAN�ص�����,���յ������  ������Ϣ�Һ��ж�
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */

/************************** CAN1-FIFO0�жϻص�������������ݣ� **************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // ������CAN1��FIFO0�жϣ�������CAN2����
    if (hcan != &hcan1) return;

    CAN_RxHeaderTypeDef CAN1_RX_header; // ��CAN1ʹ�õĽ���ͷ
    uint8_t CAN1_RX_data[8];            // ��CAN1ʹ�õĽ�������
    HAL_StatusTypeDef read_status;

    /************************** �ؼ����ж��쳣���� **************************/
    //���FIFO0��������������Ϣ�Ѷ�ʧ���������־��
    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_FOV0) != RESET)
    {
        __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV0); // ��������־
        return;
    }

    //��ȡFIFO0��Ϣ����鷵��ֵ�������ȡʧ�ܣ�
    read_status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_RX_header, CAN1_RX_data);
    if (read_status != HAL_OK)
    {
        return;
    }

    /************************** ��������������� **************************/
    // ������Ԥ��ĵ��ID��������Ч��Ϣռ����Դ��
    switch (CAN1_RX_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
            
        case CAN_YAW_M3508_ID:
        {
            static uint8_t can_1_motor_idx = 0;
            // ������������ȷ��ID��������ID�������赥����ֵ��
            can_1_motor_idx = CAN1_RX_header.StdId - CAN_3508_M1_ID;
            // ���������������Χ����������������Խ�磩
            if (can_1_motor_idx >= 7) break;
            // ��ȡ������ݣ�����˺����������ݽ�����������/��ʱ������
            get_motor_measure(&motor_chassis[can_1_motor_idx], CAN1_RX_data);
            break;
        }
        default:
            break;
    }
}
/************************** CAN2-FIFO1�жϻص��������װ����ݣ� **************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // ������CAN2��FIFO1�жϣ�������CAN1����
    if (hcan != &hcan2){ return;}

    CAN_RxHeaderTypeDef CAN2_RX_header; // ��CAN2ʹ�õĽ���ͷ
    uint8_t CAN2_RX_data[8];            // ��CAN2ʹ�õĽ�������
    HAL_StatusTypeDef read_status;

    /************************** �ؼ����ж��쳣���� **************************/
    //���FIFO1�����CAN2-FIFO1ר�������־��
    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_FOV1) != RESET)
    {
        __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV1);
        return;
    }

    //��ȡFIFO1��Ϣ����鷵��ֵ��
    read_status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN2_RX_header, CAN2_RX_data);
    if (read_status != HAL_OK)
    {
        return;
    }

    /************************** ���������װ����� **************************/
    switch (CAN2_RX_header.StdId)
    {
        case CAN_TRIGGER_MOTOR_X_ID:     
        case CAN_TRIGGER_MOTOR_Y_ID:
        {
            static uint8_t can_2_motor_idx = 0;
            // ������������ȷ��ID��������ID�������赥����ֵ��
            can_2_motor_idx = CAN2_RX_header.StdId - CAN_3508_M1_ID;
            // ���������������Χ����������������Խ�磩
            if (can_2_motor_idx >= 7) break;
            // ��ȡ������ݣ�����˺����������ݽ�����������/��ʱ������
            get_motor_measure(&motor_chassis[can_2_motor_idx], CAN2_RX_data);
            break;
        }
        default:
            break;
    }
}
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN_cmd_Shoot(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    HAL_StatusTypeDef CHASSIS_CAN_STATE;
    
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    if((CHASSIS_CAN_STATE = HAL_CAN_AddTxMessage(&Shoot_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box)) == HAL_ERROR)//hcan1
        HAL_CAN_AddTxMessage(&Shoot_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
/**
  * @brief          ���͵�����Ƶ���(0x205) M3508
  * @param[in]      YAW_M3508: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]0]
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t YAW_M3508)
{
    uint32_t send_mail_box;
    HAL_StatusTypeDef GIMBAL_CAN_STATE;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;         //YAW��ID Pitch��ID 
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x02;
	
    
    gimbal_can_send_data[0] = (YAW_M3508 >> 8);         //0x205
    gimbal_can_send_data[1] = YAW_M3508;        
 
    if((GIMBAL_CAN_STATE = HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box)) == HAL_ERROR) //hcan1
        HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

static CAN_TxHeaderTypeDef  gimbal_tx_message_Fire;
static uint8_t              gimbal_can_send_data_Fire[8];
/**
  * @brief          ���͵�����Ƶ���(0x206,0x207,0x208) M2006
  * @param[in]      trigger_X:  (0x206) 3508������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      trigger_Y: (0x207) 3508������Ƶ���, ��Χ [-30000,30000]
  * @retval         none
  */
void CAN_cmd_gimbal_Frie(int16_t trigger_X, int16_t trigger_Y)
{
    uint32_t send_mail_box;
    HAL_StatusTypeDef GIMBAL_SHOOT_CAN_STATE;
    gimbal_tx_message_Fire.StdId = CAN_SHOOT_ALL_ID;               //����������Ħ���ֵ�ALL_ID
    gimbal_tx_message_Fire.IDE = CAN_ID_STD;
    gimbal_tx_message_Fire.RTR = CAN_RTR_DATA;
    gimbal_tx_message_Fire.DLC = 0x06;

    gimbal_can_send_data_Fire[0] = (0 >> 8);
    gimbal_can_send_data_Fire[1] = 0;
    gimbal_can_send_data_Fire[2] = (trigger_X >> 8);    //0x206
    gimbal_can_send_data_Fire[3] = trigger_X;           
    gimbal_can_send_data_Fire[4] = (trigger_Y >> 8);    //0x207
    gimbal_can_send_data_Fire[5] = trigger_Y;
    
    if((GIMBAL_SHOOT_CAN_STATE = HAL_CAN_AddTxMessage(&GIMBAL_Trigger_CAN, &gimbal_tx_message_Fire, gimbal_can_send_data_Fire, &send_mail_box)) == HAL_ERROR) //hcan2
        HAL_CAN_AddTxMessage(&GIMBAL_Trigger_CAN, &gimbal_tx_message_Fire, gimbal_can_send_data_Fire, &send_mail_box);
}

/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&Shoot_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          ����Ħ���ֵ�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

/**
  * @brief          ����yaw M3508�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}


/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_trigger_X_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_trigger_Y_motor_measure_point(void)
{
    return &motor_chassis[6];
}

