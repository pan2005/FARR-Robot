//
// Created by Admin on 2025/11/9.
//

#ifndef __LKMF9025_H
#define __LKMF9025_H
#include "main.h"
#include "daemon.h"
#define LK9025_Single_StdId 0x140
#define Torque_Control_Mode 0xA1
#define Speed_Control_Mode 0xA2
#define Multi_Loop_Control_Mode1 0xA3
#define Multi_Loop_Control_Mode2 0xA4
#define Single_Loop_Control_Mode1 0xA5
#define Single_Loop_Control_Mode2 0xA6   //加入了MAXSpeed 设置
#define Increment_Control_Mode1 0xA7
#define Increment_Control_Mode2 0xA8


typedef struct
{
    /* 配置部分 */
    uint8_t id;                  // 电机 ID (1, 2, 3...)
    CAN_HandleTypeDef *hcan;     // 挂载的 CAN

    /* 状态反馈 */
    int16_t speed;
    int16_t iq;
    int16_t encoder;
    uint8_t tempreture;

    /* 外部依赖 (Daemon/Buzzer 等可按需添加指针) */
    void *daemon;
} LK_Motor_t;

/**
 * @brief 初始化电机并注册接收回调
 * @param motor 电机对象指针
 * @param hcan CAN 句柄指针
 * @param id 电机物理 ID (1-based)
 */
void LK_Motor_Init(LK_Motor_t *motor, CAN_HandleTypeDef *hcan, uint8_t id);

/**
 * @brief 速度闭环控制
 * @param motor 电机对象指针
 * @param speed 目标速度
 */
void LK_Motor_SpeedControl(LK_Motor_t *motor, int32_t speed);
void LK_Motor_Increment_AngleControl2(LK_Motor_t *motor ,int32_t angleControl,uint16_t maxSpeed);
extern  LK_Motor_t YAW_Motor;


#endif