#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H

#include "bsp_can.h"
#include <stdint.h>



/* 电机信息结构体 */
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    uint16_t last_ecd;

    // 你可以在这里添加 PID 结构体、目标速度等
    // pid_t speed_pid;
} dji_motor_measure_t;

/* 电机对象结构体 */
typedef struct
{
    dji_motor_measure_t measure; // 测量数据
    CAN_HandleTypeDef *hcan;     // 挂载在哪个CAN总线
    uint32_t rx_id;              // 接收ID (如 0x201)
    float target_velocity;
    int16_t input_current;

    //void * data;
    // 可以添加发送ID等信息，或者分组发送指针
} dji_motor_object_t;




/* 初始化与注册函数 */
void DJI_Motor_Init(dji_motor_object_t *motor, CAN_HandleTypeDef *hcan, uint32_t id);

/* 发送函数封装 (示例：底盘4电机发送) */


void GM6020_PV_init(dji_motor_object_t *motor, uint32_t motor_ID, CAN_HandleTypeDef *hcan, int para_num, ...);
void GM6020_enable(dji_motor_object_t *motor);
void GM6020_disable(dji_motor_object_t *motor);
void GM6020_PV_update(dji_motor_object_t *motor);
void GM6020_PV_set_target(dji_motor_object_t *motor, float target);

#endif