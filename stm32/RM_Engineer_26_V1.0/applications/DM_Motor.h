//
// Created by Admin on 2026/1/12.
//

#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H

#include "main.h"
/**********************************************************************************************************************/
/* 达妙电机——MIT控制 */

struct DM_MIT_data {
    //控制量
    float _p_des;
    float _v_des;
    float _t_ff;

    //参数
    float _kp;
    float _kd;

    //反馈量
    int8_t ERR;  //电机错误状态反馈
    int16_t POS;  //当前电机位置反馈
    int16_t VEL;  //当前电机转速反馈
    int16_t T;  //当前电机转矩反馈
    int8_t TEMP_MOS;  //电机MOS温度反馈
    int8_t TEMP_Rotor;  //电机线圈温度反馈

    //控制幅值
    float P_MAX;
    float V_MAX;
    float T_MAX;
    uint32_t last_rx_tick; // 新增：记录最后一次收到CAN数据的时间
};

typedef struct DM_Motor_obj_t {
    struct DM_MIT_data motor_data;
    CAN_HandleTypeDef *motor_can_handle;     // 挂载在哪个CAN总线
    uint32_t motor_id;              // 接收ID (如 0x201)


}DM_Motor_obj_t;


void DM_MIT_init(DM_Motor_obj_t *motor, uint32_t motor_ID, CAN_HandleTypeDef *hcan, int para_num, ...);

void DM_MIT_send_ctrl_cmd(DM_Motor_obj_t *motor);

void DM_MIT_send_disable_cmd(DM_Motor_obj_t *motor);

void DM_MIT_send_enable_cmd(DM_Motor_obj_t *motor);

void DM_MIT_send_save_zero_cmd(DM_Motor_obj_t *motor);

void DM_MIT_set_target(DM_Motor_obj_t *motor, const int para_num, ...);

void DM_MIT_get_status(DM_Motor_obj_t *motor, const char* which_status, void* status_data) ;

void DM_MIT_set_para(DM_Motor_obj_t *motor, const char* which_para, void* para_data);

void DM_MIT_clear_error(DM_Motor_obj_t *motor);

#endif
