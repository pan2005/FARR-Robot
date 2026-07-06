#include <math.h>
#include <stdio.h>
#include <string.h>

#include "dji_motor.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/**
  * @brief 具体的解码逻辑 (私有函数)
  * @param device: 在注册时传入的 dji_motor_object_t 指针
  * @param data: CAN 原始数据
  */
static void _dji_motor_decode_callback(void *device, uint8_t *data)
{
    dji_motor_object_t *motor = (dji_motor_object_t *)device;
    dji_motor_measure_t *ptr = &(motor->measure);

    ptr->last_ecd = ptr->ecd;
    ptr->ecd = (uint16_t)((data[0] << 8) | data[1]);
    ptr->speed_rpm = (int16_t)((data[2] << 8) | data[3]);
    ptr->given_current = (int16_t)((data[4] << 8) | data[5]);
    ptr->temperate = data[6];
}


/*typedef struct GM6020_data_t {
    // 目标量
    float   _p_des;          // 目标位置 (rad)
    float   _v_des_internal; // 内部速度目标 (位置环输出)
    int16_t _out_output;     // 最终电压输出 (-30000 ~ 30000)

    // 位置环 PID (外环)
    float _kp_p;
    float _ki_p;
    float _i_term_p;
    float _i_p_max;

    // 速度环 PID (内环)
    float _kp_v;
    float _ki_v;
    float _kd_v;
    float _i_term_v;
    float _i_v_max;
    float _last_d_out;
    float _d_filter_alpha;
    int16_t _last_v_error;

    // 反馈量
    dji_motor_measure_t measure;


    // 限制与安全
    float   _v_limit;        // 最大限制转速 (rpm)
    float   _out_max;        // 最大输出电压

    // 使能状态
    uint8_t enable_flag;
}GM6020_data_t;

typedef struct M3508_data_t {
    float _target_velocity;//单位RPM
    int16_t _output_current;//输出电流

    float _Kp;  //Kp
    float _Ki;  //Ki
    float _Kd;
    float output_limit;


}M3508_data_t;*/
/**
  * @brief 初始化一个电机对象并注册到CAN驱动
  */
void DJI_Motor_Init(dji_motor_object_t *motor, CAN_HandleTypeDef *hcan, uint32_t rx_id)
{
    if (motor == NULL) return;





    motor->hcan = hcan;
    motor->rx_id = rx_id;

    // 清零数据
    motor->measure.ecd = 0;
    motor->measure.speed_rpm = 0;
    motor->measure.given_current = 0;
    motor->measure.temperate = 0;

    // 向 BSP 层注册：当收到 rx_id 时，调用 _dji_motor_decode_callback，并将 motor 指针传回去
    BSP_CAN_RegisterRxCallback(hcan, rx_id,CAN_ID_STD, _dji_motor_decode_callback, motor);
}





#define GM6020_ANGLE_TO_RAD (2.0f * M_PI / 8192.0f)
#define GM6020_RAD_TO_ANGLE (8192.0f / (2.0f * M_PI))


/*
void GM6020_PV_init(dji_motor_object_t *motor, uint32_t motor_ID, CAN_HandleTypeDef *hcan, int para_num, ...)
{
    if (motor == NULL || motor->data == NULL) return;
    GM6020_data_t *d = (GM6020_data_t *)motor->data;

    /* 1. 全清零，防止脏数据导致电机疯转 #1#
    memset(d, 0, sizeof(GM6020_data_t));

    /* 2. 基础配置 #1#
    motor->rx_id = (uint32_t)(motor_ID & 0x7FFU);
    if (hcan != NULL) motor->hcan = hcan;

    /* 3. 设定默认安全值 (如果外部没传参) #1#
    d->_v_limit = 320.0f;        // GM6020 额定转速
    d->_out_max = 5000.0f;       // 初始安全电压限幅
    d->_i_p_max = 100.0f;        // 位置环积分限幅
    d->_i_v_max = 5000.0f;       // 速度环积分限幅
    d->_d_filter_alpha = 1.0f;   // 默认关闭滤波

    /* 4. 补全参数映射 (按照 8 个参数设计) #1#
    if (para_num > 0) {
        va_list ap;
        va_start(ap, para_num);

        // 顺序：P_Kp, P_Ki, V_Kp, V_Ki, V_Kd, Out_Max, V_Limit, Alpha
        if (para_num >= 1) d->_kp_p = (float)va_arg(ap, double);
        if (para_num >= 2) d->_ki_p = (float)va_arg(ap, double);

        if (para_num >= 3) d->_kp_v = (float)va_arg(ap, double);
        if (para_num >= 4) d->_ki_v = (float)va_arg(ap, double);
        if (para_num >= 5) d->_kd_v = (float)va_arg(ap, double);

        if (para_num >= 6) d->_out_max = (float)va_arg(ap, double);
        if (para_num >= 7) d->_v_limit = (float)va_arg(ap, double);
        if (para_num >= 8) d->_d_filter_alpha = (float)va_arg(ap, double);

        va_end(ap);
    }
    BSP_CAN_RegisterRxCallback(hcan, motor_ID,CAN_ID_STD, _dji_motor_decode_callback, motor);
}*/

/* 反馈解析 */

//
// void GM6020_enable(dji_motor_object_t *motor) {
//     if (motor == NULL || motor->data == NULL) return;
//     ((GM6020_data_t *)motor->data)->enable_flag = 1;
// }

/*void GM6020_disable(dji_motor_object_t *motor) {
    if (motor == NULL || motor->data == NULL) return;
    GM6020_data_t *d = (GM6020_data_t *)motor->data;
    d->enable_flag = 0;
    d->_i_term_p = 0.0f;
    d->_i_term_v = 0.0f;
    d->_out_output = 0;

    // [重要] 失能时将目标位置重置为当前反馈位置，防止下次使能时猛甩
    d->_p_des = (float)motor->measure.ecd * GM6020_ANGLE_TO_RAD;
}*/

// void GM6020_PV_update(dji_motor_object_t *motor) {
//     if (motor == NULL || motor->data == NULL) return;
//     GM6020_data_t *d = (GM6020_data_t *)motor->data;
//
//     if (d->enable_flag == 0) {
//         d->_i_term_p = 0.0f;
//         d->_i_term_v = 0.0f;
//         d->_out_output = 0;
//         // 持续同步目标值
//         d->_p_des = (float)motor->measure.ecd * GM6020_ANGLE_TO_RAD;
//         return;
//     }
//
//     // 1. 位置环计算
//     float current_p_rad = (float)motor->measure.ecd * GM6020_ANGLE_TO_RAD;
//     float p_error = d->_p_des - current_p_rad;
//
//     // 最短路径处理
//     while (p_error > M_PI)  p_error -= 2.0f * M_PI;
//     while (p_error < -M_PI) p_error += 2.0f * M_PI;
//
//     d->_i_term_p += d->_ki_p * p_error;
//     if (d->_i_term_p > d->_i_p_max) d->_i_term_p = d->_i_p_max;
//     if (d->_i_term_p < -d->_i_p_max) d->_i_term_p = -d->_i_p_max;
//
//     d->_v_des_internal = d->_kp_p * p_error + d->_i_term_p;
//
//     // 速度限制
//     if (d->_v_des_internal > d->_v_limit) d->_v_des_internal = d->_v_limit;
//     if (d->_v_des_internal < -d->_v_limit) d->_v_des_internal = -d->_v_limit;
//
//     // 2. 速度环计算
//     float v_error = d->_v_des_internal - (float)motor->measure.speed_rpm;
//     float v_p_out = d->_kp_v * v_error;
//
//     d->_i_term_v += d->_ki_v * v_error;
//     if (d->_i_term_v > d->_i_v_max) d->_i_term_v = d->_i_v_max;
//     if (d->_i_term_v < -d->_i_v_max) d->_i_term_v = -d->_i_v_max;
//
//     float d_raw = d->_kd_v * (v_error - (float)d->_last_v_error);
//     float v_d_out = d->_d_filter_alpha * d_raw + (1.0f - d->_d_filter_alpha) * d->_last_d_out;
//
//     d->_last_v_error = (int16_t)v_error;
//     d->_last_d_out = v_d_out;
//
//     // 3. 输出限幅
//     float total_out = v_p_out + d->_i_term_v + v_d_out;
//     if (total_out > d->_out_max) total_out = d->_out_max;
//     if (total_out < -d->_out_max) total_out = -d->_out_max;
//
//     d->_out_output = (int16_t)total_out;
//     motor->input_current = (int16_t)total_out;
// }

/*void GM6020_PV_set_target(dji_motor_object_t *motor, float target) {
    if (motor == NULL || motor->data == NULL) return;
     GM6020_data_t *d = (GM6020_data_t*)motor->data;
    d->_p_des = target;


}*/
/**
  * @brief 发送底盘控制指令 (0x200 ID 对应 0x201-0x204)
  */
