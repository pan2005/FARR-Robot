//
// Created by Admin on 2026/1/12.
//

#include "DM_Motor.h"
#include <stdio.h>

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "bsp_can.h"
#include "math.h"
#include "cmsis_os.h"

struct motor_device;

static void DM_get_measure(void * device, uint8_t *data);


void DM_MIT_init(DM_Motor_obj_t *motor, uint32_t motor_ID, CAN_HandleTypeDef *hcan, int para_num, ...)
{
    if (motor == NULL) return;

    /* 确保 motor->motor_data 已分配（调用者负责） */
  //  if (motor->motor_data == NULL) return;

    struct DM_MIT_data *d = &motor->motor_data;

    /* 清零结构 */
    memset(d, 0, sizeof(*d));

    /* 存 ID（保留 11 位安全掩码） */
    motor->motor_id = (uint32_t)(motor_ID & 0x7FFU);

    /* 如果传入 CAN 句柄，复制到 device 中以便后续使用；否则 caller 应保证 motor->motor_can_handle 有效 */
    if (hcan != NULL) {
        motor->motor_can_handle = hcan;
    }


    /* 处理可变参数 */
    if (para_num > 0) {
        va_list ap;
        va_start(ap, para_num);
        if (para_num >= 1) {
            const double v = va_arg(ap, double); d->_kp = (float)v;
        }
        if (para_num >= 2) {
            const double v = va_arg(ap, double); d->_kd = (float)v;
        }
        if (para_num >= 3) {
            const double v = va_arg(ap, double); d->P_MAX = (float)v;
        }
        if (para_num >= 4) {
            const double v = va_arg(ap, double); d->V_MAX = (float)v;
        }
        if (para_num >= 5) {
            const double v = va_arg(ap, double); d->T_MAX = (float)v;
        }

        va_end(ap);
    }

    BSP_CAN_RegisterRxCallback(hcan, 0x00,CAN_ID_STD, DM_get_measure, motor);

    {
        uint32_t send_mail_box;
        CAN_TxHeaderTypeDef enable_tx_message;
        uint8_t enable_can_send_data[8];

        memset(&enable_tx_message, 0, sizeof(enable_tx_message));
        enable_tx_message.StdId = (uint32_t)(motor->motor_id & 0x7FFU);
        enable_tx_message.IDE = CAN_ID_STD;
        enable_tx_message.RTR = CAN_RTR_DATA;
        enable_tx_message.DLC = 0x08;

        for (int i = 0; i < 7; ++i) enable_can_send_data[i] = 0xFF;
        enable_can_send_data[7] = 0xFC; // 初始化使能帧标识

        HAL_CAN_AddTxMessage(motor->motor_can_handle, &enable_tx_message, enable_can_send_data, &send_mail_box);
    }
}


/* 解析 CAN 8 字节反馈帧：
   D[0]: ID (低4位，可选) | ERR (高4位)
   D[1]: POS[15:8]
   D[2]: POS[7:0]
   D[3]: VEL[11:4]
   D[4]: VEL[3:0]
   D[5]: T_MOS
   D[6]: T[11:8] (低4位)
   D[7]: T[7:0]
*/
static void DM_get_measure(void * device, uint8_t *data)
{
    // if (motor == NULL || motor->motor_data == NULL || data == NULL) {
    //     return;
    // }

    DM_Motor_obj_t * motor = (DM_Motor_obj_t *)(device);
    const uint8_t rx_motor_id = data[0] & 0x0F;

    if (rx_motor_id != (uint8_t)(motor->motor_id & 0x0F)) {
        return;
    }

    struct DM_MIT_data *d = &motor->motor_data;

    /* ERR：高 4 位 */
    d->ERR = (int8_t)((data[0] >> 4) & 0x0F);

    /* ID：低 4 位，不需要解析 */
    /* uint8_t id = data[0] & 0x0F; */

    /* POS：16 位有符号 */
    d->POS = (int16_t)((uint16_t)data[1] << 8 | (uint16_t)data[2]);

    /* VEL：12 位有符号，拼接后右移 4 位 */
    {
        const uint16_t vel_raw = (uint16_t)data[3] << 8 | (uint16_t)data[4];
        /* 12 位有符号扩展 */
        int16_t vel12 = (int16_t)(vel_raw >> 4);
        /* 如果最高位（位11）为 1，需要符号扩展到 16 位 */
        if (vel12 & (1 << 11)) {
            vel12 |= 0xF000; /* 保留上位 4 位为 1 */
        }
        d->VEL = vel12;
    }

    /* T：12 位有符号，高 4 位在 data[4] 的低 4 位，低 8 位在 data[5] */
    {
        uint16_t t_raw = ((uint16_t)(data[4] & 0x0F) << 8) | (uint16_t)data[5];
        int16_t t12 = (int16_t)t_raw;
        if (t12 & (1 << 11)) {
            t12 |= 0xF000;
        }
        d->T = t12;
    }

    /* T_MOS：Data[6]（单位：℃）*/
    d->TEMP_MOS = (int8_t)data[6];

    /* T_Rotor：Data[7]（单位：℃）*/
    d->TEMP_Rotor = (int8_t)data[7];
    d->last_rx_tick = osKernelGetTickCount();
}

void DM_update(struct motor_device *motor)
{
    // 此处可添加状态更新逻辑（如滤波等），目前为空
}

/* MIT 模式控制帧打包并发送（原DM_MIT_enable，重命名为更贴合功能的名称） */
void DM_MIT_send_ctrl_cmd(DM_Motor_obj_t *motor)
{
    if (motor == NULL) return;

    struct DM_MIT_data *d = &(motor->motor_data);

    /* p_des: 16-bit unsigned */
    uint16_t pdes = (uint16_t)((d->_p_des + d->P_MAX) / (d->P_MAX * 2.0f) * 65535.0f);
    if (pdes < 0) pdes = 0;
    if (pdes > 0xFFFF) pdes = 0xFFFF;

    /* v_des: 12-bit unsigned */
    uint16_t vdes = (int16_t)((d->_v_des + d->V_MAX) / (d->V_MAX * 2.0f) * 4095.0f);
    if (vdes < 0) vdes = 0;
    if (vdes > 0x0FFF) vdes = 0x0FFF;

    /* Kp: 12-bit unsigned 文档范围[0,500] */
    uint16_t kp = (int16_t)(d->_kp / 500.0f * 4095.0f);
    if (kp < 0) kp = 0;
    if (kp > 0x0FFF) kp = 0x0FFF;

    /* Kd: 12-bit unsigned 文档范围[0,5] */
    uint16_t kd = (int16_t)(d->_kd / 5.0f * 4095.0f);
    if (kd < 0) kd = 0;
    if (kd > 0x0FFF) kd = 0x0FFF;

    /* t_ff: 12-bit unsigned (0..4095) */
    uint16_t t_ff = (int16_t)((d->_t_ff + d->T_MAX) / (d->T_MAX * 2.0f) * 4095.0f);
    if (t_ff < 0) t_ff = 0;
    if (t_ff > 0x0FFF) t_ff = 0x0FFF;

    /* 组帧 */
    CAN_TxHeaderTypeDef tx_msg;
    uint8_t tx_data[8];
    uint32_t send_mail_box = 0;

    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.StdId = (uint32_t)(motor->motor_id & 0x7FFU); /* 帧 ID 等于设定 CAN ID */
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 8;

    tx_data[0] = (uint8_t)(pdes >> 8);
    tx_data[1] = (uint8_t)(pdes & 0xFF);

    /* v_des 12 位：高 8 位放在 D[2]（即 vdes >> 4），低 4 位放在 D[3] 高 4 位 */
    tx_data[2] = (uint8_t)(vdes >> 4);
    tx_data[3] = (uint8_t)((vdes & 0x0F) << 4);

    /* Kp 12 位：高 4 位拼到 D[3] 低 4 位，低 8 位放 D[4] */
    tx_data[3] |= (uint8_t)((kp >> 8) & 0x0F);
    tx_data[4] = (uint8_t)(kp & 0xFF);

    /* Kd 12 位：高 8 位放 D[5]（即 Kd >> 4），低 4 位放 D[6] 高 4 位 */
    tx_data[5] = (uint8_t)(kd >> 4);
    tx_data[6] = (uint8_t)((kd & 0x0F) << 4);

    /* t_ff 12 位：高 4 位拼到 D[6] 低 4 位，低 8 位放 D[7] */
    tx_data[6] |= (uint8_t)((t_ff >> 8) & 0x0F);
    tx_data[7] = (uint8_t)(t_ff & 0xFF);

    /* 发送 */
    HAL_CAN_AddTxMessage(motor->motor_can_handle, &tx_msg, tx_data, &send_mail_box);
}

/* 电机失能指令帧发送（原DM_disable */
void DM_MIT_send_disable_cmd(DM_Motor_obj_t *motor)
{
    // 空指针校验，符合代码中一贯的安全处理风格
    if (motor == NULL) return;

    /* 组帧：失能帧数据段为 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFD */
    CAN_TxHeaderTypeDef tx_msg;
    uint8_t tx_data[8];
    uint32_t send_mail_box = 0;

    // 初始化CAN发送头，与控制指令帧保持一致的初始化方式
    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.StdId = (uint32_t)(motor->motor_id & 0x7FFU); /* 帧ID等于电机ID，保留11位安全掩码 */
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 8; /* 数据长度固定为8字节 */

    // 设置失能帧的数据段：前7字节为0xFF，第8字节为0xFD
    for (int i = 0; i < 7; ++i) {
        tx_data[i] = 0xFF;
    }
    tx_data[7] = 0xFD;

    /* 发送CAN帧，复用HAL库函数，与控制指令帧逻辑一致 */
    HAL_CAN_AddTxMessage(motor->motor_can_handle, &tx_msg, tx_data, &send_mail_box);
}

// 使能帧的发送函数
void DM_MIT_send_enable_cmd(DM_Motor_obj_t *motor)
{
    if (motor == NULL) return;

    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef enable_tx_message;
    uint8_t enable_can_send_data[8];

    memset(&enable_tx_message, 0, sizeof(enable_tx_message));
    enable_tx_message.StdId = (uint32_t)(motor->motor_id & 0x7FFU);
    enable_tx_message.IDE = CAN_ID_STD;
    enable_tx_message.RTR = CAN_RTR_DATA;
    enable_tx_message.DLC = 0x08;

    for (int i = 0; i < 7; ++i) enable_can_send_data[i] = 0xFF;
    enable_can_send_data[7] = 0xFC; // 初始化使能帧标识

    HAL_CAN_AddTxMessage(motor->motor_can_handle, &enable_tx_message, enable_can_send_data, &send_mail_box);
}

void DM_MIT_send_save_zero_cmd(DM_Motor_obj_t *motor)
{
    if (motor == NULL) return;

    CAN_TxHeaderTypeDef tx_msg;
    uint8_t tx_data[8];
    uint32_t send_mail_box = 0;

    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.StdId = (uint32_t)(motor->motor_id & 0x7FFU);
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 8;

    for (int i = 0; i < 7; ++i) {
        tx_data[i] = 0xFF;
    }
    tx_data[7] = 0xFE;

    HAL_CAN_AddTxMessage(motor->motor_can_handle, &tx_msg, tx_data, &send_mail_box);
}

void DM_MIT_set_target(DM_Motor_obj_t *motor, const int para_num, ...) {
    if (motor == NULL) return;

    struct DM_MIT_data *d = &motor->motor_data;

    va_list ap;
    va_start(ap, para_num);

    if (para_num >= 1) {
        const double v = va_arg(ap, double); d->_p_des = (float)v;
    }
    if (para_num >= 2) {
        const double v = va_arg(ap, double); d->_v_des = (float)v;
    }
    if (para_num >= 3) {
        const double v = va_arg(ap, double); d->_t_ff = (float)v;
    }

    va_end(ap);
}

void DM_MIT_get_status(DM_Motor_obj_t *motor, const char* which_status, void* status_data) {
    if (motor == NULL || which_status == NULL || status_data == NULL) return;

    struct DM_MIT_data *d = &motor->motor_data;

    if (strcmp(which_status, "ERR") == 0) {
        *(int8_t *)status_data = d->ERR;
    } else if (strcmp(which_status, "POS") == 0) {
        *(float *) status_data = (float) d->POS / 65535.0f * 2 * d->P_MAX;
        if (*(float *) status_data > 0.0f) {
            *(float *) status_data -= d->P_MAX;
        } else {
            *(float *) status_data += d->P_MAX;
        }
    } else if (strcmp(which_status, "VEL") == 0) {
        *(float *)status_data = (float)d->VEL / 4096.0f * 2 * d->V_MAX;
        if (*(float *)status_data > 0.0f) {
            *(float *)status_data -= d->V_MAX;
        }else if (*(float *)status_data <= 0.0f) {
            *(float *)status_data += d->V_MAX;
        }
    } else if (strcmp(which_status, "T") == 0) {
        *(float *)status_data = (float)d->T / 4096.0f * 2 * d->T_MAX;
        if (*(float *)status_data > 0.0f) {
            *(float *)status_data -= d->T_MAX;
        }else if (*(float *)status_data <= 0.0f) {
            *(float *)status_data += d->T_MAX;
        }
    } else if (strcmp(which_status, "TEMP_MOS") == 0) {
        *(int8_t *)status_data = d->TEMP_MOS;
    } else if (strcmp(which_status, "TEMP_Rotor") == 0) {
        *(int8_t *)status_data = d->TEMP_Rotor;
    } else if (strcmp(which_status, "P_MAX") == 0) {
        *(float *)status_data = d->P_MAX;
    } else if (strcmp(which_status, "V_MAX") == 0) {
        *(float *)status_data = d->V_MAX;
    } else if (strcmp(which_status, "T_MAX") == 0) {
        *(float *)status_data = d->T_MAX;
    } else if (strcmp(which_status, "Kp") == 0) {
        *(float *)status_data = d->_kp;
    } else if (strcmp(which_status, "Kd") == 0) {
        *(float *)status_data = d->_kd;
    } else if (strcmp(which_status, "p_des") == 0) {
        *(float *)status_data = d->_p_des;
    } else if (strcmp(which_status, "v_des") == 0) {
        *(float *)status_data = d->_v_des;
    } else if (strcmp(which_status, "tff") == 0) {
        *(float *)status_data = d->_t_ff;
    }

}

void DM_MIT_set_para(DM_Motor_obj_t *motor, const char* which_para, void* para_data) {
    if (motor == NULL  || which_para == NULL || para_data == NULL) return;

    struct DM_MIT_data *d = &motor->motor_data;

    if (strcmp(which_para, "Kp") == 0) {
        d->_kp = *(float *)para_data;
        if (d->_kp < 0.0f) d->_kp = 0.0f;
        if (d->_kp > 500.0f) d->_kp = 500.0f;
    } else if (strcmp(which_para, "Kd") == 0) {
        d->_kd = *(float *)para_data;
        if (d->_kd < 0.0f) d->_kd = 0.0f;
        if (d->_kd > 5.0f) d->_kd = 5.0f;
    } else if (strcmp(which_para, "P_MAX") == 0) {
        d->P_MAX = *(float *)para_data;
    } else if (strcmp(which_para, "V_MAX") == 0) {
        d->V_MAX = *(float *)para_data;
    } else if (strcmp(which_para, "T_MAX") == 0) {
        d->T_MAX = *(float *)para_data;
    }
}


/* 电机清除错误指令帧发送 */
void DM_MIT_clear_error(DM_Motor_obj_t *motor)
{
    if (motor == NULL) return;

    CAN_TxHeaderTypeDef tx_msg;
    uint8_t tx_data[8];
    uint32_t send_mail_box = 0;

    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.StdId = (uint32_t)(motor->motor_id & 0x7FFU);
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 8;

    for (int i = 0; i < 7; ++i) {
        tx_data[i] = 0xFF;
    }
    // 0xFB 为达妙官方协议中的“清除错误”控制字
    tx_data[7] = 0xFB;

    HAL_CAN_AddTxMessage(motor->motor_can_handle, &tx_msg, tx_data, &send_mail_box);
}
