#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"

#define CAN_MAX_RX_CALLBACKS 32

// 定义回调函数类型
typedef void (*can_rx_callback_t)(void *device, uint8_t *data);

// 接收钩子结构体
typedef struct {
    CAN_HandleTypeDef *hcan;
    uint32_t id;            // CAN ID (标准帧或扩展帧ID)
    uint32_t id_type;       // CAN_ID_STD 或 CAN_ID_EXT
    can_rx_callback_t callback;
    void *device_ptr;
    uint8_t is_registered;
} can_rx_hook_t;

// 函数声明
void BSP_CAN_Init(void);
void can_bsp_init(void);

// 注意：增加了 id_type 参数
int8_t BSP_CAN_RegisterRxCallback(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t id_type, can_rx_callback_t cb, void *obj);

// 注意：增加了 id_type 参数
uint8_t BSP_CAN_SendMsg(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t id_type, uint8_t *data, uint8_t len);
uint8_t CAN1_SendData(uint8_t *msgData, uint8_t len);
uint8_t CAN1_Extended_SendData(CAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *msgData, uint32_t len);

#endif
