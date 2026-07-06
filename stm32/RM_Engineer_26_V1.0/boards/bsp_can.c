#include "bsp_can.h"
#include <string.h>

/* 管理所有注册的回调函数 */
static can_rx_hook_t rx_hooks[CAN_MAX_RX_CALLBACKS];

static uint8_t hook_count = 0;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
  * @brief 初始化CAN过滤器
  * @note  配置为允许通过所有标准帧和扩展帧
  */
void BSP_CAN_Init(void)
{
    CAN_FilterTypeDef can_filter_st;

    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;

    // Mask 和 Id 全部设置为 0，表示不关心 ID 的任何位，即接收所有
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;

    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    // CAN2 配置 (FilterBank 从 14 开始)
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    // 清空回调表
    memset(rx_hooks, 0, sizeof(rx_hooks));
    hook_count = 0;
}

void can_bsp_init(void)
{
    BSP_CAN_Init();
}

/**
  * @brief 注册接收回调函数 (支持扩展帧)
  * @param hcan: CAN句柄
  * @param id: CAN ID
  * @param id_type: CAN_ID_STD 或 CAN_ID_EXT
  * @param cb: 回调函数
  * @param obj: 设备对象指针
  */
int8_t BSP_CAN_RegisterRxCallback(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t id_type, can_rx_callback_t cb, void *obj)
{
    if (hook_count >= CAN_MAX_RX_CALLBACKS) return -1;

    rx_hooks[hook_count].hcan = hcan;
    rx_hooks[hook_count].id = id;          // 存储 ID
    rx_hooks[hook_count].id_type = id_type;// 存储 帧类型
    rx_hooks[hook_count].callback = cb;
    rx_hooks[hook_count].device_ptr = obj;
    rx_hooks[hook_count].is_registered = 1;
    hook_count++;

    return 0;
}

/**
  * @brief 通用CAN发送函数 (支持扩展帧)
  * @param id_type: CAN_ID_STD 或 CAN_ID_EXT
  */
uint8_t BSP_CAN_SendMsg(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t id_type, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t send_mail_box;

    if (hcan == NULL || data == NULL) {
        return 1;
    }

    if (len > 8) {
        len = 8;
    }

    // 1. 设置帧类型 (标准/扩展)
    tx_header.IDE = id_type;

    // 2. 根据类型填入 ID
    if (id_type == CAN_ID_STD) {
        tx_header.StdId = id;
        tx_header.ExtId = 0; // 这是一个好习惯，虽然不是必须
    } else {
        tx_header.ExtId = id;
        tx_header.StdId = 0;
    }

    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;

    // 默认不使用时间戳
    tx_header.TransmitGlobalTime = DISABLE;

    // 3. 发送
    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &send_mail_box) != HAL_OK)
    {
        return 1; // Error
    }
    return 0; // OK
}

uint8_t CAN1_SendData(uint8_t *msgData, uint8_t len)
{
    return BSP_CAN_SendMsg(&hcan1, 0x666, CAN_ID_EXT, msgData, len);
}

uint8_t CAN1_Extended_SendData(CAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *msgData, uint32_t len)
{
    if (len > 8U) {
        len = 8U;
    }
    return BSP_CAN_SendMsg(hfdcan, id, CAN_ID_EXT, msgData, (uint8_t)len);
}

/**
  * @brief 统一的接收中断回调
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint32_t recv_id;

    // 获取数据
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    // 1. 判断接收到的 ID 是标准帧还是扩展帧
    if (rx_header.IDE == CAN_ID_STD) {
        recv_id = rx_header.StdId;
    } else {
        recv_id = rx_header.ExtId;
    }



    // 2. 遍历注册表
    for (uint8_t i = 0; i < hook_count; i++)
    {
        // 匹配条件：已注册 + 句柄相同 + ID类型相同 + ID数值相同
        if (rx_hooks[i].is_registered &&
            rx_hooks[i].hcan == hcan &&
            rx_hooks[i].id_type == rx_header.IDE &&  // 关键：匹配帧类型
            rx_hooks[i].id == recv_id)               // 关键：匹配ID
        {
            if (rx_hooks[i].callback != NULL)
            {
                rx_hooks[i].callback(rx_hooks[i].device_ptr, rx_data);
            }
        }
    }
}
