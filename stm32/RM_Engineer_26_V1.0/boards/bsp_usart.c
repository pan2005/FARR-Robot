#include "bsp_usart.h"
#include <stdarg.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

// ==========================================
// 1. 变量定义 (双缓冲)
// ==========================================

// 定义二维数组作为双缓冲: [0][...] 和 [1][...]
static uint8_t usart1_rx_buffers[USART1_BUF_COUNT][USART1_RX_BUF_SIZE];

// 当前正在使用的缓冲区索引 (0 或 1)
static uint8_t rx_buf_idx = 0;

// 回调函数
static usart_rx_callback_t usart1_rx_callback = NULL;


// ==========================================
// 2. 初始化与设置
// ==========================================

void usart1_init(void)
{
    // 确保开启 DMA 标志位
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    // 初始状态：DMA 指向 Buffer[0]
    rx_buf_idx = 0;

    __HAL_DMA_DISABLE(huart1.hdmatx);

    while(huart1.hdmatx->Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(huart1.hdmatx);
    }

    huart1.hdmatx->Instance->PAR = (uint32_t)&(huart1.Instance->DR);
    huart1.hdmatx->Instance->M0AR = (uint32_t)(NULL);
    huart1.hdmatx->Instance->NDTR = 0;

    // 启动接收
    // 注意：这里传入的是 usart1_rx_buffers[0]
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffers[rx_buf_idx], USART1_RX_BUF_SIZE);

    // 关闭半传输中断 (防止还没收完就进中断)
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

void usart1_register_callback(usart_rx_callback_t callback)
{
    usart1_rx_callback = callback;
}


// ==========================================
// 3. 接收中断回调 (乒乓逻辑核心)
// ==========================================

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        // 1. 保存当前处理完的缓冲区索引 (Old)
        uint8_t finished_idx = rx_buf_idx;

        // 2. 计算下一个缓冲区的索引 (Next): 0->1, 1->0
        rx_buf_idx = (rx_buf_idx + 1) % USART1_BUF_COUNT;

        // 3. 【关键】立即重启 DMA，指向“下一个”缓冲区
        //    这样在 CPU 处理旧数据时，DMA 已经准备好接收新数据了，实现无缝衔接。
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffers[rx_buf_idx], USART1_RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

        // 4. 处理刚刚收到的数据 (使用 Old 索引)
        if (usart1_rx_callback != NULL && Size > 0)
        {
            // 可选：为了调试方便，在末尾截断 (非必须)
            if (Size < USART1_RX_BUF_SIZE) {
                usart1_rx_buffers[finished_idx][Size] = 0;
            }

            // 调用应用层回调，传入刚才填满的那个 Buffer
            usart1_rx_callback(usart1_rx_buffers[finished_idx], Size);
        }
    }
}

// 错误处理
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 如果出错，复位索引并重启接收
        rx_buf_idx = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffers[0], USART1_RX_BUF_SIZE);
    }
}


// ==========================================
// 4. 发送部分 (保持原样)
// ==========================================

static void usart_tx_dma_enable(UART_HandleTypeDef* huart, uint8_t *data, uint16_t len)
{
    __HAL_DMA_DISABLE(huart->hdmatx);

    while(huart->hdmatx->Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(huart->hdmatx);
    }

    __HAL_DMA_CLEAR_FLAG(huart->hdmatx, DMA_HISR_TCIF7);

    huart->hdmatx->Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(huart->hdmatx, len);

    __HAL_DMA_ENABLE(huart->hdmatx);
}

void usart_tx_binary(UART_HandleTypeDef* huart, uint8_t *data, uint16_t len) {
    usart_tx_dma_enable(huart, data, len);
}

void usart_printf(UART_HandleTypeDef* huart, const char *fmt, ...) {
    static uint8_t tx_buffer[256];
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);
    len = vsnprintf((char *)tx_buffer, 256, fmt, ap);
    va_end(ap);
    usart_tx_dma_enable(huart, tx_buffer, len);
}