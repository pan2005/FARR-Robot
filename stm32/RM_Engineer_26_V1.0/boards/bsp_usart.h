#ifndef BSP_USART_H
#define BSP_USART_H

#include "main.h"
#include <stdio.h>

// 单个缓冲区大小
#define USART1_RX_BUF_SIZE 128
// 缓冲区数量 (双缓冲 = 2)
#define USART1_BUF_COUNT   2

// 回调函数类型
typedef void (*usart_rx_callback_t)(uint8_t *data, uint16_t len);

// --- 接口 ---
void usart1_init(void);
void usart1_register_callback(usart_rx_callback_t callback);

// --- 发送 ---
void usart_printf(UART_HandleTypeDef* huart, const char *fmt, ...);
void usart_tx_binary(UART_HandleTypeDef* huart, uint8_t *data, uint16_t len);

#endif // BSP_USART_H