//
// Created by Admin on 2025/11/4.
//

#include "bsp_rc.h"



extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver request
    //Ê¹ÄÜDMA´®¿Ú½ÓÊÕ
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //Ê¹ÄÜ¿ÕÏÐÖÐ¶Ï
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //Ê§Ð§DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //ÄÚ´æ»º³åÇø1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //ÄÚ´æ»º³åÇø2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //Êý¾Ý³¤¶È
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //Ê¹ÄÜË«»º³åÇø
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //Ê¹ÄÜDMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);


}
void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart3);
}
void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);

}