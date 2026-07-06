/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң¿؆÷´¦À�¿؆÷ʇͨ¹ýÀˆSBUSµĐ­ҩ´«ʤ£¬ÀûӃDMA´«ʤ·½ʽ½ڔ¼CPU
  *             ׊Դ£¬ÀûӃ´®¿ڿՏЖжπ´À­ư´¦À�ʽ£¬ͬʱ̡¹©һЩµ􏟖؆􄍁£¬´®¿ڊ  *             µķ½ʽ±£֤Ȉ²導µĎȶ¨Д¡£
  * @note       ¸ÈΎ񊇍¨¹ý´®¿ږжφ��¬²»ʇfreeRTOSȎα
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"

#include "main.h"

#include "bsp_usart.h"
#include "cmsis_os2.h"
#include "string.h"

//#include "detect_task.h"




#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart3_rx;



static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң¿؆÷Эҩ½⎶
  * @param[in]      sbus_buf: ԭɺʽ¾ݖ¸ի
  * @param[out]     rc_ctrl: ң¿؆÷ʽ¾ݖ¸
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data

RC_ctrl_t rc_ctrl;
const RC_ctrl_t *local_rc_ctrl;

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];


/**
  * @brief          remote control init
  * @param[in]      none
  *
  * @retval         none
  */
/**
  * @brief          ң¿؆÷³õʼ»¯
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          »񈡒£¿؆÷ʽ¾ݖ¸ի
  * @param[in]      none
  * @retval         ң
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}


uint8_t RC_data_is_error(void)
{

    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}


void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */


            __HAL_DMA_DISABLE(&hdma_usart3_rx);


            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;


            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;


            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;


            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);

                //detect_hook(DBUS_TOE);
                //sbus_to_usart1(sbus_rx_buf[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA

            __HAL_DMA_DISABLE(&hdma_usart3_rx);


            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght

            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0

            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //enable DMA

            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {

                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);

                //detect_hook(DBUS_TOE);
                //sbus_to_usart1(sbus_rx_buf[1]);
            }
        }
    }

}


static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң¿؆÷Эҩ½⎶
  * @param[in]      sbus_buf: ԭɺʽ¾ݖ¸ի
  * @param[out]     rc_ctrl: ң¿؆÷ʽ¾ݖ¸
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
/**
  * @brief          ͨ¹ýusart1·¢ˍsbusʽ¾ݬԚusart3_IRQHandleµ÷Ӄ
  * @param[in]      sbus: sbusʽ¾ݬ 18ז½ڊ  * @retval         none
  */
void sbus_to_usart1(uint8_t *sbus)
{
    static uint8_t usart_tx_buf[20];
    static uint8_t i =0;
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, sbus, 18);
    for(i = 0, usart_tx_buf[19] = 0; i < 19; i++)
    {
        usart_tx_buf[19] += usart_tx_buf[i];
    }
  //  usart_tx_dma_enable(&huart1,usart_tx_buf, 20);
}

void remote_control_task(void * arugument) {
    remote_control_init();
    HAL_Delay(100);
    local_rc_ctrl = get_remote_control_point();

    while (1) {
        osDelay(100);
    }
}