#include "rdk_comm_task.h"

#include <string.h>

#include "bsp_usart.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "usart_protocol.h"

#define RDK_STATUS_PERIOD_MS 100U
#define RDK_LINK_TIMEOUT_MS  500U

static volatile uint32_t rdk_rx_count = 0;
static volatile uint32_t rdk_parse_ok_count = 0;
static volatile uint32_t rdk_parse_error_count = 0;
static volatile uint32_t rdk_last_rx_tick = 0;
static volatile uint32_t rdk_last_seq = 0;
static volatile RdkControl_t rdk_latest_control = {0};

static void RdkComm_OnRx(uint8_t *data, uint16_t len)
{
    uint8_t cmd_id = 0;
    uint8_t payload_len = 0;
    uint8_t payload[PROTOCOL_MAX_PAYLOAD_LEN];

    rdk_rx_count++;

    if (Protocol_ParseFrame(data, len, &cmd_id, payload, sizeof(payload), &payload_len) == 0U) {
        rdk_parse_error_count++;
        return;
    }

    if (cmd_id == CMD_ID_RDK_HEARTBEAT && payload_len == sizeof(RdkHeartbeat_t)) {
        RdkHeartbeat_t heartbeat;
        memcpy(&heartbeat, payload, sizeof(heartbeat));
        rdk_last_seq = heartbeat.seq;
        rdk_last_rx_tick = osKernelGetTickCount();
        rdk_parse_ok_count++;
    } else if (cmd_id == CMD_ID_RDK_CONTROL && payload_len == sizeof(RdkControl_t)) {
        RdkControl_t control;
        memcpy(&control, payload, sizeof(control));
        rdk_latest_control = control;
        rdk_last_seq = control.seq;
        rdk_last_rx_tick = osKernelGetTickCount();
        rdk_parse_ok_count++;
    } else {
        rdk_parse_error_count++;
    }
}

uint8_t rdk_comm_is_online(void)
{
    const uint32_t now = osKernelGetTickCount();
    return ((uint32_t)(now - rdk_last_rx_tick) <= RDK_LINK_TIMEOUT_MS) ? 1U : 0U;
}

uint8_t rdk_comm_get_control(RdkControl_t *control)
{
    if (control == NULL || rdk_comm_is_online() == 0U) {
        return 0;
    }

    *control = rdk_latest_control;
    return ((control->flags & RDK_CONTROL_FLAG_ENABLE) != 0U) ? 1U : 0U;
}

void rdk_comm_task(void *argument)
{
    (void)argument;

    static uint8_t tx_buf[96];
    static uint16_t tx_len = 0;

    usart1_register_callback(RdkComm_OnRx);
    usart1_init();

    while (1) {
        const uint32_t now = osKernelGetTickCount();
        Stm32Status_t status = {
            .echoed_seq = rdk_last_seq,
            .uptime_ms = now,
            .rx_count = rdk_rx_count,
            .parse_ok_count = rdk_parse_ok_count,
            .parse_error_count = rdk_parse_error_count,
            .link_online = rdk_comm_is_online(),
        };

        if (Protocol_PackFrame(CMD_ID_STM32_STATUS, &status, (uint8_t)sizeof(status), tx_buf, &tx_len) != 0U) {
            usart_tx_binary(&huart1, tx_buf, tx_len);
        }

        osDelay(RDK_STATUS_PERIOD_MS);
    }
}
