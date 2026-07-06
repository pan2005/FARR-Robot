#include "usart_protocol.h"

#include <string.h>

static uint8_t Calc_Checksum(const uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;

    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }

    return sum;
}

uint8_t Protocol_PackFrame(uint8_t cmd_id, const void *payload, uint8_t payload_len, uint8_t *tx_buf, uint16_t *tx_len)
{
    if (tx_buf == NULL || tx_len == NULL || (payload_len > 0U && payload == NULL)) {
        return 0;
    }

    uint16_t idx = 0;
    tx_buf[idx++] = FRAME_HEADER;
    tx_buf[idx++] = cmd_id;
    tx_buf[idx++] = payload_len;

    if (payload_len > 0U) {
        memcpy(&tx_buf[idx], payload, payload_len);
        idx += payload_len;
    }

    tx_buf[idx] = Calc_Checksum(tx_buf, idx);
    idx++;
    tx_buf[idx++] = FRAME_TAIL;

    *tx_len = idx;
    return 1;
}

uint8_t Protocol_ParseFrame(uint8_t *rx_buf, uint16_t rx_len, uint8_t *cmd_id, uint8_t *payload, uint8_t payload_max_len, uint8_t *payload_len)
{
    if (rx_buf == NULL || cmd_id == NULL || payload_len == NULL || rx_len < PROT_MIN_LEN) {
        return 0;
    }

    if (rx_buf[0] != FRAME_HEADER || rx_buf[rx_len - 1] != FRAME_TAIL) {
        return 0;
    }

    if (rx_buf[rx_len - 2] != Calc_Checksum(rx_buf, rx_len - 2)) {
        return 0;
    }

    const uint8_t data_len = rx_buf[2];
    if ((uint16_t)data_len + PROT_MIN_LEN != rx_len || data_len > payload_max_len) {
        return 0;
    }

    *cmd_id = rx_buf[1];
    *payload_len = data_len;

    if (data_len > 0U && payload != NULL) {
        memcpy(payload, &rx_buf[3], data_len);
    }

    return 1;
}

void Protocol_Pack_GimbalInfo(GimbalInfo_t *info, uint8_t *tx_buf, uint16_t *tx_len)
{
    Protocol_PackFrame(CMD_ID_CHASSIS_INFO, info, (uint8_t)sizeof(GimbalInfo_t), tx_buf, tx_len);
}

uint8_t Protocol_Parse(uint8_t *rx_buf, uint16_t rx_len, GimbalInfo_t *out_info)
{
    uint8_t cmd_id = 0;
    uint8_t data_len = 0;
    uint8_t payload[sizeof(GimbalInfo_t)];

    if (out_info == NULL) {
        return 0;
    }

    if (Protocol_ParseFrame(rx_buf, rx_len, &cmd_id, payload, sizeof(payload), &data_len) == 0U) {
        return 0;
    }

    if (cmd_id == CMD_ID_CHASSIS_INFO && data_len == sizeof(GimbalInfo_t)) {
        memcpy(out_info, payload, sizeof(GimbalInfo_t));
        return 1;
    }

    return 0;
}
