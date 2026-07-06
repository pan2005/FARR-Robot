#ifndef USART_PROTOCOL_H
#define USART_PROTOCOL_H

#include <stdint.h>

#define FRAME_HEADER    0xA5
#define FRAME_TAIL      0x5A
#define PROT_MIN_LEN    5

#define CMD_ID_CHASSIS_INFO  0x01
#define CMD_ID_RDK_HEARTBEAT 0x10
#define CMD_ID_STM32_STATUS  0x11
#define CMD_ID_RDK_CONTROL   0x12

#define RDK_CONTROL_FLAG_ENABLE 0x01U

#define PROTOCOL_MAX_PAYLOAD_LEN 64

#pragma pack(1)

typedef struct {
    float pitch_position;
    uint8_t shoot_gear;
} GimbalInfo_t;

typedef struct {
    uint32_t seq;
    uint32_t timestamp_ms;
    uint8_t flags;
} RdkHeartbeat_t;

typedef struct {
    uint32_t seq;
    float vx;
    float vy;
    float w;
    float front_arm_delta;
    float rear_arm_delta;
    uint8_t flags;
} RdkControl_t;

typedef struct {
    uint32_t echoed_seq;
    uint32_t uptime_ms;
    uint32_t rx_count;
    uint32_t parse_ok_count;
    uint32_t parse_error_count;
    uint8_t link_online;
} Stm32Status_t;

#pragma pack()

void Protocol_Pack_GimbalInfo(GimbalInfo_t *info, uint8_t *tx_buf, uint16_t *tx_len);
uint8_t Protocol_Parse(uint8_t *rx_buf, uint16_t rx_len, GimbalInfo_t *out_info);

uint8_t Protocol_PackFrame(uint8_t cmd_id, const void *payload, uint8_t payload_len, uint8_t *tx_buf, uint16_t *tx_len);
uint8_t Protocol_ParseFrame(uint8_t *rx_buf, uint16_t rx_len, uint8_t *cmd_id, uint8_t *payload, uint8_t payload_max_len, uint8_t *payload_len);

#endif
