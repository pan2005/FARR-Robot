#ifndef RDK_COMM_TASK_H
#define RDK_COMM_TASK_H

#include <stdint.h>

#include "usart_protocol.h"

void rdk_comm_task(void *argument);
uint8_t rdk_comm_get_control(RdkControl_t *control);
uint8_t rdk_comm_is_online(void);

#endif
