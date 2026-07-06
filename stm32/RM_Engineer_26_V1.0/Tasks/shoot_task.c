//
// Created by PanZhaoHui on 25-12-13.
//

#include "shoot_task.h"
#include "chassis_control_task.h"

#include "remote_control.h"

#include "CAN_receive.h"
#include "cmsis_os2.h"
#include "LKMF9025.h"


#include "pid.h"
#include "DJI_Motor.h"
#include "LKMF9025.h"

extern CAN_HandleTypeDef hcan1;



extern dji_motor_object_t YAW_Motor6020;
void shoot_task() {

    int16_t I = 0;
    int16_t velocity = 0;
    osDelay(100);
    while (1) {

    }

}
