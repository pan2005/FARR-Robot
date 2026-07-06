#include "LKMF9025.h"
#include "bsp_buzzer.h" // 假设有蜂鸣器
#include "daemon.h"     // 假设有守护进程
#include "bsp_can.h"
#include "cmsis_os2.h"

/**
 * @brief 私有：数据解析回调函数
 * 该函数会被 bsp_can 在中断中调用
 */
LK_Motor_t YAW_Motor;
extern CAN_HandleTypeDef hcan1;
static void _lk_motor_decode_callback(void *device_ptr, uint8_t *data)
{
    LK_Motor_t *motor = (LK_Motor_t *)device_ptr;

    // 喂狗 (如果有 Daemon)
    // if(motor->daemon) {
    //     DaemonReload(motor->daemon);
    // }

    // 如果需要蜂鸣器关闭
    // buzzer_off();

    // 解析数据 (根据你提供的 switch case 逻辑)
    // 注意：data[0] 是命令字返回
    switch (data[0]) {
        case Speed_Control_Mode: // 假设返回的也是这个指令头
            motor->tempreture = data[1];
            motor->iq = (int16_t)((data[3] << 8) | data[2]);
            motor->speed = (int16_t)((data[5] << 8) | data[4]);
            motor->encoder = (int16_t)((data[7] << 8) | data[6]);
        case Multi_Loop_Control_Mode1:
            motor->tempreture = data[1];
            motor->iq = (int16_t)((data[3] << 8) | data[2]);
            motor->speed = (int16_t)((data[5] << 8) | data[4]);
            motor->encoder = (int16_t)((data[7] << 8) | data[6]);
            break;
        default:
            break;
    }
}

void LK_Motor_Init(LK_Motor_t *motor, CAN_HandleTypeDef *hcan, uint8_t id)
{
    if (motor == NULL) return;

    motor->id = id;
    motor->hcan = hcan;

    // 初始化数据
    motor->speed = 0;
    motor->iq = 0;
    motor->encoder = 0;
    motor->tempreture = 0;

    // 注册守护进程 (示例)
    /*
    Daemon_Init_Config_s daemon_config = {
        .callback = LKMotorLostCallback, // 需自行定义丢失回调
        .owner_id = (void *)motor,
        .reload_count = 100,
    };
    motor->daemon = DaemonRegister(&daemon_config);
    */

    // 关键：注册 CAN 回调
    // 领控电机反馈 ID 通常是 0x140 + ID (需确认手册)
    uint32_t rx_id = LK9025_Single_StdId + id - 1;

    BSP_CAN_RegisterRxCallback(hcan, rx_id,CAN_ID_STD, _lk_motor_decode_callback, motor);
}

void LK_Motor_SpeedControl(LK_Motor_t *motor, int32_t speed)
{
    uint8_t tx_data[8] = {0};

    // 根据协议填充数据
    // 假设: Cmd, 0, 0, 0, DataL, DataML, DataMH, DataH
    tx_data[0] = Speed_Control_Mode; // 你的代码中是 Speed_Control_Mode
    tx_data[4] = (speed >> 0) & 0xFF;
    tx_data[5] = (speed >> 8) & 0xFF;
    tx_data[6] = (speed >> 16) & 0xFF;
    tx_data[7] = (speed >> 24) & 0xFF;

    // 发送 ID: 0x140 + ID
    uint32_t tx_id = LK9025_Single_StdId + motor->id;

    BSP_CAN_SendMsg(motor->hcan, tx_id, CAN_ID_STD,tx_data, 8);
}

void LK_Motor_SingleLoop_AngleControl1(LK_Motor_t *motor ,uint16_t angleControl,uint8_t Spin_direction) {
    uint8_t tx_data[8] = {0};

    // 根据协议填充数据
    // 假设: Cmd, 0, 0, 0, DataL, DataML, DataMH, DataH
    tx_data[0] = Single_Loop_Control_Mode1; // 你的代码中是 Speed_Control_Mode
    tx_data[1] = Spin_direction;
    tx_data[4] = (angleControl >> 0) & 0xFF;
    tx_data[5] = (angleControl >> 8) & 0xFF;


    // 发送 ID: 0x140 + ID
    uint32_t tx_id = LK9025_Single_StdId + motor->id;

    BSP_CAN_SendMsg(motor->hcan, tx_id,CAN_ID_STD, tx_data, 8);

}

void LK_Motor_SingleLoop_AngleControl2(LK_Motor_t *motor ,uint16_t angleControl,uint8_t Spin_direction,uint16_t maxSpeed) {
    uint8_t tx_data[8] = {0};

    // 根据协议填充数据
    // 假设: Cmd, 0, 0, 0, DataL, DataML, DataMH, DataH
    tx_data[0] = Single_Loop_Control_Mode2; // 你的代码中是 Speed_Control_Mode
    tx_data[1] = Spin_direction;
    tx_data[2] = (maxSpeed >> 0) & 0xFF;
    tx_data[3] = (maxSpeed >> 8) & 0xFF;
    tx_data[4] = (angleControl >> 0) & 0xFF;
    tx_data[5] = (angleControl >> 8) & 0xFF;


    // 发送 ID: 0x140 + ID
    uint32_t tx_id = LK9025_Single_StdId + motor->id;

    BSP_CAN_SendMsg(motor->hcan, tx_id,CAN_ID_STD, tx_data, 8);

}

void LK_Motor_Increment_AngleControl1(LK_Motor_t *motor ,int32_t angleControl){
    uint8_t tx_data[8] = {0};
    tx_data[0] = Increment_Control_Mode1;
    tx_data[4] = (angleControl >> 0) & 0xFF;
    tx_data[5] = (angleControl >> 8) & 0xFF;
    tx_data[6] = (angleControl >> 16) & 0xFF;
    tx_data[7] = (angleControl >> 24) & 0xFF;
    BSP_CAN_SendMsg(motor->hcan, LK9025_Single_StdId + motor->id, CAN_ID_STD,tx_data, 8);
}
void LK_Motor_Increment_AngleControl2(LK_Motor_t *motor ,int32_t angleControl,uint16_t maxSpeed){
    uint8_t tx_data[8] = {0};
    tx_data[0] = Increment_Control_Mode2;
    tx_data[1] = (maxSpeed >> 0) & 0xFF;
    tx_data[2] = (maxSpeed >> 8) & 0xFF;
    tx_data[4] = (angleControl >> 0) & 0xFF;
    tx_data[5] = (angleControl >> 8) & 0xFF;
    tx_data[6] = (angleControl >> 16) & 0xFF;
    tx_data[7] = (angleControl >> 24) & 0xFF;
    BSP_CAN_SendMsg(motor->hcan, LK9025_Single_StdId + motor->id,CAN_ID_STD, tx_data, 8);
}

void LK_Motor_testtask(void * pvParameters) {
    LK_Motor_Init(&YAW_Motor, &hcan1, 1);
    static uint16_t counter = 0;
    while (1) {
        if (counter < 1000) {
            LK_Motor_SingleLoop_AngleControl2(&YAW_Motor,18000,0,200);
            counter ++;
        }
        else if (counter < 2000) {
            LK_Motor_SingleLoop_AngleControl2(&YAW_Motor,0,0,200);
            counter ++;
        }
        else {
            counter = 0;
        }
        osDelay(6);


    }
}