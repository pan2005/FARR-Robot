#include "can_motor.h"
#include "robot_global.h"
#include "math.h"
#include <string.h>

float Radian_Normalize(float angle) {
    float res = fmodf(angle, TWO_PI);
    if (res > PI) res -= TWO_PI;
    if (res < -PI) res += TWO_PI;
    return res;
}
void Can_Motor_Init(Can_Motor_t *motor, CAN_HandleTypeDef *hcan, uint32_t rx_id,
                    Motor_Decode_Func decode, Motor_Update_Func update, void *priv) {
    if (!motor || !priv) return;

    motor->hcan = hcan;
    motor->rx_id = rx_id;
    motor->decode_func = decode;
    motor->update_func = update;
    motor->priv_data = priv;

    // 假设你的 BSP 层支持这种注册方式
    BSP_CAN_RegisterRxCallback(hcan, rx_id, CAN_ID_STD, decode, motor);
}

// ---------------- M3508 逻辑 ----------------
void M3508_Decode(void *device, uint8_t *data) {
    Can_Motor_t *self = (Can_Motor_t *)device;
    self->measure.last_ecd = self->measure.ecd;
    self->measure.ecd = (uint16_t)(data[0] << 8 | data[1]);
    self->measure.speed_rpm = (int16_t)(data[2] << 8 | data[3]);
    self->measure.given_current = (int16_t)((data[4] << 8) | data[5]);
    self->measure.temperate = data[6];
}

void M3508_Update(Can_Motor_t *self) {
    M3508_Data_t *d = (M3508_Data_t *)self->priv_data;
    self->output_value = (int16_t)PID_Calculate(&d->speed_pid, d->target_speed, (float)self->measure.speed_rpm);
}

// ---------------- M3508 位置环逻辑 ----------------
void M3508_Pos_Decode(void *device, uint8_t *data) {
    Can_Motor_t *self = (Can_Motor_t *)device;
    self->measure.last_ecd = self->measure.ecd;
    self->measure.ecd = (uint16_t)(data[0] << 8 | data[1]);
    self->measure.speed_rpm = (int16_t)(data[2] << 8 | data[3]);
    self->measure.given_current = (int16_t)((data[4] << 8) | data[5]);
    self->measure.temperate = data[6];

    M3508_Pos_Data_t *d = (M3508_Pos_Data_t *)self->priv_data;

    // 过零检测和圈数统计
    int16_t diff = self->measure.ecd - self->measure.last_ecd;
    if (diff < -4096)      d->round_count++;
    else if (diff > 4096)  d->round_count--;

    // 计算总角度 (考虑减速比)
    // 编码器分辨率 8192，输出轴角度 = (总脉冲数 / 8192) * 2π / gear_ratio
    float raw_rad = ((d->round_count * 8192 + self->measure.ecd) / 8192.0f) * TWO_PI / d->gear_ratio;
    //d->total_angle = Radian_Normalize(raw_rad);
    d->current_angle = raw_rad;
}

void M3508_Pos_Update(Can_Motor_t *self) {
    if (!self || !self->priv_data) return;
    M3508_Pos_Data_t *d = (M3508_Pos_Data_t *)self->priv_data;

    // --- 位置环 + 速度环级联控制 ---
    float current_angle = d->current_angle;
    float target_angle = d->target_angle;



    // 3. 位置环 PID -> 生成目标速度
    float v_target = PID_Calculate(&d->pos_pid, target_angle, current_angle);

    // 4. 速度环 PID -> 生成电流输出
    self->output_value = (int16_t)PID_Calculate(&d->speed_pid, v_target, (float)self->measure.speed_rpm);
}

// ---------------- GM6020 逻辑 ----------------
void GM6020_Decode(void * device, uint8_t *data) {
    Can_Motor_t *self = (Can_Motor_t *)device;
    self->measure.last_ecd = self->measure.ecd;
    self->measure.ecd = (uint16_t)(data[0] << 8 | data[1]);
    self->measure.speed_rpm = (int16_t)(data[2] << 8 | data[3]);

    GM6020_Data_t *d = (GM6020_Data_t *)self->priv_data;

    float raw_rad = (self->measure.ecd / 8192.0f) * TWO_PI;
    d->total_angle = Radian_Normalize(raw_rad);
}

void GM6020_Update(Can_Motor_t *self) {



    if (!self || !self->priv_data) return;
    GM6020_Data_t *d = (GM6020_Data_t *)self->priv_data;
    float friction_comp = 0.0f;

    // --- 驱动层只处理编码器闭环 ---
    float current_ecd_angle = d->total_angle; // 当前编码器归一化角度
    float target_ecd_angle = d->target_angle; // 应用层算好的目标角度

    // 1. 计算编码器空间下的最短路径误差
    float ecd_error = Radian_Normalize(target_ecd_angle - current_ecd_angle);

    // 2. 目标重映射（隔离逻辑）：构造一个连续的目标值
    // 使得 PID 计算 (virtual_target - current_ecd_angle) 永远等于 ecd_error
    float virtual_target = current_ecd_angle + ecd_error;

    // 3. 级联 PID 计算
    d->target_v = PID_Calculate(&d->pos_pid, virtual_target, current_ecd_angle);

    float pid_current = PID_Calculate(&d->speed_pid, d->target_v, (float)self->measure.speed_rpm);


    if (d->target_v > 2.0f) {
        friction_comp = 3000;
    }else if (d->target_v < -2.0f){
        friction_comp = -3000;
    }
    float total_current = pid_current + d->feedforward_current + friction_comp;
    // 绝对安全的软件限幅 (GM6020 电流范围 -30000 ~ 30000，取 24990 留余量)
    if (total_current > 24990.0f) {
        total_current = 24990.0f;
    } else if (total_current < -24990.0f) {
        total_current = -24990.0f;
    }
    // 最终输出到电机对象
    self->output_value = (int16_t)total_current;
}

void DJI_Motor_SendGroup_0x200(CAN_HandleTypeDef *hcan, int16_t c1, int16_t c2, int16_t c3, int16_t c4)
{
    uint8_t data[8];

    data[0] = c1 >> 8;
    data[1] = c1;
    data[2] = c2 >> 8;
    data[3] = c2;
    data[4] = c3 >> 8;
    data[5] = c3;
    data[6] = c4 >> 8;
    data[7] = c4;

    BSP_CAN_SendMsg(hcan, 0x200,CAN_ID_STD, data, 8);
}

void DJI_Motor_SendGroup_0x1FF(CAN_HandleTypeDef *hcan, int16_t c1, int16_t c2, int16_t c3, int16_t c4) {
    uint8_t data[8];

    data[0] = c1 >> 8;
    data[1] = c1;
    data[2] = c2 >> 8;
    data[3] = c2;
    data[4] = c3 >> 8;
    data[5] = c3;
    data[6] = c4 >> 8;
    data[7] = c4;
    BSP_CAN_SendMsg(hcan, 0x1FF,CAN_ID_STD, data, 8);
}

void DJI_Motor_SendGroup_0x1FE(CAN_HandleTypeDef *hcan, int16_t c1, int16_t c2, int16_t c3) {
    uint8_t data[8];

    data[0] = c1 >> 8;
    data[1] = c1;
    data[2] = c2 >> 8;
    data[3] = c2;
    data[4] = c3 >> 8;
    data[5] = c3;
    data[6] = 0;
    data[7] = 0;
    BSP_CAN_SendMsg(hcan, 0x1FE,CAN_ID_STD, data, 8);
}

void DJI_Motor_SendGroup_0x2FF(CAN_HandleTypeDef *hcan, int16_t c1, int16_t c2, int16_t c3) {
    uint8_t data[8];

    data[0] = c1 >> 8;
    data[1] = c1;
    data[2] = c2 >> 8;
    data[3] = c2;
    data[4] = c3 >> 8;
    data[5] = c3;
    data[6] = 0;
    data[7] = 0;
    BSP_CAN_SendMsg(hcan, 0x2FF,CAN_ID_STD, data, 8);
}
