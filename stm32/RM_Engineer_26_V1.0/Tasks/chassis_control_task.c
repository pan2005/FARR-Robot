//
// Created by Admin on 2025/12/4.
//

#include "chassis_control_task.h"

#include "cmsis_os2.h"
#include "can.h"
#include "can_motor.h"
#include "DM_Motor.h"
#include "pid.h"
#include "rdk_comm_task.h"
#include "remote_control.h"

#define CHASSIS_WHEEL_SPEED_SCALE     10.0f
#define CHASSIS_ROTATE_WHEEL_SCALE    12.0f

#define CHASSIS_WHEELBASE_M           0.41131f
#define CHASSIS_TRACK_WIDTH_M         0.43520f
#define CHASSIS_ROTATION_RADIUS_M     ((CHASSIS_WHEELBASE_M + CHASSIS_TRACK_WIDTH_M) * 0.5f)

#define DM4340_KP                     25.0f
#define DM4340_KD                     3.0f
#define DM4340_P_MAX                  12.5f
#define DM4340_V_MAX                  3.0f
#define DM4340_T_MAX                  10.0f

#define ARM_POSITION_MIN              (-3.5f)
#define ARM_POSITION_MAX              3.5f
#define ARM_POSITION_STEP_PER_LOOP     0.0001f
#define ARM_RC_DEADBAND               20
#define ARM_CONTROL_PERIOD_MS         20U
#define ARM_RDK_DELTA_LIMIT           0.02f
#define ARM_ZERO_SWITCH_INDEX         0U

Can_Motor_t wheel_motor_lf;
M3508_Data_t wheel_priv_lf;

Can_Motor_t wheel_motor_rf;
M3508_Data_t wheel_priv_rf;

Can_Motor_t wheel_motor_lr;
M3508_Data_t wheel_priv_lr;

Can_Motor_t wheel_motor_rr;
M3508_Data_t wheel_priv_rr;

DM_Motor_obj_t arm_motor_lf;
DM_Motor_obj_t arm_motor_rf;
DM_Motor_obj_t arm_motor_lr;
DM_Motor_obj_t arm_motor_rr;

static float arm_target_lf = 0.0f;
static float arm_target_rf = 0.0f;
static float arm_target_lr = 0.0f;
static float arm_target_rr = 0.0f;
static uint8_t arm_zero_reset_mode = 0U;

static void Chassis_SendArmTargets(void);
static void Chassis_StopWheels(void);

static float Chassis_ClampFloat(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static int16_t Chassis_ApplyDeadband(int16_t value, int16_t deadband)
{
    if (value > -deadband && value < deadband) {
        return 0;
    }

    return value;
}

static void Chassis_WheelMotorInit(void)
{
    PID_Init(&wheel_priv_lf.speed_pid, 7.0f, 0.1f, 0.0f, 500.0f, 16384.0f);
    Can_Motor_Init(&wheel_motor_lf, &hcan1, 0x201, M3508_Decode, M3508_Update, &wheel_priv_lf);

    PID_Init(&wheel_priv_rf.speed_pid, 7.0f, 0.1f, 0.0f, 500.0f, 16384.0f);
    Can_Motor_Init(&wheel_motor_rf, &hcan1, 0x202, M3508_Decode, M3508_Update, &wheel_priv_rf);

    PID_Init(&wheel_priv_lr.speed_pid, 7.0f, 0.1f, 0.0f, 500.0f, 16384.0f);
    Can_Motor_Init(&wheel_motor_lr, &hcan1, 0x203, M3508_Decode, M3508_Update, &wheel_priv_lr);

    PID_Init(&wheel_priv_rr.speed_pid, 7.0f, 0.1f, 0.0f, 500.0f, 16384.0f);
    Can_Motor_Init(&wheel_motor_rr, &hcan1, 0x204, M3508_Decode, M3508_Update, &wheel_priv_rr);
}

static void Chassis_DM4340Init(void)
{
    DM_MIT_init(&arm_motor_lf, 0x01, &hcan1, 5, DM4340_KP, DM4340_KD, DM4340_P_MAX, DM4340_V_MAX, DM4340_T_MAX);
    osDelay(1);
    DM_MIT_init(&arm_motor_rf, 0x02, &hcan1, 5, DM4340_KP, DM4340_KD, DM4340_P_MAX, DM4340_V_MAX, DM4340_T_MAX);
    osDelay(1);
    DM_MIT_init(&arm_motor_lr, 0x03, &hcan1, 5, DM4340_KP, DM4340_KD, DM4340_P_MAX, DM4340_V_MAX, DM4340_T_MAX);
    osDelay(1);
    DM_MIT_init(&arm_motor_rr, 0x04, &hcan1, 5, DM4340_KP, DM4340_KD, DM4340_P_MAX, DM4340_V_MAX, DM4340_T_MAX);
    osDelay(1);

    DM_MIT_send_enable_cmd(&arm_motor_lf);
    osDelay(1);
    DM_MIT_send_enable_cmd(&arm_motor_rf);
    osDelay(1);
    DM_MIT_send_enable_cmd(&arm_motor_lr);
    osDelay(1);
    DM_MIT_send_enable_cmd(&arm_motor_rr);
    osDelay(1);
}

static void Chassis_DisableArmMotors(void)
{
    DM_MIT_send_disable_cmd(&arm_motor_lf);
    osDelay(1);
    DM_MIT_send_disable_cmd(&arm_motor_rf);
    osDelay(1);
    DM_MIT_send_disable_cmd(&arm_motor_lr);
    osDelay(1);
    DM_MIT_send_disable_cmd(&arm_motor_rr);
    osDelay(1);
}

static void Chassis_EnableArmMotors(void)
{
    DM_MIT_send_enable_cmd(&arm_motor_lf);
    osDelay(1);
    DM_MIT_send_enable_cmd(&arm_motor_rf);
    osDelay(1);
    DM_MIT_send_enable_cmd(&arm_motor_lr);
    osDelay(1);
    DM_MIT_send_enable_cmd(&arm_motor_rr);
    osDelay(1);
}

static void Chassis_SaveArmZeroPosition(void)
{
    DM_MIT_send_save_zero_cmd(&arm_motor_lf);
    osDelay(2);
    DM_MIT_send_save_zero_cmd(&arm_motor_rf);
    osDelay(2);
    DM_MIT_send_save_zero_cmd(&arm_motor_lr);
    osDelay(2);
    DM_MIT_send_save_zero_cmd(&arm_motor_rr);
    osDelay(2);
}

static void Chassis_ResetArmTargetsToZero(void)
{
    arm_target_lf = 0.0f;
    arm_target_rf = 0.0f;
    arm_target_lr = 0.0f;
    arm_target_rr = 0.0f;
}

static void Chassis_EnterArmZeroResetMode(void)
{
    arm_zero_reset_mode = 1U;
    Chassis_StopWheels();
    Chassis_DisableArmMotors();
}

static void Chassis_FinishArmZeroResetMode(void)
{
    Chassis_SaveArmZeroPosition();
    Chassis_ResetArmTargetsToZero();
    Chassis_EnableArmMotors();
    arm_zero_reset_mode = 0U;
    Chassis_SendArmTargets();
}

static void Chassis_UpdateArmZeroSwitchEdge(uint16_t current_switch, uint16_t *last_switch)
{
    if (last_switch == NULL) {
        return;
    }

    if (!switch_is_up(*last_switch) && switch_is_up(current_switch)) {
        Chassis_EnterArmZeroResetMode();
    } else if (switch_is_up(*last_switch) && !switch_is_up(current_switch) && arm_zero_reset_mode != 0U) {
        Chassis_FinishArmZeroResetMode();
    }

    *last_switch = current_switch;
}

static void Chassis_UpdateArmTargetFromRc(void)
{
    const int16_t front_input = Chassis_ApplyDeadband(local_rc_ctrl->rc.ch[0], ARM_RC_DEADBAND);
    const int16_t rear_input = Chassis_ApplyDeadband(local_rc_ctrl->rc.ch[1], ARM_RC_DEADBAND);

    const float front_delta = (float)front_input * ARM_POSITION_STEP_PER_LOOP;
    const float rear_delta = (float)rear_input * ARM_POSITION_STEP_PER_LOOP;

    arm_target_lf = Chassis_ClampFloat(arm_target_lf + front_delta, ARM_POSITION_MIN, ARM_POSITION_MAX);
    arm_target_rf = Chassis_ClampFloat(arm_target_rf - front_delta, ARM_POSITION_MIN, ARM_POSITION_MAX);

    arm_target_lr = Chassis_ClampFloat(arm_target_lr + rear_delta, ARM_POSITION_MIN, ARM_POSITION_MAX);
    arm_target_rr = Chassis_ClampFloat(arm_target_rr - rear_delta, ARM_POSITION_MIN, ARM_POSITION_MAX);
}

static void Chassis_UpdateArmTargetFromRdk(const RdkControl_t *control)
{
    if (control == NULL) {
        return;
    }

    const float front_delta = Chassis_ClampFloat(control->front_arm_delta, -ARM_RDK_DELTA_LIMIT, ARM_RDK_DELTA_LIMIT);
    const float rear_delta = Chassis_ClampFloat(control->rear_arm_delta, -ARM_RDK_DELTA_LIMIT, ARM_RDK_DELTA_LIMIT);

    arm_target_lf = Chassis_ClampFloat(arm_target_lf + front_delta, ARM_POSITION_MIN, ARM_POSITION_MAX);
    arm_target_rf = Chassis_ClampFloat(arm_target_rf - front_delta, ARM_POSITION_MIN, ARM_POSITION_MAX);

    arm_target_lr = Chassis_ClampFloat(arm_target_lr + rear_delta, ARM_POSITION_MIN, ARM_POSITION_MAX);
    arm_target_rr = Chassis_ClampFloat(arm_target_rr - rear_delta, ARM_POSITION_MIN, ARM_POSITION_MAX);
}

static void Chassis_SendArmTargets(void)
{
    DM_MIT_set_target(&arm_motor_lf, 3, arm_target_lf, 0.0, 0.0);
    DM_MIT_send_ctrl_cmd(&arm_motor_lf);
    osDelay(1);

    DM_MIT_set_target(&arm_motor_rf, 3, arm_target_rf, 0.0, 0.0);
    DM_MIT_send_ctrl_cmd(&arm_motor_rf);
    osDelay(1);

    DM_MIT_set_target(&arm_motor_lr, 3, arm_target_lr, 0.0, 0.0);
    DM_MIT_send_ctrl_cmd(&arm_motor_lr);
    osDelay(1);

    DM_MIT_set_target(&arm_motor_rr, 3, arm_target_rr, 0.0, 0.0);
    DM_MIT_send_ctrl_cmd(&arm_motor_rr);
    osDelay(1);
}

static void Chassis_SetVelocityTarget(float vx, float vy, float w)
{
    const float rotate_component = w * CHASSIS_ROTATION_RADIUS_M;

    wheel_priv_lf.target_speed = vx + vy + rotate_component;
    wheel_priv_rf.target_speed = -(vx - vy - rotate_component);
    wheel_priv_lr.target_speed = vx - vy + rotate_component;
    wheel_priv_rr.target_speed = -(vx + vy - rotate_component);
}

static void Chassis_UpdateAndSendWheels(void)
{
    wheel_motor_lf.update_func(&wheel_motor_lf);
    wheel_motor_rf.update_func(&wheel_motor_rf);
    wheel_motor_lr.update_func(&wheel_motor_lr);
    wheel_motor_rr.update_func(&wheel_motor_rr);

    DJI_Motor_SendGroup_0x200(&hcan1,
                              wheel_motor_lf.output_value,
                              wheel_motor_rf.output_value,
                              wheel_motor_lr.output_value,
                              wheel_motor_rr.output_value);
}

static void Chassis_StopWheels(void)
{
    wheel_priv_lf.target_speed = 0.0f;
    wheel_priv_rf.target_speed = 0.0f;
    wheel_priv_lr.target_speed = 0.0f;
    wheel_priv_rr.target_speed = 0.0f;
    Chassis_UpdateAndSendWheels();
}

void chassis_control_task(void *argument)
{
    (void)argument;

    osDelay(1000);

    Chassis_WheelMotorInit();
    Chassis_DM4340Init();

    osDelay(100);
    Chassis_SendArmTargets();

    uint32_t last_arm_ctrl_tick = osKernelGetTickCount();
    uint16_t last_arm_zero_switch = (uint16_t)local_rc_ctrl->rc.s[ARM_ZERO_SWITCH_INDEX];

    while (1) {
        RdkControl_t rdk_control;
        Chassis_UpdateArmZeroSwitchEdge((uint16_t)local_rc_ctrl->rc.s[ARM_ZERO_SWITCH_INDEX], &last_arm_zero_switch);

        if (arm_zero_reset_mode != 0U) {
            Chassis_StopWheels();
            osDelay(2);
            continue;
        }

        if (switch_is_mid(local_rc_ctrl->rc.s[1]) || switch_is_down(local_rc_ctrl->rc.s[1])) {
            const float vx = (float)local_rc_ctrl->rc.ch[3] * CHASSIS_WHEEL_SPEED_SCALE;
            const float vy = -(float)local_rc_ctrl->rc.ch[2] * CHASSIS_WHEEL_SPEED_SCALE;
            float w = 0.0f;

            if (switch_is_mid(local_rc_ctrl->rc.s[1])) {
                w = (float)local_rc_ctrl->rc.ch[1] * CHASSIS_ROTATE_WHEEL_SCALE / CHASSIS_ROTATION_RADIUS_M;
            } else {
                Chassis_UpdateArmTargetFromRc();
            }

            Chassis_SetVelocityTarget(vx, vy, w);
            Chassis_UpdateAndSendWheels();
        } else if (rdk_comm_get_control(&rdk_control) != 0U) {
            Chassis_SetVelocityTarget(rdk_control.vx, rdk_control.vy, rdk_control.w);
            Chassis_UpdateArmTargetFromRdk(&rdk_control);
            Chassis_UpdateAndSendWheels();
        } else {
            Chassis_StopWheels();
        }

        const uint32_t now = osKernelGetTickCount();
        if ((uint32_t)(now - last_arm_ctrl_tick) >= ARM_CONTROL_PERIOD_MS) {
            last_arm_ctrl_tick = now;
            Chassis_SendArmTargets();
        }

        osDelay(2);
    }
}

void M3508_test_task(void)
{
}
