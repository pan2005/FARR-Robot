   //
// Created by Admin on 2025/11/21.
//

#ifndef __SUPERCAPACITOR_H
#define __SUPERCAPACITOR_H

#include "main.h"

#define Super_Cap_Control_ID    0x300
#define Super_Cap_Return_ID     0x301

typedef enum {
    CAP_AUTO = 0x00,                //自动模式（默认）
    CAP_OUTPUT_DISABLE = 0x01,      //只充不放
    CAP_WIRELESS_CHARGING = 0x02,   //无线充电（进行无线充电的时候需要发送该码）
    CAP_SWITCH_OFF = 0x03,          //电容关机（关闭超电功率通道、充放都禁止）
} Super_Cap_CMD;
typedef struct
{
    float       Power_Now;              //当前底盘功率*10
    uint16_t    Power_Limit;            //功率限制
    uint16_t    Energy_Buffer;          //缓冲能量
    uint8_t     Cmd;                    //命令码
    uint8_t     Chassis_Status;         //底盘是否放电（1放电，0不放电）

}Super_Cap_Control_Pack;

typedef struct
{
    uint8_t SwitchStatus : 2;
   uint8_t CANOffLine : 1;
   uint8_t SwithError : 1;
   uint8_t LowVotage : 1;
   uint8_t HighCurrent : 1;
    uint8_t HighTemperature : 1;
    uint8_t FANStatus : 1;
} Super_Cap_Return_Status;
// 输出状态
// CAN离线
// 开关板异常
// 低电压警告
// 大电流警告
// 高温警告
// 风扇工作状态
typedef union
{
    Super_Cap_Return_Status Status;
    uint8_t value;
} Super_Cap_Return_Union;


typedef struct
{
    int16_t Cell_Power;   // 电管输出功率 * 10
    int16_t Cap_Power;  // 电容充电功率（正为充电，负为放电）* 10



    int16_t Capacity_Voltage;  //当前电压x100
    uint8_t Temperature; //温度
    Super_Cap_Return_Union Status;
} Super_Cap_Return_Pack;

#endif //RM_HERO_26_V1_0_RTOS_SUPERCAPACITOR_H