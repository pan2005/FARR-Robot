//
// Created by Admin on 2025/11/21.
//

#include "Supercapacitor.h"
#include "can.h"
#include "cmsis_os2.h"
#include "bsp_can.h"


CAN_TxHeaderTypeDef Super_cap_txmsg;
CAN_RxHeaderTypeDef Super_cap_rxmsg;

uint32_t Sup_cap_tx_smendmailbox = {0};


Super_Cap_Control_Pack Sup_Cap_Control_Pack = {
    .Chassis_Status =  0,
    .Cmd =  0,
    .Energy_Buffer =0,
    .Power_Limit = 0,
    .Power_Now = 0.0f,
};

Super_Cap_Return_Pack super_cap_return_pack;

// 初始化一个控制包实例
void Sup_cap_init(void) {
    //BSP_CAN_RegisterRxCallback(hcan1, 0x301, CAN_ID_STD, , &Sup_Cap_Control_Pack)


}
void Sup_cap_send_cmd(Super_Cap_Control_Pack *  Sup_Cap_Control_Packet) {
    uint8_t Sup_cap_senddata[8] = {0};
    Super_cap_txmsg.DLC = 8;
    Super_cap_txmsg.StdId = Super_Cap_Control_ID;
    Super_cap_txmsg.RTR = CAN_RTR_DATA;
    Super_cap_txmsg.IDE = CAN_ID_STD;
    Sup_cap_senddata[0] = Sup_Cap_Control_Packet->Cmd;
    Sup_cap_senddata[1] = (Sup_Cap_Control_Packet->Power_Limit >> 8) & 0xFF;
    Sup_cap_senddata[2] = (Sup_Cap_Control_Packet->Power_Limit >> 0) & 0xFF;
    Sup_cap_senddata[3] = 0;
    Sup_cap_senddata[4] = 0;
    Sup_cap_senddata[5] = (Sup_Cap_Control_Packet->Energy_Buffer >> 8) & 0xFF;
    Sup_cap_senddata[6] = (Sup_Cap_Control_Packet->Energy_Buffer >> 0) & 0xFF;
    Sup_cap_senddata[7] = Sup_Cap_Control_Packet->Chassis_Status;
    HAL_CAN_AddTxMessage(&hcan1,&Super_cap_txmsg,Sup_cap_senddata,&Sup_cap_tx_smendmailbox);
    HAL_Delay(1);
}


void Super_capacitor_Task(void *pvParameter) {
    Sup_Cap_Control_Pack.Cmd = CAP_AUTO;
    Sup_Cap_Control_Pack.Energy_Buffer = 0;
    Sup_Cap_Control_Pack.Power_Limit = 0;
    Sup_Cap_Control_Pack.Power_Now = 0;
    Sup_Cap_Control_Pack.Chassis_Status = 0;
    while (1) {
        Sup_cap_send_cmd(&Sup_Cap_Control_Pack);
        osDelay(500);
    }
}