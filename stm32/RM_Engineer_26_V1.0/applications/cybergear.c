//
// Created by PanZhaoHui on 25-11-1.
//

/**
  ****************************(C)SWJTU_ROBOTCON****************************
  * @file       cybergear.c/h
  * @brief      小米电机函数库
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     1-10-2023       ZDYukino        1. done
  *
  @verbatim
  =========================================================================
  =========================================================================
  @endverbatim
  ****************************(C)SWJTU_ROBOTCON****************************
  **/
#include "main.h"
#include "can.h"
#include "cybergear.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include "remote_control.h"


CAN_TxHeaderTypeDef txMsg;//发送配置结构体

uint32_t Motor_Can_ID;    //接收数据电机ID
uint8_t byte[4];          //转换临时数据
uint32_t send_mail_box = {0};//NONE

#define can_txd() HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, &send_mail_box)//CAN发送宏定义




void test() {
	uint8_t tx_data[8] = {0};
	txMsg.IDE = CAN_ID_EXT;     //配置CAN发送：扩展帧
	txMsg.RTR = CAN_RTR_DATA;   //配置CAN发送：数据帧
	txMsg.DLC = 0x08;           //配置CAN发送：数据长度
	txMsg.ExtId = 0x0300017F;
	can_txd();

}


MI_Motor mi_motor[4];//预先定义四个小米电机
extern const RC_ctrl_t * local_rc_ctrl;
/**
  * @brief          浮点数转4字节函数
  * @param[in]      f:浮点数
  * @retval         4字节数组
  * @description  : IEEE 754 协议
  */
static uint8_t* Float_to_Byte(float f)
{
	unsigned long longdata = 0;
	longdata = *(unsigned long*)&f;
	byte[0] = (longdata & 0xFF000000) >> 24;
	byte[1] = (longdata & 0x00FF0000) >> 16;
	byte[2] = (longdata & 0x0000FF00) >> 8;
	byte[3] = (longdata & 0x000000FF);
	return byte;
}

/**
  * @brief          小米电机回文16位数据转浮点
  * @param[in]      x:16位回文
  * @param[in]      x_min:对应参数下限
  * @param[in]      x_max:对应参数上限
  * @param[in]      bits:参数位数
  * @retval         返回浮点值
  */
static float uint16_to_float(uint16_t x,float x_min,float x_max,int bits)
{
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

/**
  * @brief          小米电机发送浮点转16位数据
  * @param[in]      x:浮点
  * @param[in]      x_min:对应参数下限
  * @param[in]      x_max:对应参数上限
  * @param[in]      bits:参数位数
  * @retval         返回浮点值
  */
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  if(x > x_max) x=x_max;
  else if(x < x_min) x= x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief          写入电机参数
  * @param[in]      Motor:对应控制电机结构体
  * @param[in]      Index:写入参数对应地址
  * @param[in]      Value:写入参数值
  * @param[in]      Value_type:写入参数数据类型
  * @retval         none
  */
static void Set_Motor_Parameter(MI_Motor *Motor,uint16_t Index,float Value,char Value_type){
	uint8_t tx_data[8];
	txMsg.ExtId = Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|Motor->CAN_ID;
	tx_data[0]=Index;
	tx_data[1]=Index>>8;
	tx_data[2]=0x00;
	tx_data[3]=0x00;
	if(Value_type == 'f'){
		Float_to_Byte(Value);
		tx_data[4]=byte[3];
		tx_data[5]=byte[2];
		tx_data[6]=byte[1];
		tx_data[7]=byte[0];
	}
	else if(Value_type == 's'){
		tx_data[4]=(uint8_t)Value;
		tx_data[5]=0x00;
		tx_data[6]=0x00;
		tx_data[7]=0x00;
	}
	can_txd();
	HAL_Delay(1);
}

/**
  * @brief          提取电机回复帧扩展ID中的电机CANID
  * @param[in]      CAN_ID_Frame:电机回复帧中的扩展CANID
  * @retval         电机CANID
  */
uint32_t Get_Motor_ID(uint32_t CAN_ID_Frame)
{
	return (CAN_ID_Frame&0xFFFF)>>8;
}

/**
  * @brief          电机回复帧数据处理函数
  * @param[in]      Motor:对应控制电机结构体
  * @param[in]      DataFrame:数据帧
  * @param[in]      IDFrame:扩展ID帧
  * @retval         None
  */
void Motor_Data_Handler(MI_Motor *Motor,uint8_t DataFrame[8],uint32_t IDFrame)
{
		Motor->Angle=uint16_to_float(DataFrame[0]<<8|DataFrame[1],MIN_P,MAX_P,16);
		Motor->Speed=uint16_to_float(DataFrame[2]<<8|DataFrame[3],V_MIN,V_MAX,16);
		Motor->Torque=uint16_to_float(DataFrame[4]<<8|DataFrame[5],T_MIN,T_MAX,16);
		Motor->Temp=(DataFrame[6]<<8|DataFrame[7])*Temp_Gain;
		Motor->error_code=(IDFrame&0x1F0000)>>16;
}

/**
  * @brief          小米电机ID检查
  * @param[in]      id:  控制电机CAN_ID【出厂默认0x7F】
  * @retval         none
  */
void chack_cybergear(uint8_t ID)
{
    uint8_t tx_data[8] = {0};
    txMsg.ExtId = Communication_Type_GetID<<24|Master_CAN_ID<<8|ID;
    can_txd();
}

/**
  * @brief          使能小米电机
  * @param[in]      Motor:对应控制电机结构体
  * @retval         none
  */
void start_cybergear(MI_Motor *Motor)
{
    uint8_t tx_data[8] = {0};
    txMsg.ExtId = Communication_Type_MotorEnable<<24|Master_CAN_ID<<8|Motor->CAN_ID;
    can_txd();
}

/**
  * @brief          停止电机
  * @param[in]      Motor:对应控制电机结构体
  * @param[in]      clear_error:清除错误位（0 不清除 1清除）
  * @retval         None
  */
void stop_cybergear(MI_Motor *Motor,uint8_t clear_error)
{
	uint8_t tx_data[8]={0};
	tx_data[0]=clear_error;//清除错误位设置
	txMsg.ExtId = Communication_Type_MotorStop<<24|Master_CAN_ID<<8|Motor->CAN_ID;
    can_txd();
}

/**
  * @brief          设置电机模式(必须停止时调整！)
  * @param[in]      Motor:  电机结构体
  * @param[in]      Mode:   电机工作模式（1.运动模式Motion_mode 2. 位置模式Position_mode 3. 速度模式Speed_mode 4. 电流模式Current_mode）
  * @retval         none
  */
void set_mode_cybergear(MI_Motor *Motor,uint8_t Mode)
{
	stop_cybergear(Motor, 0);
	Set_Motor_Parameter(Motor,Run_mode,Mode,'s');
	start_cybergear(Motor);
}

/**
  * @brief          位置控制模式下设置位置
  * @param[in]      Motor:  电机结构体
  * @param[in]      position:目标位置
  * @retval         none
  */
void set_position_cybergear(MI_Motor *Motor, float position, float max_speed, float limit_current , float kp)
{
	Set_Motor_Parameter(Motor,Limit_Spd,max_speed,'f');
	Set_Motor_Parameter(Motor,Limit_Cur,limit_current,'f');
	Set_Motor_Parameter(Motor,Loc_Kp,kp,'f');
	Set_Motor_Parameter(Motor,Loc_Ref,position,'f');
}


/**
  * @brief          速度控制模式下设置速度
  * @param[in]      Motor:  电机结构体
  * @param[in]      speed:目标速度
  * @retval         none
  */
void set_speed_cybergear(MI_Motor *Motor, float speed)
{
	Set_Motor_Parameter(Motor,Spd_Ref,speed,'f');
}

/**
  * @brief          电流控制模式下设置电流
  * @param[in]      Motor:  电机结构体
  * @param[in]      Current:电流设置
  * @retval         none
  */
void set_current_cybergear(MI_Motor *Motor,float Current)
{
	Set_Motor_Parameter(Motor,Iq_Ref,Current,'f');
}

/**
  * @brief          设置电机零点
  * @param[in]      Motor:  电机结构体
  * @retval         none
  */
void set_zeropos_cybergear(MI_Motor *Motor)
{
	uint8_t tx_data[8]={0};
	txMsg.ExtId = Communication_Type_SetPosZero<<24|Master_CAN_ID<<8|Motor->CAN_ID;
	can_txd();
}

/**
  * @brief          设置电机CANID
  * @param[in]      Motor:  电机结构体
  * @param[in]      Motor:  设置新ID
  * @retval         none
  */
void set_CANID_cybergear(MI_Motor *Motor,uint8_t CAN_ID)
{
	uint8_t tx_data[8]={0};
	txMsg.ExtId = Communication_Type_CanID<<24|CAN_ID<<16|Master_CAN_ID<<8|Motor->CAN_ID;
    Motor->CAN_ID = CAN_ID;//将新的ID导入电机结构体
    can_txd();
}
/**
  * @brief          小米电机初始化
  * @param[in]      Motor:  电机结构体
  * @param[in]      Can_Id: 小米电机ID(默认0x7F)
  * @param[in]      Motor_Num: 电机编号
  * @param[in]      mode: 电机工作模式（0.运动模式Motion_mode 1. 位置模式Position_mode 2. 速度模式Speed_mode 3. 电流模式Current_mode）
  * @retval         none
  */
void init_cybergear(MI_Motor *Motor,uint8_t Can_Id, uint8_t mode)
{
    txMsg.StdId = 0;            //配置CAN发送：标准帧清零
    txMsg.ExtId = 0;            //配置CAN发送：扩展帧清零
    txMsg.IDE = CAN_ID_EXT;     //配置CAN发送：扩展帧
    txMsg.RTR = CAN_RTR_DATA;   //配置CAN发送：数据帧
    txMsg.DLC = 0x08;           //配置CAN发送：数据长度

	Motor->CAN_ID=Can_Id;       //ID设置
	start_cybergear(Motor);
	HAL_Delay(1000);
	set_mode_cybergear(Motor,mode);//设置电机模式
	//start_cybergear(Motor);        //使能电机
}

/**
  * @brief          小米运控模式指令
  * @param[in]      Motor:  目标电机结构体
  * @param[in]      torque: 力矩设置[-12,12] N*M
  * @param[in]      MechPosition: 位置设置[-12.5,12.5] rad
  * @param[in]      speed: 速度设置[-30,30] rpm
  * @param[in]      kp: 比例参数设置
  * @param[in]      kd: 微分参数设置
  * @retval         none
  */
void motor_controlmode(MI_Motor *Motor,float torque, float MechPosition, float speed, float kp, float kd)
{
    uint8_t tx_data[8];//发送数据初始化
    //装填发送数据
    tx_data[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8;
    tx_data[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16);
    tx_data[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;
    tx_data[3]=float_to_uint(speed,V_MIN,V_MAX,16);
    tx_data[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;
    tx_data[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);
    tx_data[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;
    tx_data[7]=float_to_uint(kd,KD_MIN,KD_MAX,16);

    txMsg.ExtId = Communication_Type_MotionControl<<24|float_to_uint(torque,T_MIN,T_MAX,16)<<8|Motor->CAN_ID;//装填扩展帧数据
    can_txd();
}


void cybergear_task(void * argument) {
	mi_motor[1].error_code = 1;
	//can_bsp_init();
	HAL_Delay(1000);
	while(mi_motor[1].error_code != 0) {
		init_cybergear(&mi_motor[1],0x7F,Speed_mode);
	}
	init_cybergear(&mi_motor[1],0x7F,Speed_mode);
	HAL_Delay(1000);

	while (1) {
		set_speed_cybergear(&mi_motor[1],local_rc_ctrl->rc.ch[3] / 10);
	}
}