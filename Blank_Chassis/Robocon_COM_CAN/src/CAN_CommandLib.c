/**
 ******************************************************************************
 * @file    RobotCAN_CommandLib.c
 * @author  Robocon
 * @brief   本代码提供了利用CAN通信发送消息功能的封装:
 *           - 根据每年机器人的具体需求，自行定义相应的发送函数
 *  @verbatim
 *
 *  @endverbatim
 ******************************************************************************
 */
#include "CAN_Basic.h"
#include "CAN_IDconf.h"
#include "CAN_Proplist.h"
#include "Frame_Change.h"
#include "TaskChassis.h"

/***************************************CAN1发送函数******************************************/

void Get_Train_Request(void)
{
    g_CAN1_Frame_tsm.Id.ExtID.DesDeviceId = CHASSIS_BROADCAST_ID;
    g_CAN1_Frame_tsm.Id.ExtID.Permit = 0;
    g_CAN1_Frame_tsm.Id.ExtID.Priority = 0;
    g_CAN1_Frame_tsm.Id.ExtID.Property = GET_TRAIN_INFO;
    g_CAN1_Frame_tsm.Id.ExtID.SrcDeviceId = LOCATE_ID;
    g_CAN1_Frame_tsm.IDE = CAN_ID_EXT;
    g_CAN1_Frame_tsm.isRemote = 1;
    g_CAN1_Frame_tsm.Length = 0;
    Send_Frame_CAN(&g_CAN1_Frame_tsm, MYCAN1);
}

float Wheel_Speed[4] = {0.0f, 0.0f, 0.0f,0.0f};
void Set_Train_Info(ChassisController *Chassis_Controller, int Train_Num)
{
    uint32_t ID[WHEEL_NUM] = {CHASSIS_TRAIN1_ID, CHASSIS_TRAIN2_ID, CHASSIS_TRAIN3_ID,CHASSIS_TRAIN4_ID};

    g_CAN1_Frame_tsm.Id.ExtID.DesDeviceId = ID[Train_Num - 1];
    g_CAN1_Frame_tsm.Id.ExtID.Permit = 0;
    g_CAN1_Frame_tsm.Id.ExtID.Priority = 0;
    g_CAN1_Frame_tsm.Id.ExtID.Property = SEND_TRAIN_INFO;
    g_CAN1_Frame_tsm.Id.ExtID.SrcDeviceId = LOCATE_ID;
    g_CAN1_Frame_tsm.IDE = CAN_ID_EXT;
    g_CAN1_Frame_tsm.isRemote = 0;
    g_CAN1_Frame_tsm.Length = 8;
    g_CAN1_Frame_tsm.Data.floats[0] =Chassis_Controller->AimState.WheelSpeed[Train_Num - 1] * kWHEEL_mms2rpm;
		
    Wheel_Speed[Train_Num - 1] = Chassis_Controller->AimState.WheelSpeed[Train_Num - 1];
		
    g_CAN1_Frame_tsm.Data.floats[1] = GlobalAngle2RobotAngle(Chassis_Controller->AimState.HelmAngle[Train_Num - 1] *DEG2RAD) * RAD2DEG;

    Send_Frame_CAN(&g_CAN1_Frame_tsm, MYCAN1);
}

void Stop_Train(void)
{
    g_CAN1_Frame_tsm.Id.ExtID.DesDeviceId = CHASSIS_TRAIN1_ID;
    g_CAN1_Frame_tsm.Id.ExtID.Permit = 0;
    g_CAN1_Frame_tsm.Id.ExtID.Priority = 0;
    g_CAN1_Frame_tsm.Id.ExtID.Property = STOP_TRAIN;
    g_CAN1_Frame_tsm.Id.ExtID.SrcDeviceId = LOCATE_ID;
    g_CAN1_Frame_tsm.IDE = CAN_ID_EXT;
    g_CAN1_Frame_tsm.isRemote = 0;
    g_CAN1_Frame_tsm.Length = 0;
    Send_Frame_CAN(&g_CAN1_Frame_tsm, MYCAN1);
    g_CAN1_Frame_tsm.Id.ExtID.DesDeviceId = CHASSIS_TRAIN2_ID;
    Send_Frame_CAN(&g_CAN1_Frame_tsm, MYCAN1);
    g_CAN1_Frame_tsm.Id.ExtID.DesDeviceId = CHASSIS_TRAIN3_ID;
    Send_Frame_CAN(&g_CAN1_Frame_tsm, MYCAN1);
	  g_CAN1_Frame_tsm.Id.ExtID.DesDeviceId = CHASSIS_TRAIN4_ID;
    Send_Frame_CAN(&g_CAN1_Frame_tsm, MYCAN1);
}

/***************************************CAN2发送函数******************************************/
