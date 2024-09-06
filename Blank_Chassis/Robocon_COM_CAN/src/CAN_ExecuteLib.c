/**
 ******************************************************************************
 * @file    RobotCAN_ExecuteLib.c
 * @author  Robocon
 * @brief   本代码提供了CAN消息处理函数的定义:
 *           - CAN1，CAN2非团队协议扩展帧消息处理函数
 *           - CAN1，CAN2非团队协议标准帧消息处理函数
 *           - 其他自定义团队协议扩展帧消息处理函数
 *  @verbatim
 *          根据需求添加相应的消息处理函数，添加后更新相应的proplist.c即可
 *  @endverbatim
 ******************************************************************************
 */
#include "CAN_Basic.h"
#include "CAN_Proplist.h"
#include "TaskChassis.h"
#include "Frame_Change.h"
#include "HUST_Math_Lib.h"


/*******************************CAN1接收函数***********************************/
/**
 * @brief  
 * @param  
 * @retval 无
 */
void ExtID_Non_Protocol_CAN1(CANFrame *Frame)
{
}

/**
 * @brief  CAN1非团队协议标准帧消息处理函数
 * @param  数据帧地址
 * @retval 无
 */
void StdID_Non_Protocol_CAN1(CANFrame *Frame)
{
}

void Get_Train_Info(CANFrame *Frame)
{
    EXT_ID_Typedef Ext_Id = Frame->Id.ExtID;

    switch (Ext_Id.SrcDeviceId)
    {
    case CHASSIS_TRAIN1_ID:
        g_Chassis_Instance.CurrentState.WheelSpeed[0] =
            Frame->Data.floats[0] / kWHEEL_mms2rpm;
        g_Chassis_Instance.CurrentState.HelmAngle[0] =
            RobotAngle2GlobalAngle(Frame->Data.floats[1] * DEG2RAD) * RAD2DEG;
        break;

    case CHASSIS_TRAIN2_ID:
        g_Chassis_Instance.CurrentState.WheelSpeed[1] =
            Frame->Data.floats[0] / kWHEEL_mms2rpm;
        g_Chassis_Instance.CurrentState.HelmAngle[1] =
            RobotAngle2GlobalAngle(Frame->Data.floats[1] * DEG2RAD) * RAD2DEG;
        break;

    case CHASSIS_TRAIN3_ID:
        g_Chassis_Instance.CurrentState.WheelSpeed[2] =
            Frame->Data.floats[0] / kWHEEL_mms2rpm;
        g_Chassis_Instance.CurrentState.HelmAngle[2] =
            RobotAngle2GlobalAngle(Frame->Data.floats[1] * DEG2RAD) * RAD2DEG;
        break;
		
		case CHASSIS_TRAIN4_ID:
        g_Chassis_Instance.CurrentState.WheelSpeed[3] =
            Frame->Data.floats[0] / kWHEEL_mms2rpm;
        g_Chassis_Instance.CurrentState.HelmAngle[3] =
            RobotAngle2GlobalAngle(Frame->Data.floats[1] * DEG2RAD) * RAD2DEG;
        break;
		
    }
		
}
/*******************************CAN2接收函数***********************************/

/**
 * @brief  CAN2非团队协议扩展帧消息处理函数
 * @param  数据帧地址
 * @retval 无
 */
void ExtID_Non_Protocol_CAN2(CANFrame *Frame)
{
}

/**
 * @brief  CAN1非团队协议标准帧消息处理函数
 * @param  数据帧地址
 * @retval 无
 */
void StdID_Non_Protocol_CAN2(CANFrame *Frame)
{
}
