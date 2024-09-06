/**
 ******************************************************************************
 * @file    RobotCOM_CommandLib.c
 * @author  Robocon
 * @brief   本代码提供了利用串口通信发送消息功能的封装:
 *           - 根据每年机器人的具体需求，自行定义相应的发送函数
 *  @verbatim
 *
 *  @endverbatim
 ******************************************************************************
 */
#include "COM_Basic.h"
#include "COM_proplist.h"
#include "TaskChassis.h"
#include "HUST_Math_Lib.h"
#include "usart.h"
// MYUSART1 : PC
// MYUSART4 : 手柄
/***************************************MYUSART1发送函数******************************************/
void Send_Chassis_Info(void)//发送底盘位置给PC
{
    COMFrame SendFrame;

    SendFrame.Length = 12;
    SendFrame.Prop = SEND_CHASSIS_INFO;	
		SendFrame.Data.floats_ts[0]=g_Chassis_Instance.CurrentState.X;//x,4个字节
		SendFrame.Data.floats_ts[1]=g_Chassis_Instance.CurrentState.Y;//y,4个字节
		SendFrame.Data.floats_ts[2]=g_Chassis_Instance.CurrentState.Alpha;//theta,4个字节
//	  SendFrame.Data.floats_ts[3]=LadarSubSystem.g_x;
//	  SendFrame.Data.floats_ts[4]=LadarSubSystem.g_y;
		
    Send_Frame_COM(&SendFrame, MYUSART1);//发给PC
}

/***************************************MYUSART2发送函数******************************************/

/***************************************MYUSART3发送函数******************************************/

/***************************************MYUSART4发送函数******************************************/














