/**
  ******************************************************************************
  * @file    Frame_Change.c
  * @author  Robocon
  * @brief   本代码提供了全场，机器人，电机三个坐标系相互转换的函数定义:
  *           - 角度转换函数
  *           - 矢量转换函数（待写）
  *  @verbatim
  *          对于电机坐标系而言，存在角度增大方向的定义区别，即需根据实际安装修改
  *          Helm_Motor_DIR变量，逆时针为1，顺时针为-1
  *  @endverbatim
  ******************************************************************************  
  */ 
#include "Frame_Change.h"

/**
 * @brief  transmit the angle in global frame to the angel in robot frame
 * @param  globalangel为全场坐标系下的角度（单位：rad）
 * @retval 无
 */
float GlobalAngle2RobotAngle(float Global_Angel)
{
    float Robot_Angle;

	Robot_Angle = Global_Angel - g_Chassis_Instance.CurrentState.Alpha * DEG2RAD;
    Robot_Angle = NormalizeAngle(Robot_Angle);

	return Robot_Angle;
}

/**
 * @brief  transmit the angle in robot frame to the angel in global frame
 * @param  robotangle为机器人坐标系下的角度（单位：rad）
 * @retval 无
 */
float RobotAngle2GlobalAngle(float Robot_Angle)
{
    float Global_Angel;

	Global_Angel = Robot_Angle + g_Chassis_Instance.CurrentState.Alpha * DEG2RAD;
  Global_Angel = NormalizeAngle(Global_Angel);
    
	return Global_Angel;
}



