/**
  ******************************************************************************
  * @file    Frame_Change.h
  * @author  Robocon
  * @brief   本代码提供了全场，机器人，电机三个坐标系相互转换的函数声明:
  *           - 角度转换函数
  *           - 矢量转换函数（待写）
  *  @verbatim
  * 
  *  @endverbatim
  ******************************************************************************  
  */ 
#ifndef FRAME_CHANGE_H_
#define FRAME_CHANGE_H_

#include "main.h"
#include "ChassisInit.h"
#include "TaskChassis.h"
#include "HUST_Math_Lib.h"

float RobotAngle2MotorAngle (float, int16_t);
float MotorAngle2RobotAngle (float, int16_t);
float RobotAngle2GlobalAngle(float);
float GlobalAngle2RobotAngle(float);

#endif  // FRAME_CHANGE_H_
