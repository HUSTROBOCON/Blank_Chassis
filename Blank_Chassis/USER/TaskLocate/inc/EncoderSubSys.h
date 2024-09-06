/**
 * @file EncoderSubSys.h
 * @author Vulcan-wzy
 * @brief 码盘定位子系统，包含码盘定位的结构体声明、初始化以及码盘本身在机器人坐标系中的位置参数
 *        子系统的输出坐标是机器人坐标系下的坐标（没有包含陀螺仪信息，运动坐标系）
 * @version 1.0
 * @date 2023-10-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __ENCODERSUBSYS_H__
#define __ENCODERSUBSYS_H__

#include "TaskChassis.h"
#include "Locate.h"
#include "gyro.h"
#include "hust_math_lib.h"


/*===============================坐标系相关=============================================*/
// 码盘相关
#define ENCn 2
#define ENC_X ENC1
#define ENC_Y ENC2

/*=========标定得到，也可由模型测量==========*/
#define ENCODER_X_IN_ROBOT 0.0f
#define ENCODER_Y_IN_ROBOT 3.56382f
//===========由旋转矩阵得================================*/
#define ROBOT_X_IN_ENCODER 0.0f
#define ROBOT_Y_IN_ENCODER -3.56382f

/*============MATLAB得到============*/
#define ENCODER_THETA_IN_ROBOT 0.800864301858825901203858285024f // 编码器x轴与机器人的x轴(刚开始水平)的夹角，弧度制  0.771225873f
#define COS_ENCODER_THETA_IN_ROBOT 0.717957987789663679367578638506f
#define SIN_ENCODER_THETA_IN_ROBOT 0.696086436995447526644517973665f


// 码盘与90°的偏差  小于90为正， 大于90为负
#define DELTA_ANGLE -0.000762352699761404998919893352059f
#define COS_DELTA_ANGLE 0.999999709409194657085658819441f
#define SIN_DELTA_ANGLE -0.000762352625917175242018988971028f
#define TAN_DELTA_ANGLE -0.000762352847449903137950263430428f


// =========== 码盘定位系统 ===========

typedef struct {
    // 用于计算，即每次计算周期的码盘数据变化量
    // e表示码盘(encoder)
    float eDeltaX;
    float eDeltaY;
    float e_vx;
    float e_vy;
//    float e_ax;
//    float e_ay;
    float e_orientate; // 码盘x轴与世界坐标系x轴的夹角
    // 子系统的坐标转换，将码盘非正交坐标系转换为正交坐标系后的全局定位信息
    // r表示机器人坐标系(robot)
    float rDeltaX;
    float rDeltaY;
    float r_vx;
    float r_vy;
//    float r_ax;
//    float r_ay;
    // 世界坐标系坐标值
    float gDeltaX;
    float gDeltaY;
    float gX;
    float gY;
		float eX;
		float eY;
		float rX;
		float rY;
		
		float g_x_robot;//机器人全局坐标 供观看
		float g_y_robot;
		float last_g_x;
		float last_g_y;
} ENCODER_SUB_SYS;

extern ENCODER_SUB_SYS EncoderSubSystem;

void encoderSysInit(ENCODER_SUB_SYS *EncoderSysInfo,ChassisController *ChassisController);
void encoderSysLocate(ENCODER_SUB_SYS *EncoderSysInfo,GyroController *GyroSysInfo);
void encoder2Global(ENCODER_SUB_SYS *EncoderSysInfo);

#endif
/****************** Copyright HUST Robocon Team 2023 *******************/
