/**
 ******************************************************************************
 * @file    Navigation.h
 * @brief   导航控制器结构体定义及相关函数声明
 * @verbatim
 * @endverbatim
 * @edit date
 ******************************************************************************
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

/************************* Includes *************************/

#include "main.h"
#include "PID.h"
#include "ChassisInit.h"

/************************* Defines *************************/

/************************* Exported types *************************/

typedef struct
{
    float LinearSteer;
    float LinearSpeed;
    float RotateSteer[WHEEL_NUM];
    float RotateSpeed[WHEEL_NUM];
} NavigationTypedef;

typedef struct
{
    /* position adjust */
    PIDStructTypedef LateralPosDiviationPid;
    /* pose adjust */
    PIDStructTypedef PoseDiviationPid;
    /* run parameter */
    NavigationTypedef RunParameter;
    /* Watch */
    float LateralError;
    float LateralV;
    float AlphaError;
    float Alphaw;
} NaviController;

/************************* Exported constants *************************/

/************************* Exported Functions *************************/

void NaviController_Init(NaviController *);

void NaviController_Reset(NaviController *);

void Linear_Navigation_CtrlCmd(NaviController *, ChassisController *);

void Rotate_Navigation_CtrlCmd(NaviController *, ChassisController *);

static void Rotate_DecidePidParam(NaviController *Self);

#endif // NAVIGATION_H_
