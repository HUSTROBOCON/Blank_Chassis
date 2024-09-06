/**
 ******************************************************************************
 * @file    Manual_Operation.h
 * @brief   手操控制器结构体定义和相关物理参数定义
 * @verbatim
 * @endverbatim
 * @edit date
 ******************************************************************************
 */

#ifndef MANUAL_OPERATION_H_
#define MANUAL_OPERATION_H_

/************************* Includes *************************/

#include "main.h"
#include "ChassisInit.h"

/************************* Defines *************************/

/* the range of distance between the finger and the Rocker center */
#define ROCKER_CENTER_X 1000.0f
#define ROCKER_CENTER_Y 1000.0f
#define ROCKER_DIS_MAX 800.0f

#define MANUAL_V_MAX 1500.0f
#define MANUAL_w_MAX 110.0f
#define MANUAL_a_MAX 6000.0f
#define MANUAL_aw_MAX 60.0f

/************************* Exported types *************************/

typedef struct
{
    float AttitudeAlignKp;
    float AttitudeAlignw;
    bool IsReleaseRight;
    bool IsReleaseLeft;
} ManualController;



/************************* Exported constants *************************/

/************************* Exported Functions *************************/

void Manual_Controller_Init(ManualController *);

void Manual_Controller_CtrlCmd(ManualController *, ChassisController *);

void Manual_Controller_Reset(ManualController *);

#endif // MANUAL_OPERATION
