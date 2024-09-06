#ifndef TASKCHASSIS_H_
#define TASKCHASSIS_H_

/************************* Includes *************************/
#include "FreeRTOS.h"
#include "LED.h"
#include "main.h"
#include "cmsis_os.h"
#include "task.h"
#include "Trajectory_Analyzer.h"
#include "ChassisInit.h"
/************************* Defines *************************/
#define CHASSIS_START_X  0.0f;
#define CHASSIS_START_Y  0.0f;
#define CHASSIS_START_ALPHA  0.0f;

#define WHEEL_NUM 4
#define WHEEL_PARA 17.576148267524f

#define WHEEL1_MOTOR 1
#define WHEEL2_MOTOR 2 
#define WHEEL3_MOTOR 3
#define WHEEL4_MOTOR 4

#define WHEEL_RADIUS 33.0f              //轮子半径
#define kWHEEL_mms2rpm 0.2893726238f //轮线速度转为轮电机转速，计算：60/(2*PI*r)：r代表轮子半径
#define DISTANCE_WHEEL2CENTER   240.42f

#define WHEEL1_X -170.0f
#define WHEEL1_Y 170.0f //轮1矢量在机器人坐标系中角度：135度
#define WHEEL2_X 170.0f
#define WHEEL2_Y 170.0f //轮2矢量在机器人坐标系中角度：-135度
#define WHEEL3_X 170.0f
#define WHEEL3_Y -170.0f //轮3矢量在机器人坐标系中角度：-45度
#define WHEEL4_X -170.0f
#define WHEEL4_Y -170.0f //轮3矢量在机器人坐标系中角度：45度

#define HELM1_START_POS 135.0f
#define HELM2_START_POS -135.0f
#define HELM3_START_POS -45.0f
#define HELM4_START_POS 45.0f

typedef enum
{
    STOP,
    MANUAL,
    AUTORUN,
    NAVIGATE,
	  PARKING,
} ChassisModeTypedef;

typedef struct
{
    float X;
    float Y;
    float Alpha;
    float Theta;
    float Vx;
    float Vy;
	  float V;
    float w;
    float WheelSpeed[WHEEL_NUM];
    float HelmAngle[WHEEL_NUM];
} ChassisStateTypedef;

// All Informations are under global coordinate
typedef struct
{
    ChassisStateTypedef CurrentState;
    ChassisStateTypedef AimState;
    ChassisModeTypedef Mode;
} ChassisController;



/************************* Exported types *************************/


/************************* Exported constants *************************/
extern ChassisController g_Chassis_Instance;
/************************* Exported Functions *************************/
void TaskLocate();
void TaskChassis();
void TaskMonitor();

#endif