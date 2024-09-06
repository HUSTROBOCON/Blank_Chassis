/**
 ******************************************************************************
 * @file    Trajectory_Analyzer.h
 * @brief   路径分析控制器结构体定义及相关函数声明
 * @verbatim
 * @endverbatim
 * @edit date
 ******************************************************************************
 */

#ifndef TRAJECTORY_ANALYZER_H_
#define TRAJECTORY_ANALYZER_H_

/************************* Includes *************************/

#include "main.h"
#include "ChassisInit.h"
#include "Path.h"
#include "TaskChassis.h"
/************************* Defines *************************/

/************************* Exported types *************************/

typedef struct
{
    int16_t CurrentPointIndex;
    int16_t AimPointIndex;
    int16_t NavigationPathType;
    int16_t NavigationPathNum;
    int16_t MaxPointIndex;
    float Dis;
    NavigationPoints *CurrentPath;
    bool IsFirstNavigation; // used to judge the startup of the Navigation,
                            // you can find it in Chassis_Motor_Drive
    bool IsNavigationFinish;
} TrajectoryAnalyzer;

/************************* Exported constants *************************/
extern TrajectoryAnalyzer g_Trajectory_Analyzer_Instance;
/************************* Exported Functions *************************/

void Trajectory_Analyzer_Init(TrajectoryAnalyzer *);

void Trajectory_Analyzer_Reset(TrajectoryAnalyzer *);

void Trajectory_Analyzer_SetPathNum(TrajectoryAnalyzer *, PathInfoTypedef *,
                                    int16_t, int16_t);

void Trajectory_Analyzer_GetAimPointIndex(TrajectoryAnalyzer *);


#endif // TRAJECTORY_ANALYZER_H_
