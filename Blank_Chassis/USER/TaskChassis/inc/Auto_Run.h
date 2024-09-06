#ifndef AUTO_RUN_H_
#define AUTO_RUN_H_

/************************* Includes *************************/

#include "main.h"
#include "PID.h"
#include "ChassisInit.h"

/************************* Defines *************************/

/************************* Exported types *************************/

typedef struct
{
    float DeltaX;
    float DeltaY;
    float Dis;
    float DeltaAlpha;
    float LinearVelocity;
    float Orientation;
    float AngualrVelocity;
    float PosResolution;
    float PoseResolution;
    PIDStructTypedef RotationPID;
    PIDStructTypedef TranslationPID;
    bool IsAutoRunFinish;
} AutoRunController;

/************************* Exported constants *************************/
/************************* Exported Functions *************************/

void Auto_Run_Controller_Init(AutoRunController *);

void Auto_Run_Controller_Reset(AutoRunController *);

void Auto_Run_Controller_CtrlCmd(AutoRunController *, ChassisController *);

void Chassis_Lock(ChassisController *Chassis_Controller);
#endif // PARKING_H_
