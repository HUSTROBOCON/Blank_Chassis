#ifndef PARKING_H_
#define PARKING_H_

/************************* Includes *************************/

#include "PID.h"
#include "ChassisInit.h"

/************************* Defines *************************/

/************************* Exported types *************************/

typedef struct
{
    float DeltaAlpha;
    float DeltaX;
    float DeltaY;
    float Dis;
		float Orientation;
    float PosResolution;
    float PoseResolution;
    PIDStructTypedef TranslationPID;
    PIDStructTypedef RotationPID;
    int8_t Period;
    bool IsParkingFinish;
		float LinearVelocity;
		float AngualrVelocity;
		float Minimum_Speed;
		bool UnderLimit;
} ParkingController;

/************************* Exported constants *************************/
/************************* Exported Functions *************************/

void Parking_Controller_Init(ParkingController *);

void Parking_Controller_Reset(ParkingController *);

void Parking_Controller_Update(ParkingController *, ChassisController *);

void Linear_Parking_CtrlCmd(ParkingController *, ChassisController *);

void Rotate_Parking_CtrlCmd(ParkingController *, ChassisController *);

void Parking_Controller_CtrlCmd(ParkingController *, ChassisController *);


void Check_Speed(ParkingController *, ChassisController *);

#endif // PARKING_H_
