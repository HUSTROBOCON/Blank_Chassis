#include "ChassisInit.h"

void Chassis_Init(ChassisController *Self)
{
	Self->CurrentState.X = CHASSIS_START_X;
  Self->CurrentState.Y = CHASSIS_START_Y;
	Self->CurrentState.Alpha = CHASSIS_START_ALPHA;//0.0f;
  Self->AimState.w = 0.0f;
	
	Self->Mode = STOP;
}