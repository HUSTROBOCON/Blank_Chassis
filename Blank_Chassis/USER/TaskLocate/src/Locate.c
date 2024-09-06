#include "main.h"
#include "Gyro.h"
#include "TaskChassis.h"
#include "Locate.h"
#include "Encoder.h"
#include "tim.h"
#include "EncoderSubSys.h"

LOCATE_SYS_Typedef LocateSystem;
GyroController GyroSystem;
ENCController ENCSystem;

uint8_t init_time=0;
void LocateSysInit()
{
	LocateSystem.Alpha=CHASSIS_START_ALPHA;
	LocateSystem.X=CHASSIS_START_X;
	LocateSystem.Y=CHASSIS_START_Y;
	ENC_Init(&ENCSystem);
	if(!init_time)
	{
		init_time=1;
		Gyro_Init(&GyroSystem);
	}
	encoderSysInit(&EncoderSubSystem,&g_Chassis_Instance);
	HAL_TIM_Base_Start_IT(&htim5);
}
void UpdateLocateInfo(ChassisController *Chassis_Controller)
{
	encoderSysLocate(&EncoderSubSystem,&GyroSystem);
	LocateSystem.X=EncoderSubSystem.g_x_robot;
	LocateSystem.Y=EncoderSubSystem.g_y_robot;
	Chassis_Controller->CurrentState.Alpha=LocateSystem.Alpha;
  Chassis_Controller->CurrentState.Vx = LocateSystem.Vx;
  Chassis_Controller->CurrentState.Vy = LocateSystem.Vy;
  Chassis_Controller->CurrentState.X = LocateSystem.X;
  Chassis_Controller->CurrentState.Y = LocateSystem.Y;
  Chassis_Controller->CurrentState.w = LocateSystem.w;
LocateSystem.V=sqrtf(powf(LocateSystem.Vx, 2) + powf(LocateSystem.Vy, 2));
}