/* (C) COPYRIGHT @Hust-Robocon-Team
 *         ChengJie 2019
 *       309815765@qq.com         */
#include "Chassis_Motor_Drive.h"
#include "CAN_proplist.h"
#include "TaskChassis.h"

void Chassis_Motor_Drive(ChassisController *Chassis_Controller)
{
	Set_Train_Info(Chassis_Controller, 1);
	Set_Train_Info(Chassis_Controller, 2);
	Set_Train_Info(Chassis_Controller, 3);
	Set_Train_Info(Chassis_Controller, 4);
}
