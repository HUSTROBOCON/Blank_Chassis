/**
  ******************************************************************************
  * @file     TaskChassis 
  * @brief    空白底盘代码:
  *           - 陀螺仪加码盘定位
  *           - autorun、导航、parking、手操
	*						- 通讯协议
  * @verbatim
	*    去除了往届代码中屎山的空白底盘代码，同时也把注释的编码格式统一了
  *    只包含了基本的底盘跑动代码以及陀螺仪码盘定位，四舵轮底盘工程可以直接在这套底盘代码上搭,也可以拿来给学弟练手
	* 	 底盘相关的常量都在TaskChassis.h里，跑之前先建好自己的坐标系，在根据坐标系以及具体底盘参数更改常量
	*    相关硬件配置也要和cubemx里的核对一下
	*		 该工程采用UTF-8编码，之后的注释也要用UTF-8
 	* @endverbatim
	* @edit date
	*	剧冠男 2024.9.6 QQ：2891270021
  ******************************************************************************  
*/ 

#include "FreeRTOS.h"
#include "LED.h"
#include "main.h"
#include "cmsis_os.h"
#include "task.h"
#include "TaskChassis.h"
#include "Locate.h"
#include "ChassisInit.h"
#include "Auto_Run.h"
#include "Chassis_motor_Drive.h"
#include "Manual_Operation.h"
#include "Navigation.h"
#include "Trajectory_Analyzer.h"
#include "Path.h"
#include "Parking.h"

ChassisController g_Chassis_Instance;
AutoRunController g_AutoRun_Controller_Instance;
ManualController g_Manual_Controller_Instance;
NaviController g_NaviController_Instance;
//下面这个定义放在Trajectory_Analyzer.c里了，放在这里傻逼编译器编译不过
//TrajectoryAnalyzer g_Trajectory_Analyzer_Instance;
PathInfoTypedef g_PathInforms;
ParkingController g_Parking_Controller_Instance;

//定位更新放在了中断里，在main.c的203行
void TaskLocate(void *argument)
{
	LocateSysInit();
	while(1)
	{
		vTaskDelay(5/portTICK_RATE_MS);
	}
}


int Path_Type=0,Path_Num=1;
void TaskChassis(void *argument)
{
	Chassis_Init(&g_Chassis_Instance);
	Auto_Run_Controller_Init(&g_AutoRun_Controller_Instance);
	Manual_Controller_Init(&g_Manual_Controller_Instance);
	Trajectory_Analyzer_Init(&g_Trajectory_Analyzer_Instance);
	NaviController_Init(&g_NaviController_Instance);
	Parking_Controller_Init(&g_Parking_Controller_Instance);
	Path_Init(&g_PathInforms);
	while(1)
	{
		switch(g_Chassis_Instance.Mode)
		{
			case STOP :
				Manual_Controller_Reset(&g_Manual_Controller_Instance);
				Auto_Run_Controller_Reset(&g_AutoRun_Controller_Instance);
				Trajectory_Analyzer_Reset(&g_Trajectory_Analyzer_Instance);//轨迹规划的复位
				NaviController_Reset(&g_NaviController_Instance);//导航控制的复位	
				Chassis_Lock(&g_Chassis_Instance);
				break;
			case MANUAL :
				Auto_Run_Controller_Reset(&g_AutoRun_Controller_Instance);
				Trajectory_Analyzer_Reset(&g_Trajectory_Analyzer_Instance);//轨迹规划的复位
				NaviController_Reset(&g_NaviController_Instance);//导航控制的复位	
				Manual_Controller_CtrlCmd(&g_Manual_Controller_Instance,&g_Chassis_Instance);
				break;
			case AUTORUN :
				if(g_AutoRun_Controller_Instance.IsAutoRunFinish)
				{
					Auto_Run_Controller_Reset(&g_AutoRun_Controller_Instance);
				}
				Auto_Run_Controller_CtrlCmd(&g_AutoRun_Controller_Instance,&g_Chassis_Instance);
				break;
				//导航不要直接跑path里的路径
			case NAVIGATE:
				if(g_Trajectory_Analyzer_Instance.IsNavigationFinish)
				{	
					Trajectory_Analyzer_Reset(&g_Trajectory_Analyzer_Instance);//轨迹规划的复位
					NaviController_Reset(&g_NaviController_Instance);//导航控制的复位	
					Trajectory_Analyzer_SetPathNum(&g_Trajectory_Analyzer_Instance,&g_PathInforms,Path_Type,Path_Num);
				}
				else
				{
					Trajectory_Analyzer_GetAimPointIndex(&g_Trajectory_Analyzer_Instance);
					Linear_Navigation_CtrlCmd(&g_NaviController_Instance, &g_Chassis_Instance);//线性位置控制器
					Rotate_Navigation_CtrlCmd(&g_NaviController_Instance, &g_Chassis_Instance);//旋转位姿控制器
				}
				break;
			case PARKING:
				if(g_Parking_Controller_Instance.IsParkingFinish)
				{
					Parking_Controller_Reset(&g_Parking_Controller_Instance);
				}
				Parking_Controller_CtrlCmd(&g_Parking_Controller_Instance,&g_Chassis_Instance);
				break;
		}
		Chassis_Motor_Drive(&g_Chassis_Instance);
		vTaskDelay(5/portTICK_RATE_MS);
	}
}



uint8_t LED_num=0;
uint8_t LED_count=0;
void TaskMonitor(void *argument)
{
	while(1)
	{
		
		LED_Off(LED1);
		LED_Off(LED2);
		LED_Off(LED3);
		if(LED_count>=100)
		{
			LED_num++;
			LED_count=0;
			if(LED_num>=3)
			{
				LED_num=LED1;
			}
		}
		LED_On(LED_num);
		LED_count++;
		vTaskDelay(5/portTICK_RATE_MS);
	}
}