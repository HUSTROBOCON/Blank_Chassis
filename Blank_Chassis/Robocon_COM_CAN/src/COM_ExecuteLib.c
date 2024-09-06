/**
 ******************************************************************************
 * @file    RobotCOM_ExecuteLib.c
 * @author  Robocon
 * @brief   �������ṩ�˴���Ϣ���������Ķ���:
 *           - �Զ����ŶӴ���Э��֡��Ϣ��������
 *  @verbatim
 *          ��������������Ӧ����Ϣ�������������Ӻ������Ӧ��proplist.c����
 *  @endverbatim
 ******************************************************************************
 */
#include "COM_Basic.h"
#include "COM_Proplist.h"
#include "TaskChassis.h"
#include "HUST_Math_Lib.h"
#include "Frame_Change.h"
#include "ChassisInit.h"
#include "Locate.h"
#include "Manual_Operation.h"
#include <arm_math.h>
/*******************************MYUSART1���պ���***********************************/






/*******************************MYUSART2���պ���***********************************/

/*******************************MYUSART3���պ���***********************************/

/*******************************MYUSART4���պ���***********************************/
void Run(COMFrame *Frame)
{
	g_Chassis_Instance.Mode=MANUAL;
}

void Reset(COMFrame *Frame)
{
	LocateSysInit();
	g_Chassis_Instance.Mode=STOP;
}

void Update_Aim_Velocity(COMFrame *Frame)
{
    float Rocker_X = (Frame->Data.int16_ts[1] - ROCKER_CENTER_X);
    float Rocker_Y = -(Frame->Data.int16_ts[0] - ROCKER_CENTER_Y);

    if (fabs(Rocker_X) < 20 && fabs(Rocker_Y) < 20) //死区限制
    {
        g_Chassis_Instance.AimState.Vx = 0.0f;
        g_Chassis_Instance.AimState.Vy = 0.0f;
			  g_Chassis_Instance.AimState.V=0;
    }
    else
    {
			  g_Chassis_Instance.AimState.V=500.0f;
        if (fabs(Rocker_X) < 600.0f)
            g_Chassis_Instance.AimState.Vy = 5.0f / 3.0f * Rocker_X;
        else
            g_Chassis_Instance.AimState.Vy =-(
                -5.0f * Rocker_X + Rocker_X / fabs(Rocker_X) * 3000.0f);

        if (fabs(Rocker_Y) < 600.0f)
            g_Chassis_Instance.AimState.Vx = -5.0f / 3.0f * Rocker_Y;
        else
            g_Chassis_Instance.AimState.Vx = (
                -5.0f * Rocker_Y + Rocker_Y / fabs(Rocker_Y) * 3000.0f);
    }
		
		g_Chassis_Instance.AimState.Theta=FastTableAtan2(g_Chassis_Instance.AimState.Vy,g_Chassis_Instance.AimState.Vx)*RAD2DEG;
		g_Chassis_Instance.AimState.Vx=g_Chassis_Instance.AimState.V*arm_cos_f32(g_Chassis_Instance.AimState.Theta*DEG2RAD);
		g_Chassis_Instance.AimState.Vy=g_Chassis_Instance.AimState.V*OptimizeArm_sin_f32(g_Chassis_Instance.AimState.Theta*DEG2RAD);
		
		
		g_Chassis_Instance.Mode=MANUAL;
}

void Update_Aim_Angular_Velocity(COMFrame *Frame)
{
	
    static float s_k_w = MANUAL_w_MAX / ROCKER_DIS_MAX;//*10;

    float Rocker_X = Frame->Data.int16_ts[0] - ROCKER_CENTER_X;

    if (fabs(Rocker_X) < 10) //��ֹ��Ư
        g_Chassis_Instance.AimState.w = 0;
    else
        g_Chassis_Instance.AimState.w = -s_k_w * Rocker_X;

		g_Chassis_Instance.Mode=MANUAL;
}
