/**
 ******************************************************************************
 * @file    Manual_Operation.c
 * @brief   手操函数定义，包含以下内容：
 *          - 手操控制器初始化函数
 *          - 摇杆操作速度融合函数
 *          - 手操状态判断函数
 * @verbatim
 *          此处手操状态判断函数相当于写成了一个子状态机，有点问题，应该和底盘状态机合成
 * @endverbatim
 * @edit date
 ******************************************************************************
 */

#include "Manual_Operation.h"
#include "TaskChassis.h"
#include "Frame_Change.h"
#include <arm_math.h>
#include "HUST_Math_Lib.h"

/**
 * @brief  手操控制器初始化
 * @note
 * @param  Self为控制器全局变量
 * @retval
 */
void Manual_Controller_Init(ManualController *Self)
{
    Self->AttitudeAlignKp = 6.0f;
    Self->AttitudeAlignw = 0.0f;
    Self->IsReleaseLeft = true;
    Self->IsReleaseRight = true;
}

void Manual_Controller_Reset(ManualController *Self)
{
    Self->AttitudeAlignw = 0.0f;
    Self->IsReleaseLeft = true;
    Self->IsReleaseRight = true;
}

/**
 * @brief  速度融合，平移速度和自转速度矢量叠加
 * @note
 * @param  Self为控制器全局变量
 * @retval 更新底盘全局变量中的目标舵角和目标轮速
 */

void Velocity_Merging(ManualController *Self,
                      ChassisController *Chassis_Controller)
{
    float Steer_Wheel[WHEEL_NUM] = {-135.0f, 135.0f, 45.0f,-45.0f};//逆时针转
    float Global_Steer_Wheel[WHEEL_NUM] = {0.0f, 0.0f, 0.0f,0.0f};
    float Vx_Motor[WHEEL_NUM] = {0.0f, 0.0f, 0.0f,0.0f};
    float Vy_Motor[WHEEL_NUM] = {0.0f, 0.0f, 0.0f,0.0f};
    float Delta_Alpha = 0.0f;
    float w = 0.0f;

    Self->AttitudeAlignw = 0.0f;

    if (Chassis_Controller->AimState.w == 0)
    {
        Delta_Alpha = Chassis_Controller->AimState.Alpha -
                      Chassis_Controller->CurrentState.Alpha;
        Delta_Alpha = NormalizeAngle(Delta_Alpha * DEG2RAD) * RAD2DEG;

        if (fabs(Delta_Alpha) > 2.0f && fabs(Delta_Alpha) < 90.0f)
            Self->AttitudeAlignw = Self->AttitudeAlignKp * Delta_Alpha * DEG2RAD;
    }

    w = Chassis_Controller->AimState.w + Self->AttitudeAlignw;

    for (int i = 0; i < WHEEL_NUM; i++)
    {
        Global_Steer_Wheel[i] =  RobotAngle2GlobalAngle(Steer_Wheel[i] * DEG2RAD);

        Vx_Motor[i] = w * DEG2RAD * DISTANCE_WHEEL2CENTER * arm_cos_f32(Global_Steer_Wheel[i]);
			
        Vy_Motor[i] = w * DEG2RAD * DISTANCE_WHEEL2CENTER * OptimizeArm_sin_f32(Global_Steer_Wheel[i]);

        Vx_Motor[i] += Chassis_Controller->AimState.Vx;
			
        Vy_Motor[i] += Chassis_Controller->AimState.Vy;

        if (g_Chassis_Instance.AimState.Vx != 0 ||
            g_Chassis_Instance.AimState.Vy != 0 ||
            g_Chassis_Instance.AimState.w != 0 ||
            Self->AttitudeAlignw != 0)
            Chassis_Controller->AimState.HelmAngle[i] = FastTableAtan2(Vy_Motor[i], Vx_Motor[i]) * RAD2DEG;

						Chassis_Controller->AimState.WheelSpeed[i] = sqrtf(powf(Vx_Motor[i], 2) + powf(Vy_Motor[i], 2));
    }
}

void Manual_Controller_CtrlCmd(ManualController *Self,
                               ChassisController *Chassis_Controller)
{
    if ((Chassis_Controller->AimState.Vx != 0 ||
         Chassis_Controller->AimState.Vy != 0) &&
        Chassis_Controller->AimState.w == 0){
        ;
		}
    else
    {
        Chassis_Controller->AimState.Alpha =
            Chassis_Controller->CurrentState.Alpha;
    }

    Velocity_Merging(Self, Chassis_Controller);
}
