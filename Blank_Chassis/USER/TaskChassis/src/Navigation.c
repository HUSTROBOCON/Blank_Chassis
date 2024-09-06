/**
 ******************************************************************************
 * @file    Navigation.c
 * @brief   轨迹跟踪功能相关函数，包含以下内容：
 *          - 控制器初始化函数
 *          - 分段PID参数设置函数
 *          - 偏航校正计算函数
 *          - 姿态校正计算函数
 * @verbatim
 * @endverbatim
 * @edit date
 ******************************************************************************
 */

#include "Navigation.h"
#include "HUST_Math_Lib.h"
#include "TaskChassis.h"
#include <arm_math.h>
/**
 * @brief  导航控制器中姿态，偏航，移动三组PID结构初始化
 * @note
 * @param  Self为导航控制器全局变量
 * @retval None
 */
void NaviController_Init(NaviController *Self)
{
    Self->LateralPosDiviationPid.Ki = 0;
    Self->LateralPosDiviationPid.Kd = 0;
    Self->LateralPosDiviationPid.LimitIntegral = 0;
    Self->LateralPosDiviationPid.PreError = 0.0;
    Self->LateralPosDiviationPid.Integral = 0;

    Self->PoseDiviationPid.Kp = 3.0f;
    Self->PoseDiviationPid.Ki = 0.0;
    Self->PoseDiviationPid.Kd = 0.05f;
    Self->PoseDiviationPid.LimitOutput = 20.0f * DEG2RAD;
    Self->PoseDiviationPid.LimitIntegral = 0.0f;
    Self->PoseDiviationPid.Integral = 0;
    Self->PoseDiviationPid.PreError = 0;
    Self->RunParameter.LinearSpeed = 0.0f;
    Self->RunParameter.LinearSteer = 0.0f;
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        Self->RunParameter.RotateSpeed[i] = 0.0f;
        Self->RunParameter.RotateSteer[i] = 0.0f;
    }
    Self->LateralError = 0.0f;
    Self->LateralV = 0.0f;
    Self->AlphaError = 0.0f;
    Self->Alphaw = 0.0f;
}

/**
 * @brief  导航控制器中姿态，偏航，移动三组PID参数清零
 * @note   每次目标点切换时需要运行一次
 * @param  Self为导航控制器全局变量
 * @retval None
 */
void NaviController_Reset(NaviController *Self)
{
    Self->LateralPosDiviationPid.Integral = 0.0f;
    Self->LateralPosDiviationPid.PreError = 0.0f;
    Self->PoseDiviationPid.Integral = 0.0f;
    Self->PoseDiviationPid.PreError = 0.0f;

    Self->RunParameter.LinearSpeed = 0.0f;
    Self->RunParameter.LinearSteer = 0.0f;
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        Self->RunParameter.RotateSpeed[i] = 0.0f;
        Self->RunParameter.RotateSteer[i] = 0.0f;
    }
    Self->LateralError = 0.0f;
    Self->LateralV = 0.0f;
    Self->AlphaError = 0.0f;
    Self->Alphaw = 0.0f;
}

/**
 * @brief  位置和姿态校正采用分段式PID
 * @note
 * @param  Self为Parking控制器全局变量
 * @retval None
 */
static void Lateral_DecidePidParam(NaviController *Self)
{
    float Interval[3] = {100.0f, 400.0f, 1200.0f};
    float Kp[4] = {7.0f, 5.5f, 2.5f, 3.0f};
		//float Kp[4] = {7.0f, 4.8f, 6.0f, 3.0f};
    if (fabs(Self->LateralError) < Interval[0])
    {
        Self->LateralPosDiviationPid.Kp = Kp[0];
        Self->LateralPosDiviationPid.LimitOutput = Interval[0] * Kp[1];
    }
    else if (fabs(Self->LateralError) < Interval[1])
    {
        Self->LateralPosDiviationPid.Kp = Kp[1];
        Self->LateralPosDiviationPid.LimitOutput = Interval[1] * Kp[2];
    }
    else if (fabs(Self->LateralError) < Interval[2])
    {
        Self->LateralPosDiviationPid.Kp = Kp[2];
        Self->LateralPosDiviationPid.LimitOutput = 3000.0f;
    }
    else
    {
        Self->LateralPosDiviationPid.Kp = Kp[3];
        Self->LateralPosDiviationPid.LimitOutput = 1500.0f;
    }
}


static void Rotate_DecidePidParam(NaviController *Self)
{
	float Kp[2] = {3.5f, 4.5f};
	//float Kp[2] = {3.0f, 5.0};
	if(fabs(Self->AlphaError)<10.0f)
	{
		Self->PoseDiviationPid.Kp=Kp[0];
		Self->PoseDiviationPid.LimitOutput=70.0f* DEG2RAD;//70.0
	}
	if(fabs(Self->AlphaError)>10.0f)
	{
		Self->PoseDiviationPid.Kp=Kp[1];
		Self->PoseDiviationPid.LimitOutput=150.0f* DEG2RAD;//150.0
	}
}
/**
 * @brief  根据目标点的相关数据计算位置及偏航校正下轮舵的期望数据
 * @note   不要求目标点姿态下使用
 * @param  Self为导航控制器全局变量
 * @retval 更新底盘全局变量中的目标舵角和目标轮速
 */
void Linear_Navigation_CtrlCmd(NaviController *Self,
                               ChassisController *Chassis_Controller)
{
    /* 参考点相关参数 */
    float Ref_X = Chassis_Controller->AimState.X;
    float Ref_Y = Chassis_Controller->AimState.Y;
    float Ref_Theta = Chassis_Controller->AimState.Theta * DEG2RAD;
    float Ref_Vx = Chassis_Controller->AimState.Vx;
    float Ref_Vy = Chassis_Controller->AimState.Vy;

    float Vector_Ref2Robot_X = 0.0f, Vector_Ref2Robot_Y = 0.0f;
    float V_Feedback_Theta = 0.0f;
    float Vx_Feedback_Lateral = 0.0f, Vy_Feedback_Lateral = 0.0f;
    float Vx_Output = 0.0f, Vy_Output = 0.0f;
    float V_Output = 0.0f, Steer_Feedforward = 0.0f;

    /* 偏航矫正计算 */
    Vector_Ref2Robot_X = Chassis_Controller->CurrentState.X - Ref_X;
    Vector_Ref2Robot_Y = Chassis_Controller->CurrentState.Y - Ref_Y;

    Self->LateralError = -Vector_Ref2Robot_X * OptimizeArm_sin_f32(Ref_Theta) +
                         Vector_Ref2Robot_Y * arm_cos_f32(Ref_Theta);

    //偏移距离采用旋转坐标系得到，故PID中需取绝对值保证距离为正值
    Lateral_DecidePidParam(Self);
    Self->LateralV = Pid_Regulate(0.0f, -fabs(Self->LateralError),
                                  &(Self->LateralPosDiviationPid));

    V_Feedback_Theta = Ref_Theta - Self->LateralError /
                                       fabs(Self->LateralError) * PI / 2.0f;
    Vx_Feedback_Lateral = Self->LateralV * arm_cos_f32(V_Feedback_Theta);
    Vy_Feedback_Lateral = Self->LateralV *
                          OptimizeArm_sin_f32(V_Feedback_Theta);

    Vx_Output = Ref_Vx + Vx_Feedback_Lateral;
    Vy_Output = Ref_Vy + Vy_Feedback_Lateral;
    V_Output = sqrtf(Vx_Output * Vx_Output + Vy_Output * Vy_Output);

    Steer_Feedforward = FastTableAtan2(Vy_Output, Vx_Output) * RAD2DEG;
    /* 记录数据 */
    Self->RunParameter.LinearSpeed = V_Output;
    Self->RunParameter.LinearSteer = Steer_Feedforward;
    for (int i = 0; i < WHEEL_NUM; i++)
    {
       Chassis_Controller->AimState.HelmAngle[i] = Steer_Feedforward;
       Chassis_Controller->AimState.WheelSpeed[i] = V_Output;
    }
}

/**
 * @brief  根据目标点的相关数据计算位置，偏航姿态校正下轮舵的期望数据
 * @note   需先进行Linear_Navigation_CtrlCmd运算
 * @param  Self为导航控制器全局变量
 * @retval 更新底盘全局变量中的目标舵角和目标轮速
 */
void Rotate_Navigation_CtrlCmd(NaviController *Self,
                               ChassisController *Chassis_Controller)
{
    /* 参考点相关参数 */
    float Ref_Theta = Chassis_Controller->AimState.Theta * DEG2RAD;
    float Ref_Alpha = Chassis_Controller->AimState.Alpha * DEG2RAD;
    float Current_Alpha = Chassis_Controller->CurrentState.Alpha * DEG2RAD;

    float w_Feedback_Raw = 0.0f;
    float Alpha_Error = 0.0f, Instance_Radius = 0.0f;
    float X_Point_Rotation = 0.0f, Y_Point_Rotation = 0.0f;
    float Rotation_Angle = 0.0f;
    float Dx_Wheel2Center[WHEEL_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};
    float Dy_Wheel2Center[WHEEL_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};
    float Rotation_Angel_Wheel[WHEEL_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};
    float Rotate_Speed_Wheel[WHEEL_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};
    float Rotate_Steer_Wheel[WHEEL_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};

    /* 姿态矫正计算 */
    Alpha_Error = NormalizeAngle(Ref_Alpha - Current_Alpha);
    Self->AlphaError = Alpha_Error * RAD2DEG;
		
		Rotate_DecidePidParam(Self);
    w_Feedback_Raw = Pid_Regulate(0.0f, -Alpha_Error, &(Self->PoseDiviationPid));
    Self->Alphaw = w_Feedback_Raw * RAD2DEG;

		
    /* 计算旋转中心坐标 */
    Instance_Radius = fabs(Self->RunParameter.LinearSpeed / w_Feedback_Raw);
		
    if (Instance_Radius < DISTANCE_WHEEL2CENTER)
    {
        Instance_Radius = DISTANCE_WHEEL2CENTER;
    }

    Rotation_Angle = Self->RunParameter.LinearSteer * DEG2RAD +
                     w_Feedback_Raw / fabs(w_Feedback_Raw) * PI / 2 - Current_Alpha;

    X_Point_Rotation = Instance_Radius * arm_cos_f32(Rotation_Angle);
    Y_Point_Rotation = Instance_Radius * OptimizeArm_sin_f32(Rotation_Angle);

    /* 根据坐标计算旋转角度 */
    Dx_Wheel2Center[0] = WHEEL1_X - X_Point_Rotation;
    Dx_Wheel2Center[1] = WHEEL2_X - X_Point_Rotation;
    Dx_Wheel2Center[2] = WHEEL3_X - X_Point_Rotation;
		Dx_Wheel2Center[3] = WHEEL4_X - X_Point_Rotation;

    Dy_Wheel2Center[0] = WHEEL1_Y - Y_Point_Rotation;
    Dy_Wheel2Center[1] = WHEEL2_Y - Y_Point_Rotation;
    Dy_Wheel2Center[2] = WHEEL3_Y - Y_Point_Rotation;
		Dy_Wheel2Center[3] = WHEEL4_Y - Y_Point_Rotation;
		
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        Rotation_Angel_Wheel[i] = FastTableAtan2(Dy_Wheel2Center[i],
                                                 Dx_Wheel2Center[i]);
			
        Rotate_Speed_Wheel[i] = fabs(w_Feedback_Raw) *
                                sqrtf(powf(Dy_Wheel2Center[i], 2) +
                                      powf(Dx_Wheel2Center[i], 2));
			
        Rotate_Steer_Wheel[i] =
            NormalizeAngle(Rotation_Angel_Wheel[i] - Rotation_Angle + PI) *
            RAD2DEG; //增量

        Self->RunParameter.RotateSpeed[i] = Rotate_Speed_Wheel[i];
        Self->RunParameter.RotateSteer[i] = Rotate_Steer_Wheel[i];

        Chassis_Controller->AimState.HelmAngle[i] += Rotate_Steer_Wheel[i];
        Chassis_Controller->AimState.WheelSpeed[i] = Rotate_Speed_Wheel[i];
    }
    Self->RunParameter.LinearSpeed = 0.0f;
}
