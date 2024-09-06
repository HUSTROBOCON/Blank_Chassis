#include "Parking.h"
#include "HUST_Math_Lib.h"
#include "Frame_Change.h"
#include "TaskChassis.h"
#include <arm_math.h>

/**
 * @brief  Two sets of PID structure initialization, Preset Parking range,
 *         Reset Parking Finish Flag
 * @note
 * @param  Self为Parking控制器全局变量
 * @retval None
 */
void Parking_Controller_Init(ParkingController *Self)
{
    Self->TranslationPID.Kd = 0.8f;
    Self->TranslationPID.Ki = 0;
    Self->TranslationPID.LimitIntegral = 800.0f;
    Self->TranslationPID.Integral = 0;
    Self->TranslationPID.PreError = 0;

    Self->RotationPID.Ki = 0.0f;
    Self->RotationPID.Kd = 0.005f;
    Self->RotationPID.LimitIntegral = 0.0f;
    Self->RotationPID.Integral = 0;
    Self->RotationPID.PreError = 0;

    Self->DeltaAlpha = 0.0f;
    Self->DeltaX = 0.0f;
    Self->DeltaY = 0.0f;
    Self->Dis = 0.0f;
	  Self->Orientation = 0.0f;

    Self->PosResolution = 5.0f;
    Self->PoseResolution = 0.5f;

    Self->IsParkingFinish = false;
    Self->Period = 1;
		
		Self->LinearVelocity = 0.0f;
		Self->AngualrVelocity = 0.0f;
		
		Self->Minimum_Speed = 50.0f;
		Self->UnderLimit = 0;
}

/**
 * @brief  Parking控制器中姿态，移动三组PID参数清零
 * @note
 * @param  Self为Parking控制器全局变量
 * @retval None
 */
void Parking_Controller_Reset(ParkingController *Self)
{
    Self->TranslationPID.Integral = 0.0f;
    Self->TranslationPID.PreError = 0.0f;
    Self->RotationPID.Integral = 0.0f;
    Self->RotationPID.PreError = 0.0f;

    Self->DeltaAlpha = 0.0f;
    Self->DeltaX = 0.0f;
    Self->DeltaY = 0.0f;
    Self->Dis = 0.0f;
	  Self->Orientation = 0.0f;

    Self->PosResolution = 5.0f;
    Self->PoseResolution = 5.0f;

    Self->IsParkingFinish = false;
    Self->Period = 1;
	
		Self->LinearVelocity = 0.0f;
		Self->AngualrVelocity = 0.0f;
}

/**
 * @brief  更新位置和姿态误差
 * @note
 * @param  Self为Parking控制器全局变量
 * @retval None
 */
void Parking_Controller_Update(ParkingController *Self,
                               ChassisController *Chassis_Controller)
{
    Self->DeltaX = Chassis_Controller->AimState.X -
                   Chassis_Controller->CurrentState.X;
    Self->DeltaY = Chassis_Controller->AimState.Y -
                   Chassis_Controller->CurrentState.Y;
    Self->Dis = sqrtf(powf(Self->DeltaX, 2) + powf(Self->DeltaY, 2));
    Self->Orientation = FastTableAtan2(Self->DeltaY, Self->DeltaX);
    Self->DeltaAlpha = Chassis_Controller->AimState.Alpha -
                       Chassis_Controller->CurrentState.Alpha;
    Self->DeltaAlpha = NormalizeAngle(Self->DeltaAlpha * DEG2RAD) * RAD2DEG;
}

/**
 * @brief  位置和姿态校正采用分段式PID
 * @note
 * @param  Self为Parking控制器全局变量
 * @retval None
 */
static void Linear_Parking_DecidePidParam(ParkingController *Self)
{
		float Interval[4] = {5.0f, 10.0f, 100.0f, 400.0f};
		float Kp[5] = {5.0f, 5.0f, 4.0f, 2.0f, 1.0f};
    if (Self->Dis < Interval[0])
    {
        Self->TranslationPID.Kp = Kp[0];
        Self->TranslationPID.LimitOutput = Interval[0] * Kp[1];
    }
    else if (Self->Dis < Interval[1])
    {
        Self->TranslationPID.Kp = Kp[1];
        Self->TranslationPID.LimitOutput = Interval[1] * Kp[2];
    }
    else if (Self->Dis < Interval[2])
    {
        Self->TranslationPID.Kp = Kp[2];
        Self->TranslationPID.LimitOutput = Interval[2] * Kp[3];
    }
    else if (Self->Dis < Interval[3])
    {
        Self->TranslationPID.Kp = Kp[3];
        Self->TranslationPID.LimitOutput = Interval[3] * Kp[4];
    }
    else
    {
        Self->TranslationPID.Kp = Kp[4];
        Self->TranslationPID.LimitOutput = 1000.0f;
    }
}

float Interval_R[4] = {0.5f, 1.5f, 4.0f, 15.0f};
float Kp_R[5] = {5.5f, 4.5f, 3.5f, 2.5f, 1.5f};
static void Rotate_Parking_DecidePidParam(ParkingController *Self)
{
    if (fabs(Self->DeltaAlpha) < Interval_R[0])
    {
        Self->RotationPID.Kp = Kp_R[0];
        Self->RotationPID.LimitOutput = Interval_R[0] * Kp_R[1];
    }
    else if (fabs(Self->DeltaAlpha) < Interval_R[1])
    {
        Self->RotationPID.Kp = Kp_R[1];
        Self->RotationPID.LimitOutput = Interval_R[1] * Kp_R[2];
    }
    else if (fabs(Self->DeltaAlpha) < Interval_R[2])
    {
        Self->RotationPID.Kp = Kp_R[2];
        Self->RotationPID.LimitOutput = Interval_R[2] * Kp_R[3];
    }
    else if (fabs(Self->DeltaAlpha) < Interval_R[3])
    {
        Self->RotationPID.Kp = Kp_R[3];
        Self->RotationPID.LimitOutput = Interval_R[3] * Kp_R[4];
    }
    else
    {
        Self->RotationPID.Kp = Kp_R[4];
        Self->RotationPID.LimitOutput = 100.0f;
    }
}

/**
 * @brief  位置校正
 * @note
 * @param  Self为Parking控制器全局变量
 * @retval None
 */
void Linear_Parking_CtrlCmd(ParkingController *Self,
                            ChassisController *Chassis_Controller)
{
		float Steer_Wheel[WHEEL_NUM] = {-135.0f, 135.0f, 45.0f,-45.0f};
    float Global_Steer_Wheel[WHEEL_NUM] = {0.0f, 0.0f, 0.0f,0.0f};
    float Vx_Motor[WHEEL_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};
    float Vy_Motor[WHEEL_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};
    float Linear_Velocity = 0.0f;
		float V_X = 0.0f, V_Y = 0.0f;
    Linear_Parking_DecidePidParam(Self);

    Linear_Velocity = Pid_Regulate(0.0f, -Self->Dis, &(Self->TranslationPID));
		V_X = Linear_Velocity * arm_cos_f32(Self->Orientation);
    V_Y = Linear_Velocity * OptimizeArm_sin_f32(Self->Orientation);
		
		Self->LinearVelocity = Linear_Velocity;
		Chassis_Controller->AimState.Vx = V_X;
		Chassis_Controller->AimState.Vy = V_Y;
    for (int i = 0; i < WHEEL_NUM; i++)
    {

        Vx_Motor[i] =  Chassis_Controller->AimState.Vx;
        Vy_Motor[i] =  Chassis_Controller->AimState.Vy;

        Chassis_Controller->AimState.HelmAngle[i] = FastTableAtan2(Vy_Motor[i], Vx_Motor[i]) * RAD2DEG;
			
        Chassis_Controller->AimState.WheelSpeed[i] = sqrtf(powf(Vx_Motor[i], 2) + powf(Vy_Motor[i], 2));
    }
}

/**
 * @brief  姿态校正
 * @note
 * @param  Self为Parking控制器全局变量
 * @retval None
 */
void Rotate_Parking_CtrlCmd(ParkingController *Self,
                            ChassisController *Chassis_Controller)
{
		float Steer_Wheel[WHEEL_NUM] = {-135.0f, 135.0f, 45.0f,-45.0f};
    float Global_Steer_Wheel[WHEEL_NUM] = {0.0f, 0.0f, 0.0f,0.0f};
    float Vx_Motor[WHEEL_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};
    float Vy_Motor[WHEEL_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};
    float Angular_Velocity = 0.0f;
    float k = 0.0f;
    Rotate_Parking_DecidePidParam(Self);

    Angular_Velocity =
        Pid_Regulate(0.0f, -Self->DeltaAlpha, &(Self->RotationPID)) ;
		Self->AngualrVelocity = Angular_Velocity;
    Chassis_Controller->AimState.w = Angular_Velocity;
	
		k = Chassis_Controller->AimState.w * DEG2RAD * DISTANCE_WHEEL2CENTER;

    for (int i = 0; i < WHEEL_NUM; i++)
    {
        Global_Steer_Wheel[i] = RobotAngle2GlobalAngle(Steer_Wheel[i] * DEG2RAD);

        Vx_Motor[i] = k * arm_cos_f32(Global_Steer_Wheel[i]);
        Vy_Motor[i] = k * OptimizeArm_sin_f32(Global_Steer_Wheel[i]);

        Chassis_Controller->AimState.HelmAngle[i] = FastTableAtan2(Vy_Motor[i], Vx_Motor[i]) * RAD2DEG;
			
        Chassis_Controller->AimState.WheelSpeed[i] = sqrtf(powf(Vx_Motor[i], 2) + powf(Vy_Motor[i], 2));
    }
}

/**
 * @brief  Parking至目标点
 * @note
 * @param  Self为Parking控制器全局变量
 * @retval 更新底盘全局变量中的目标舵角和目标轮速
 */
void Parking_Controller_CtrlCmd(ParkingController *Self,
                                ChassisController *Chassis_Controller)
{
		Parking_Controller_Update(Self,Chassis_Controller);
		switch(Self->Period)
		{
			case 1:
			{
				if(Self->Dis <= Self->PosResolution)
				{
					Check_Speed(Self,Chassis_Controller);
					if(Self->UnderLimit)
					{
						Self->Period = 2;
						Chassis_Controller->AimState.Vx = 0.0f;
						Chassis_Controller->AimState.Vy = 0.0f;
					}
					for (int i = 0; i < WHEEL_NUM; i++)
						Chassis_Controller->AimState.WheelSpeed[i] = 0.0f;
				}
				else
					Linear_Parking_CtrlCmd(Self, Chassis_Controller);
			}
			break;
			case 2:
			{
				if(fabs(Self->DeltaAlpha) <= Self->PoseResolution)
				{
					Check_Speed(Self,Chassis_Controller);
					if(Self->UnderLimit)
					{
						Self->Period = 3;
						Chassis_Controller->AimState.w = 0.0f;
					}
					for (int i = 0; i < WHEEL_NUM; i++)
						Chassis_Controller->AimState.WheelSpeed[i] = 0.0f;
				}
				else
					Rotate_Parking_CtrlCmd(Self, Chassis_Controller);
			}
			break;
			case 3:
			{
				if (Self->Dis <= Self->PosResolution &&
            fabs(Self->DeltaAlpha) <= Self->PoseResolution)
				{
					Check_Speed(Self,Chassis_Controller);
					if(Self->UnderLimit)
					{
						Self->IsParkingFinish = true;
					}
					for (int i = 0; i < WHEEL_NUM; i++)
						Chassis_Controller->AimState.WheelSpeed[i] = 0.0f;
				}
				else
				{
					if (Self->Dis >= Self->PosResolution)
                Self->Period = 1;
            else
                Self->Period = 2;
						Self->IsParkingFinish = false;
				}
			}
			break;
		}
		
		
}

/**
 * @brief  底盘锁死
 * @note
 * @param
 * @retval 更新底盘全局变量中的目标点和目标轮速，向底盘发送锁死命令
 */

void Check_Speed(ParkingController *Self, ChassisController *Chassis_Controller)
{
    bool Under_limit = 1;
		for (int i = 0; i < WHEEL_NUM; i++)
    {
			Under_limit &=(fabs(Chassis_Controller->CurrentState.WheelSpeed[i]) < Self->Minimum_Speed);
		}
		Self->UnderLimit = Under_limit;
}
