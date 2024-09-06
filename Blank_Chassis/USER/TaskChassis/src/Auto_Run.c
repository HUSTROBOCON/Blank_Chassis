#include "Auto_Run.h"
#include "HUST_Math_Lib.h"
#include "Frame_Change.h"
#include <arm_math.h>
#include "TaskChassis.h"
#include "Chassis_Motor_Drive.h"

void Auto_Run_Controller_Init(AutoRunController *Self)
{
    Self->TranslationPID.Kd = 0.8f;
    Self->TranslationPID.Ki = 0;
    Self->TranslationPID.LimitIntegral = 800.0f;
    Self->TranslationPID.Integral = 0;
    Self->TranslationPID.PreError = 0;

    Self->RotationPID.Ki = 0.0f;
    Self->RotationPID.Kd = 0.0;
    Self->RotationPID.LimitIntegral = 0.0f;
    Self->RotationPID.Integral = 0;
    Self->RotationPID.PreError = 0;

    Self->DeltaX = 0.0f;
    Self->DeltaY = 0.0f;
    Self->Dis = 0.0f;
    Self->DeltaAlpha = 0.0f;
    Self->LinearVelocity = 0.0f;
    Self->PosResolution = 15.0f;
    Self->PoseResolution = 2.0f;
    Self->Orientation = 0.0f;
    Self->AngualrVelocity = 0.0f;
    Self->IsAutoRunFinish = false;
}

void Auto_Run_Controller_Reset(AutoRunController *Self)
{
    Self->RotationPID.Integral = 0;
    Self->RotationPID.PreError = 0;
    Self->RotationPID.Integral = 0;
    Self->RotationPID.PreError = 0;

    Self->DeltaX = 0.0f;
    Self->DeltaY = 0.0f;
    Self->Dis = 0.0f;
    Self->DeltaAlpha = 0.0f;
    Self->LinearVelocity = 0.0f;
    Self->PosResolution = 20.0f;
    Self->PoseResolution = 2.5f;
    Self->Orientation = 0.0f;
    Self->AngualrVelocity = 0.0f;
    Self->IsAutoRunFinish = false;
}

static void Linear_Run_DecidePidParam(AutoRunController *Self)
{
    static float Interval[2] = {200.0f, 800};
    static float Kp[3] = {5.0f, 3.0f, 2.0f};
    if (Self->Dis < Interval[0])
    {
        Self->TranslationPID.Kp = Kp[0];
        Self->TranslationPID.LimitOutput = 600.0f;
    }
    else if (Self->Dis < Interval[1])
    {
        Self->TranslationPID.Kp = Kp[1];
        Self->TranslationPID.LimitOutput = 600.0f;
    }
    else
    {
        Self->TranslationPID.Kp = Kp[2];
        Self->TranslationPID.LimitOutput = 600.0f;
    }
}

static void Rotate_Run_DecidePidParam(AutoRunController *Self)
{
    if (fabs(Self->DeltaAlpha) < 3.0f)
    {
        Self->RotationPID.Kp = 6.0f;
        Self->RotationPID.LimitOutput = 25.0f;
    }
    else if (fabs(Self->DeltaAlpha) < 15.0f)
    {
        Self->RotationPID.Kp = 5.0f;
			Self->RotationPID.LimitOutput = 100.0f;
    }
    else
    {
        Self->RotationPID.Kp = 4.0f;
        Self->RotationPID.LimitOutput = 135.0f;
    }
}

void Auto_Run_Controller_CtrlCmd(AutoRunController *Self,
                                 ChassisController *Chassis_Controller)
{
    float Steer_Wheel[WHEEL_NUM] = {-135.0f, 135.0f, 45.0f,-45.0f};
    float Global_Steer_Wheel[WHEEL_NUM] = {0.0f, 0.0f, 0.0f,0.0f};
    float Vx_Motor[WHEEL_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};
    float Vy_Motor[WHEEL_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};
    float V_X = 0.0f, V_Y = 0.0f;
    float k = 0.0f;

    Self->DeltaX = Chassis_Controller->AimState.X -
                   Chassis_Controller->CurrentState.X;
    Self->DeltaY = Chassis_Controller->AimState.Y -
                   Chassis_Controller->CurrentState.Y;
    Self->Dis = sqrtf(powf(Self->DeltaX, 2) + powf(Self->DeltaY, 2));
    Self->Orientation = FastTableAtan2(Self->DeltaY, Self->DeltaX);
    Self->DeltaAlpha = Chassis_Controller->AimState.Alpha -  Chassis_Controller->CurrentState.Alpha;
    Self->DeltaAlpha = NormalizeAngle(Self->DeltaAlpha * DEG2RAD) * RAD2DEG;

    if (Self->Dis < Self->PosResolution &&
        fabs(Self->DeltaAlpha) < Self->PoseResolution)
    {
        Self->IsAutoRunFinish = true;
        for (int i = 0; i < WHEEL_NUM; i++)
            Chassis_Controller->AimState.WheelSpeed[i] = 0.0f;
        return;
    }

    Linear_Run_DecidePidParam(Self);
    Rotate_Run_DecidePidParam(Self);

    Self->LinearVelocity =
        Pid_Regulate(0.0f, -Self->Dis, &(Self->TranslationPID));
		
    V_X = Self->LinearVelocity * arm_cos_f32(Self->Orientation);
    V_Y = Self->LinearVelocity * OptimizeArm_sin_f32(Self->Orientation);

    Self->AngualrVelocity =
        Pid_Regulate(0.0f, -Self->DeltaAlpha, &Self->RotationPID);
		
    k = Self->AngualrVelocity * DEG2RAD * DISTANCE_WHEEL2CENTER;

    for (int i = 0; i < WHEEL_NUM; i++)
    {
        Global_Steer_Wheel[i] = RobotAngle2GlobalAngle(Steer_Wheel[i] * DEG2RAD);

        Vx_Motor[i] = k * arm_cos_f32(Global_Steer_Wheel[i]) + V_X;
        Vy_Motor[i] = k * OptimizeArm_sin_f32(Global_Steer_Wheel[i]) + V_Y;

        Chassis_Controller->AimState.HelmAngle[i] = FastTableAtan2(Vy_Motor[i], Vx_Motor[i]) * RAD2DEG;
			
        Chassis_Controller->AimState.WheelSpeed[i] = sqrtf(powf(Vx_Motor[i], 2) + powf(Vy_Motor[i], 2));
    }
}
void Chassis_Lock(ChassisController *Chassis_Controller)
{
    float Speed_Wheel[WHEEL_NUM] = {0.0f, 0.0f, 0.0f};
    float Steer_Wheel[WHEEL_NUM] = {135.0f, -135.0f, -45.0f, 45.0f};

    Chassis_Controller->AimState.X = Chassis_Controller->CurrentState.X;
    Chassis_Controller->AimState.Y = Chassis_Controller->CurrentState.Y;
    Chassis_Controller->AimState.Alpha = Chassis_Controller->CurrentState.Alpha;
    Chassis_Controller->AimState.Vx = 0.0f;
    Chassis_Controller->AimState.Vy = 0.0f;
    Chassis_Controller->AimState.w = 0.0f;

    for (int i = 0; i < WHEEL_NUM; i++)
    {
        Chassis_Controller->AimState.WheelSpeed[i] = Speed_Wheel[i];
        Chassis_Controller->AimState.HelmAngle[i] =
            RobotAngle2GlobalAngle(Steer_Wheel[i] * DEG2RAD) * RAD2DEG;
    }
		
		Chassis_Motor_Drive(Chassis_Controller);

    //Stop_Train();
}