#include "EncoderSubSys.h"
#include "Encoder.h"
#include "gyro.h"
#include "Locate.h"
#include "main.h"
#include <arm_math.h>
ENCODER_SUB_SYS EncoderSubSystem;

void encoderSysInit(ENCODER_SUB_SYS *EncoderSysInfo,ChassisController *ChassisController)
{
    EncoderSysInfo->eDeltaX = 0;
    EncoderSysInfo->eDeltaY = 0;
    EncoderSysInfo->e_vx = 0;
    EncoderSysInfo->e_vy = 0;
    EncoderSysInfo->e_orientate = ChassisController->CurrentState.Alpha*DEG2RAD+ENCODER_THETA_IN_ROBOT;
    EncoderSysInfo->rDeltaX = 0;
    EncoderSysInfo->rDeltaY = 0;
    EncoderSysInfo->r_vx = 0;
    EncoderSysInfo->r_vy = 0;
    EncoderSysInfo->gDeltaX = 0;
    EncoderSysInfo->gDeltaY = 0;

    EncoderSysInfo->gX = LocateSystem.X + ENCODER_X_IN_ROBOT *  LocateSystem.cos_yaw - ENCODER_Y_IN_ROBOT * LocateSystem.sin_yaw;
    EncoderSysInfo->gY = LocateSystem.Y + ENCODER_X_IN_ROBOT * LocateSystem.sin_yaw + ENCODER_Y_IN_ROBOT *  LocateSystem.cos_yaw;
	  EncoderSysInfo->g_x_robot=CHASSIS_START_X;
	  EncoderSysInfo->g_y_robot=CHASSIS_START_Y;
	  
	  EncoderSysInfo->eX=0;
	  EncoderSysInfo->eY=0;
	  EncoderSysInfo->rX=0;
	  EncoderSysInfo->rY=0;
	  
}

void encoderSysLocate(ENCODER_SUB_SYS *EncoderSysInfo,GyroController *GyroSysInfo)
{
	  ENCController *ENC = &ENCSystem;
    ENCx_Read(ENC, ENC_X);
    ENCx_Read(ENC, ENC_Y);
	  Gyro_Get_Data(GyroSysInfo);

    
    LocateSystem.Alpha+=GyroSysInfo->Gyro.dAngle;
	  LocateSystem.Alpha=NormalizeAngle(LocateSystem.Alpha*DEG2RAD)*RAD2DEG;
	  
	  LocateSystem.cos_yaw=arm_cos_f32(LocateSystem.Alpha*DEG2RAD);
	  LocateSystem.sin_yaw=OptimizeArm_sin_f32(LocateSystem.Alpha*DEG2RAD);
  
    EncoderSysInfo->eDeltaX = ENC->Encoder[ENC_X].DeltaDistance;
    EncoderSysInfo->eDeltaY = ENC->Encoder[ENC_Y].DeltaDistance;
    EncoderSysInfo->e_vx = ENC->Encoder[ENC_X].Speed;
    EncoderSysInfo->e_vy = ENC->Encoder[ENC_Y].Speed;
    
    // 转换为正交坐标系
    EncoderSysInfo->eDeltaX = EncoderSysInfo->eDeltaX;
    EncoderSysInfo->eDeltaY = EncoderSysInfo->eDeltaY / COS_DELTA_ANGLE - EncoderSysInfo->eDeltaX * TAN_DELTA_ANGLE;
    EncoderSysInfo->e_vx = EncoderSysInfo->e_vx;
    EncoderSysInfo->e_vy = EncoderSysInfo->e_vy / COS_DELTA_ANGLE - EncoderSysInfo->e_vx * TAN_DELTA_ANGLE;
		
		EncoderSysInfo->eX+=EncoderSysInfo->eDeltaX;
		EncoderSysInfo->eY+=EncoderSysInfo->eDeltaY;

	// 转换为机器人坐标系 
    EncoderSysInfo->rDeltaX = EncoderSysInfo->eDeltaX * COS_ENCODER_THETA_IN_ROBOT - EncoderSysInfo->eDeltaY * SIN_ENCODER_THETA_IN_ROBOT;
    EncoderSysInfo->rDeltaY = EncoderSysInfo->eDeltaX * SIN_ENCODER_THETA_IN_ROBOT + EncoderSysInfo->eDeltaY * COS_ENCODER_THETA_IN_ROBOT;
    EncoderSysInfo->r_vx = EncoderSysInfo->e_vx * COS_ENCODER_THETA_IN_ROBOT - EncoderSysInfo->e_vy * SIN_ENCODER_THETA_IN_ROBOT;
    EncoderSysInfo->r_vy = EncoderSysInfo->e_vx * SIN_ENCODER_THETA_IN_ROBOT + EncoderSysInfo->e_vy * COS_ENCODER_THETA_IN_ROBOT;
		
		EncoderSysInfo->rX+=EncoderSysInfo->rDeltaX;
		EncoderSysInfo->rY+=EncoderSysInfo->rDeltaY;

    // 转换为与世界坐标系平行的方向
    EncoderSysInfo->gDeltaX = EncoderSysInfo->rDeltaX * LocateSystem.cos_yaw - EncoderSysInfo->rDeltaY *  LocateSystem.sin_yaw;
    EncoderSysInfo->gDeltaY = EncoderSysInfo->rDeltaX *  LocateSystem.sin_yaw + EncoderSysInfo->rDeltaY * LocateSystem.cos_yaw;
		
		LocateSystem.Vx = EncoderSysInfo->r_vx * LocateSystem.cos_yaw - EncoderSysInfo->r_vy *  LocateSystem.sin_yaw;
    LocateSystem.Vy = EncoderSysInfo->r_vx *  LocateSystem.sin_yaw + EncoderSysInfo->r_vy * LocateSystem.cos_yaw;

    LocateSystem.w = GyroSysInfo->Gyro.Rate;
    // 世界坐标系坐标值累加
    EncoderSysInfo->gX += EncoderSysInfo->gDeltaX;
    EncoderSysInfo->gY += EncoderSysInfo->gDeltaY;
		
//		LocateSystem.Alpha+=GyroSysInfo->Gyro.dAngle;
//	  LocateSystem.Alpha=NormalizeAngle(LocateSystem.Alpha*DEG2RAD)*RAD2DEG;
//		
//		LocateSystem.cos_yaw=arm_cos_f32(LocateSystem.Alpha*DEG2RAD);
//	  LocateSystem.sin_yaw=OptimizeArm_sin_f32(LocateSystem.Alpha*DEG2RAD);
		
		encoder2Global(EncoderSysInfo);
		
}

void encoder2Global(ENCODER_SUB_SYS *EncoderSysInfo){
    // 码盘安装位置修正
	  EncoderSysInfo->last_g_x=EncoderSysInfo->g_x_robot;
	  EncoderSysInfo->last_g_y=EncoderSysInfo->g_y_robot;
    EncoderSysInfo->g_x_robot = EncoderSysInfo->gX + ROBOT_X_IN_ENCODER * LocateSystem.cos_yaw - ROBOT_Y_IN_ENCODER * LocateSystem.sin_yaw;
    EncoderSysInfo->g_y_robot = EncoderSysInfo->gY + ROBOT_X_IN_ENCODER * LocateSystem.sin_yaw + ROBOT_Y_IN_ENCODER * LocateSystem.cos_yaw;
    // LocateSystem.X = EncoderSysInfo->gX - ENCODER_X_IN_ROBOT * Imu.cosAngle - ENCODER_Y_IN_ROBOT * Imu.sinAngle;
    // LocateSystem.Y = EncoderSysInfo->gY - ENCODER_X_IN_ROBOT * Imu.sinAngle + ENCODER_Y_IN_ROBOT * Imu.cosAngle;
    // LocateSystem.X = EncoderSysInfo->gX - ENCODER_X_IN_ROBOT;
    // LocateSystem.Y = EncoderSysInfo->gY - ENCODER_Y_IN_ROBOT;
}
