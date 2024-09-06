#ifndef LOCATE_H_
#define LOCATE_H_

/************************* Includes *************************/
#include "main.h"
#include "Gyro.h"
#include "TaskChassis.h"
#include "Encoder.h"
/************************* Defines *************************/
#define LOCATE_TIME 2000

typedef enum
{
    ENCODER_IMU,
    LASER_IMU,
    LADAR_ENCODER,
    KALMAN_FUNSION
} LOCATE_MODE; // 定位模式

typedef struct
{
    float X; // mm
    float Y;
	  float last_X;
	  float last_Y;
    // X_robot和Y_robot是方向与世界坐标系平行的坐标系（变化量）
    float X_robot;
    float Y_robot;
    float Vx; // mm/s
    float Vy;
	  float V;
    float AccX;
    float AccY;
    float Alpha;      //-pi~pi
    float w;         // rad/s
    float Orientate; // 码盘x轴与世界坐标系x轴的夹角
    LOCATE_MODE Mode;
	  float cos_yaw;
	  float sin_yaw;
} LOCATE_SYS_Typedef;

extern LOCATE_SYS_Typedef LocateSystem;
extern GyroController GyroSystem;
extern ENCController ENCSystem;
/************************* Exported types *************************/

/************************* Exported constants *************************/

/************************* Exported Functions *************************/
void LocateSysInit();
void UpdateLocateInfo(ChassisController *Chassis_Controller);
#endif

/******************* (C) COPYRIGHT 2017 HUST-Robocon *****END OF FILE**********/