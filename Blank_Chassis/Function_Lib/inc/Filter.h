#ifndef __FILTER_H
#define __FILTER_H

#include "stm32f4xx.h"
#include "stdlib.h"
#include "arm_math.h"

#define    SAMP_RATE       860.0f   				//采样频率
#define    CUTOFF_RATE     200.0f  					//截止频率
#define    pi              3.14159265359f

/************************************�˲����ṹ��************************************/
/************************************Single-Pole Filter************************************/
typedef struct {
	float Alpha;
	float FilterOutput;
} SinglePoleFilter;

/************************************Kalman Filter************************************/
typedef struct {
	float Q;      									//过程噪声最小协方差，反应系统模型建立误差
	float R;     									//测量噪声最小协方差
	float LastX,MidX,NowX;  						//上次的最优值，本次先验估计值，本次最优值
	float LastP,MidP,NowP;  						//上次的误差协方差，本次先验误差协方差，本次误差协方差
	float Kg;            							//卡尔曼增益
	float A;             							//x(n)=Ax(n-1)+Bu(n-1)+w(n-1)  	
													//A为系统转移矩阵，w(n)为过程噪声
	float H;             							//z(n)=Hx(n)+v(n)
													//z(n)为采样实测值，x(n)为实际值，H为观测转移矩阵，v(n)为测量噪声
}KalmanFilter;

/************************************Second_order Filter************************************/
typedef struct {
	unsigned int fs;         								//采样频率
	unsigned int f0;         								//截止频率
	float den;
	float a;                 								//x[n]系数
	float b;                 								//y[n]系数
	float c;                 								//y[n-1]系数
	float now_output;        								//y[n]
	float last_output;       								//y[n-1]
	float pre_last_output;   								//y[n-2]
}second_order_struct;

/************************************IIR Filter*************************************/
#define IIRn             								5  //二阶滤波器个数

typedef struct
{
	float 		testin[1024];
	float 		testout[1024];   //采样频率1KHZ,截至频率15HZ
	float32_t  	Scalevalue1;
	float32_t  	Scalevalue2;
	float32_t  	Scalevalue3;
	float32_t  	Scalevalue4;
	float32_t  	Scalevalue5;
	arm_biquad_cascade_df2T_instance_f32 SFilter;
	float32_t  	IIRState[2*IIRn];
	float32_t  	IIRCoeffsHP[5*IIRn];
}IIR_struct;

void   Single_Pole_Init(SinglePoleFilter*);
float  Single_Pole_Filter(SinglePoleFilter*, float);
void   Kalman_Init(KalmanFilter*);
float  Kalman_filter(KalmanFilter*,float);

float  MEAN_filter(float * );
void   SECOND_ORDER_init(second_order_struct* second_struct);
float  SECOND_ORDER_filter(second_order_struct* second_struct,float input);
void   IIR_Filter_Init(IIR_struct* IIR_struct);
void   IIR_Filter(IIR_struct* IIR_struct, float32_t *SignalIn,float32_t *SignalOut,uint32_t Sample_Lenth);

#endif
