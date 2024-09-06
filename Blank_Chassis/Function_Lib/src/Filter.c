#include "Filter.h"

/************************************Single-Pole Filter*************************************/
void Single_Pole_Init(SinglePoleFilter *Single_Pole)
{
	Single_Pole->Alpha = (1/SAMP_RATE)/(1/SAMP_RATE+1/2/pi/CUTOFF_RATE);
	Single_Pole->FilterOutput = 0;
}

float Single_Pole_Filter(SinglePoleFilter *Single_Pole,float Input)
{
	Single_Pole->FilterOutput = (1 - Single_Pole->Alpha) * 
        Single_Pole->FilterOutput + Single_Pole->Alpha * Input;

	return Single_Pole->FilterOutput;
}

/************************************Kalman Filter*************************************/
void Kalman_Init(KalmanFilter* Kalman)
{
	Kalman->LastX = 0.0f;                           //最优值初值初始化
	Kalman->LastP = 1.0f;                           //误差协方差初值初始化
	Kalman->Q     = 0.001f;                         //过程噪声0.001
	Kalman->R     = 0.05f;                          //观测噪声，ad采样原始数据的方差为0.27
	Kalman->A     = 1.0f;                           //一般均取1，简化模型
	Kalman->H     = 1.0f;
}

float Kalman_filter(KalmanFilter *Kalman,float Input)
{
	/* 预测 */
	Kalman->MidX = Kalman->A * Kalman->LastX;       //由上次最优值预测下次的x[n]，先验估计
	Kalman->MidP = Kalman->LastP + Kalman->Q;       //由上次最小均方差预测下次均方差，先验估计
	
	/* 校正 */
	Kalman->Kg    = Kalman->MidP / (Kalman->MidP + Kalman->R);                           //卡尔曼增益计算
	Kalman->NowX  = Kalman->MidX + (Kalman->Kg * (Input - (Kalman->H * Kalman->MidX)));  //后验计算最优值
	Kalman->NowP  = (1.0f - Kalman->Kg) * Kalman->MidP;                                  //更新误差协方差
	Kalman->LastP = Kalman->NowP;
	Kalman->LastX = Kalman->NowX;
	
	return Kalman->NowX;
}

/************************************Mean Filter*************************************/
float MEAN_filter(float *data)
{
	int i;
	long sum=0;
	float max=data[0],min=data[0];
	
	for(i=0;i < sizeof(data)/sizeof(data[0]);i++)
	{
		sum += data[i];
		if(max < data[i])
		{
			max = data[i];
		}
		else if(min > data[i])
		{
			min = data[i];
		}
	}
	return (sum-max-min)/((sizeof(data)/sizeof(data[0])-2)*1.0);
}

/************************************Second_order Filter*************************************/
void SECOND_ORDER_init(second_order_struct* second_struct)
{
	second_struct->fs = SAMP_RATE;
	second_struct->f0 = CUTOFF_RATE;
	second_struct->den = (second_struct->fs)*(second_struct->fs) + 6*pi*(second_struct->f0)*(second_struct->fs) + 4*pi*pi*(second_struct->f0)*(second_struct->f0);
	second_struct->a = 4*pi*pi*(second_struct->f0)*(second_struct->f0)/(second_struct->den);
	second_struct->b = (2*(second_struct->fs)*(second_struct->fs) + 6*pi*(second_struct->fs)*(second_struct->f0))/(second_struct->den);
	second_struct->c = (second_struct->fs)*(second_struct->fs)/(second_struct->den);
}

float SECOND_ORDER_filter(second_order_struct* second_struct,float input)
{
	second_struct->now_output=(second_struct->a)*input+(second_struct->b)*(second_struct->last_output)-(second_struct->c)*(second_struct->pre_last_output);
	second_struct->pre_last_output=second_struct->last_output;
	second_struct->last_output=second_struct->now_output;
	
	return second_struct->now_output;
}

/************************************IIR Filter*************************************/
void IIR_Filter_Init(IIR_struct* IIR_struct)
{
	uint32_t i=1024;
	
	for(i=0;i<1024;i++)
	{
		IIR_struct->testin [i] = 1.2f*arm_sin_f32(2*PI*5*i/1000)+arm_sin_f32(2*PI*100*i/1000);
		IIR_struct->testout[i] = 0.0f;
	}
	IIR_struct->Scalevalue1 = 0.9892959049403619f;
	IIR_struct->Scalevalue2 = 0.97132449733676085f;
	IIR_struct->Scalevalue3 = 0.95654322555687676f;
	IIR_struct->Scalevalue4 = 0.94608307884220311f;
	IIR_struct->Scalevalue5 = 0.94067508119380239f;
	float32_t temp[5*IIRn] = {
		1.0f , -2.0f , 1.0f , 1.9766377324327391f , -0.98054588732870851f,        
		1.0f , -2.0f , 1.0f , 1.94073041471636f   , -0.94456757463068353f,        
		1.0f , -2.0f , 1.0f , 1.9111970674260732f , -0.91497583480143385f,        
		1.0f , -2.0f , 1.0f , 1.8902974350918498f , -0.89403488027696243f,        
		1.0f , -2.0f , 1.0f , 1.8794921217822989f , -0.8832082029929107f 
	};
	for(int i = 0;i < 5*IIRn;i++)
	{
		IIR_struct->IIRCoeffsHP[i] = temp[i];
	}
	/*初始化*/
	arm_biquad_cascade_df2T_init_f32(&IIR_struct->SFilter, IIRn, (float32_t *)&IIR_struct->IIRCoeffsHP[0],
		(float32_t*)&IIR_struct->IIRState[0]);
}

/*********************************************************************************************************
*        函数名  : arm_iir_f32_hp
*        功能说明:调用arm_iir_f32_hp实现高通滤波
*        形    参:输入,输出信号,信号采样点数
*        返回值  :无 
*********************************************************************************************************/
void IIR_Filter(IIR_struct* IIR_struct, float32_t *SignalIn,float32_t *SignalOut,uint32_t Sample_Lenth)
{
	uint32_t i;
	
	float32_t  ScaleValue = IIR_struct->Scalevalue1 * IIR_struct->Scalevalue2 * IIR_struct->Scalevalue3 *
		IIR_struct->Scalevalue4 * IIR_struct->Scalevalue5;
	
	/*IIR滤波*/
	
	arm_biquad_cascade_df2T_f32(&IIR_struct->SFilter, SignalIn, SignalOut, Sample_Lenth);
	
	/*放缩系数*/
	
	for(i=0;i<Sample_Lenth;i++)
	{
		SignalOut[i] = SignalOut[i]*ScaleValue;
	}
	
}


