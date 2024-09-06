#include "Encoder.h"

void ENC_Init(ENCController *);

ENCController *GetENCControlFlag_Singleton(void)
{
    static ENCController *volatile g_ENC_control_flag = NULL;

    if (g_ENC_control_flag != NULL)
        return g_ENC_control_flag;
    else
    {
        /* ������������ʼ�� */
        g_ENC_control_flag = (ENCController *)malloc(sizeof(ENCController));
        ENC_Init(g_ENC_control_flag);
        return g_ENC_control_flag;
    }
}
/**
 * @brief  ���̳�ʼ��
 * @param
 * @retval
 */
void ENCx_Reset(ENCController *Self, ENCTypedef ENCx)
{
    ENCInstance *Encoderx = &Self->Encoder[ENCx];
    Encoderx->Count = 0;
	  if(ENCx==0)
      __HAL_TIM_SetCounter(&htim3, MID_COUNT);
		if(ENCx==1)
			__HAL_TIM_SetCounter(&htim4, MID_COUNT);
}

void ENC_Init(ENCController *Self)
{
    for (ENCTypedef i = ENC1; i < ENCn; i++)
    {
        ENCx_Reset(Self, i);
    }
    Self->ENCCoeff[0] = 0.0801688573929426608815616387595f;//0.00976182171792177289824108209691f;//码盘更改
    Self->ENCCoeff[1] = -0.081983963173762197664475996403f;//0.00981136529753523423740014368508f;
    Self->ENCDirIfInverse[0] = 0;//1;
    Self->ENCDirIfInverse[1] = 0;
		__HAL_TIM_SetCounter(&htim3, MID_COUNT);
}

void ENCx_Read(ENCController *Self, ENCTypedef ENCx)
{
    ENCInstance *Encoderx = &Self->Encoder[ENCx];

    uint16_t Now_time, Delta_time; // unit: us
    uint16_t Now_Count;
    int16_t Delta_Count;
    float Distance, Speed, Delta_Distance;

    Now_time = DT_US_COUNT;
	  if(ENCx==0)                                   //这里小改了一下，让码盘每次从MID_COUNT开始记，防止计数越界问题
		{
      Now_Count=__HAL_TIM_GET_COUNTER(&htim3);
			__HAL_TIM_SetCounter(&htim3, MID_COUNT);
		}
		if(ENCx==1)
		{
			Now_Count=__HAL_TIM_GET_COUNTER(&htim4);
			__HAL_TIM_SetCounter(&htim4, MID_COUNT);
		}
    Delta_time = Now_time - Encoderx->LastReadTime;
    Delta_Count = (int16_t)(Now_Count - MID_COUNT);

    Encoderx->LastReadTime = Now_time;
    Encoderx->LastCount = Now_Count;
    Encoderx->Count += Delta_Count;

    if (Delta_time == 0) // protect
        Delta_time = 0xffff;

    Distance = Encoderx->Count * Self->ENCCoeff[ENCx];
    Delta_Distance = Delta_Count * Self->ENCCoeff[ENCx];

    /* the following code is used to calibrate the Encoder */
    /* unit: mm/s */
    Speed = Delta_Distance * 1000000.0f / Delta_time;

    if (Self->ENCDirIfInverse[ENCx])
    {
        Encoderx->Distance = -Distance;
        Encoderx->DeltaDistance = -Delta_Distance;
        Encoderx->Speed = -Speed;
    }
    else
    {
        Encoderx->Distance = Distance;
        Encoderx->DeltaDistance = Delta_Distance;
        Encoderx->Speed = Speed;
    }
		
}

/******************* (C) COPYRIGHT 2019 HUST-Robocon *****END OF FILE**********/
