#ifndef ENCODER_H_
#define ENCODER_H_

#include "tim.h"

#define ENCn 2
#define MID_COUNT 32768
/************************* Exported types *************************/
typedef struct
{
	uint16_t 	LastCount;
	uint16_t 	LastReadTime;                                   // unit: us
	int32_t 	Count;
	float 		Distance;										// the Sum of Delta_Distance
	float 		Speed;
	float 		DeltaDistance;									// distance between two reading
} ENCInstance;

typedef struct
{
    ENCInstance Encoder[ENCn];
    float ENCCoeff[ENCn];
    bool ENCDirIfInverse[ENCn];
} ENCController;

typedef enum
{
  	ENC1 = 0,
	ENC2 = 1,
} ENCTypedef;
/************************* Exported constants *************************/

/************************* Exported Functions *************************/
ENCController* GetENCControlFlag_Singleton(void);
void ENCx_Reset	(ENCController*, ENCTypedef);
void ENCx_Read	(ENCController*, ENCTypedef);
void ENC_Init(ENCController *Self);
#endif 

/******************* (C) COPYRIGHT 2017 HUST-Robocon *****END OF FILE**********/
