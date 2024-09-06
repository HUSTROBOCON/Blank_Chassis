#ifndef GYRO_H_
#define GYRO_H_

/************************* Includes *************************/
#include "main.h"
#include "tim.h"
#include "spi.h"

/************************* Defines *************************/
#define GYRO_RESET_PIN              GPIO_PIN_3
#define GYRO_RESET_PORT             GPIOB
#define GYRO_RESET_CLK              RCC_AHB1Periph_GPIOB

#define GYRO_SYNC_PIN               GPIO_PIN_11
#define GYRO_SYNC_PORT              GPIOB
#define GYRO_SYNC_CLK               RCC_AHB1Periph_GPIOB

#define GYRO_RDY_PIN                GPIO_PIN_5
#define GYRO_RDY_PORT               GPIOC
#define GYRO_RDY_CLK                RCC_AHB1Periph_GPIOC

#define IS_ADC_RDY  HAL_GPIO_ReadPin(GYRO_RDY_PORT,GYRO_RDY_PIN)

#define GYRO_IF_DIR_INVERSE 1

#define GYRO_INVALIDE_TIME 100
#define GYRO_INTEGRAL_TIME 500
#define GYRO_RATE_DEAD 0.1f // 0.04f

#define ADC_CORRECT_COEFF 1.0f // 0.99907636507977900000f//0.99866465297231400000f//0.99918984921670100000f
#define ADC_FULL_SCALE_MV 8192.0f
#define ANALOG_GAIN 0.2f
#define ADC_FULL_DIGITAL 0XFFFFFF
#define GYRO_DEG_PER_SECOND_2MV 46.7f

#define ADC_COEFF                           \
    (ADC_FULL_SCALE_MV / ADC_FULL_DIGITAL / \
     ANALOG_GAIN / GYRO_DEG_PER_SECOND_2MV)

/************************* Exported types *************************/
typedef struct
{
    float Angle;
    float Rate;
    float dAngle;
    float ZeroDriftRate;
} GyroInstance;

typedef struct
{
    GyroInstance Gyro;
    float OriginRate;
    float LastAngle;
    float GyroCoeffNegative[3];
    float GyroCoeffPositive[3];
//	  float CosAngle;
//	  float SinAngle;
} GyroController;

/************************* Exported constants *************************/

/************************* Exported Functions *************************/
GyroController *GetGyroControlFlag_Singleton(void);
extern GyroController GyroSystem;

void Gyro_Get_Data(GyroController *);
void Gyro_Init(GyroController*);



#endif

/******************* (C) COPYRIGHT 2017 HUST-Robocon *****END OF FILE**********/
