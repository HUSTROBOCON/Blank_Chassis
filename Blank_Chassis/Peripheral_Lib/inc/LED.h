#ifndef LED_H_
#define LED_H_

/************************* Includes *************************/
#include "main.h"
/************************* Defines *************************/
#define LEFT_LED                     LED1
#define MIDDLE_LED                   LED2
#define RIGHT_LED                    LED3

/************************* Exported types *************************/
typedef enum {
	LED1,
	LED2,
	LED3,
} LEDTypeDef;
/************************* Exported constants *************************/

/************************* Exported Functions *************************/
void LED_Init(void);
void LED_On(LEDTypeDef LED);
void LED_Off(LEDTypeDef LED);
void LED_Toggle(LEDTypeDef LED);

#endif

/******************* (C) COPYRIGHT 2014 HUST-Robocon *****END OF FILE**********/
