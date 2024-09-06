#include "LED.h"


void LED_On(LEDTypeDef LED)
{
	if(LED==LEFT_LED)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	if(LED==MIDDLE_LED)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	if(LED==RIGHT_LED)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void LED_Off(LEDTypeDef LED)
{
	if(LED==LEFT_LED)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	if(LED==MIDDLE_LED)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	if(LED==RIGHT_LED)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}


/******************* (C) COPYRIGHT 2014 HUST-Robocon *****END OF FILE**********/
