#include "COM_proplist.h"

COMFunDict g_My_USART1_Prop_Array[] = {	
	{NULL,NULL},
};

COMFunDict g_My_USART2_Prop_Array[] = {
	{NULL,NULL},
};

COMFunDict g_My_USART3_Prop_Array[] = {
	{NULL,NULL},
};

COMFunDict g_My_USART4_Prop_Array[] = {
	{RUN,Run},
	{ROBOT_RESET,Reset},
	{GET_AIM_VELOCITY, Update_Aim_Velocity},
  {GET_AIM_ANGULAR_VELOCITY, Update_Aim_Angular_Velocity},
};

const uint8_t g_My_USART1_Prop_Count = sizeof(g_My_USART1_Prop_Array) /
                                       sizeof(g_My_USART1_Prop_Array[0]);
const uint8_t g_My_USART2_Prop_Count = sizeof(g_My_USART2_Prop_Array) /
                                       sizeof(g_My_USART2_Prop_Array[0]);
const uint8_t g_My_USART3_Prop_Count = sizeof(g_My_USART3_Prop_Array) /
                                       sizeof(g_My_USART3_Prop_Array[0]);
const uint8_t g_My_USART4_Prop_Count = sizeof(g_My_USART4_Prop_Array) /
                                       sizeof(g_My_USART4_Prop_Array[0]);

