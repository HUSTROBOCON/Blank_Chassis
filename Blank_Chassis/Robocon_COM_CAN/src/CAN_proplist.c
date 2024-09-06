#include "can_proplist.h"

CANFunDict g_CAN1_Prop_Array[] = {
	{EXTID_NON_PROTOCOL, ExtID_Non_Protocol_CAN1},
	{STDID_NON_PROTOCOL, StdID_Non_Protocol_CAN1},
  {GET_TRAIN_INFO,Get_Train_Info},
};

CANFunDict g_CAN2_Prop_Array[] = {
  {NULL,NULL},
};

const uint8_t g_CAN1_Prop_Count = sizeof(g_CAN1_Prop_Array) / 
    sizeof(g_CAN1_Prop_Array[0]);
const uint8_t g_CAN2_Prop_Count = sizeof(g_CAN2_Prop_Array) / 
    sizeof(g_CAN2_Prop_Array[0]);

