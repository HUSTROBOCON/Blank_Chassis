#include "stdio.h"
#include "can_basic.h"
#include "can.h"
#include "string.h"
#include "can_proplist.h"
#include "gpio.h"

CANFrame g_CAN1_Frame_tsm;
CANFrame g_CAN1_Frame_rcv;
CANFrame g_CAN2_Frame_tsm;
CANFrame g_CAN2_Frame_rcv;
void filter1_init(void)
{
	CAN_FilterTypeDef filter1;
    filter1.SlaveStartFilterBank = 14;
	filter1.FilterBank           = 0;                      //滤波器编号
	filter1.FilterMode           = CAN_FILTERMODE_IDMASK;  //掩码模式
	filter1.FilterScale          = CAN_FILTERSCALE_32BIT;
	filter1.FilterIdHigh         = 0x0000;
	filter1.FilterIdLow          = 0x0000;
	filter1.FilterMaskIdHigh     = 0x0000;
	filter1.FilterMaskIdLow      = 0x0000;
	filter1.FilterFIFOAssignment = CAN_FILTER_FIFO0;       //FIFO0
	filter1.FilterActivation     = CAN_FILTER_ENABLE;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&filter1)!=HAL_OK)
	{
		Error_Handler();
	}
}
void filter2_init(void)
{
	CAN_FilterTypeDef filter2;
    filter2.SlaveStartFilterBank = 14;
	filter2.FilterBank           = 14;                     //滤波器编号
	filter2.FilterMode           = CAN_FILTERMODE_IDMASK;  //掩码模式
	filter2.FilterScale          = CAN_FILTERSCALE_32BIT;
	filter2.FilterIdHigh         = 0x0000;
	filter2.FilterIdLow          = 0x0000;
	filter2.FilterMaskIdHigh     = 0x0000;
	filter2.FilterMaskIdLow      = 0x0000;
	filter2.FilterFIFOAssignment = CAN_FILTER_FIFO1;       //FIFO1
	filter2.FilterActivation     = ENABLE;
	
	if(HAL_CAN_ConfigFilter(&hcan2,&filter2)!=HAL_OK)
	{
		Error_Handler();
	}
}

void CAN1_Start(void)
{
	filter1_init();//滤波器初始化
    if((HAL_CAN_Start(&hcan1))==HAL_OK)//启动CAN1
	{
		HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//使能中断
	}
}
void CAN2_Start(void)
{
	filter2_init();//滤波器初始化
    if((HAL_CAN_Start(&hcan2))==HAL_OK)//启动CAN2
	{
		HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);//使能中断
	}
}

void Send_Frame_CAN(CANFrame* Frame_Send,int canx) 
{
    CAN_TxHeaderTypeDef TxMessage;
    uint32_t  Transmit_Mailbox;
	  uint8_t Transmit_Staus = HAL_ERROR;
	  uint8_t *Data_Send;
    uint16_t Time_Out_Count   = 0;
	  Data_Send  = &(Frame_Send->Data.uchars[0]);
    if (Frame_Send->IDE == CAN_ID_EXT)
    {
			  Frame_Send->Id.all=Frame_Send->Id.ExtID.DesDeviceId+(Frame_Send->Id.ExtID.Property<<8)+(Frame_Send->Id.ExtID.SrcDeviceId<<16)+(Frame_Send->Id.ExtID.Priority<<24)+(Frame_Send->Id.ExtID.Permit<<28);
        TxMessage.ExtId = Frame_Send->Id.all & 0x0fffffff;  //29?ID?????28??
        TxMessage.IDE   = CAN_ID_EXT;
    }
    else
    {
			  Frame_Send->Id.all =Frame_Send->Id.StdID;
        TxMessage.StdId = Frame_Send->Id.all & 0x000007ff;
        TxMessage.IDE   = CAN_ID_STD;
    }

    if (Frame_Send->Length > 8) 
        Frame_Send->Length = 8;

    if (Frame_Send->isRemote) 
    {
        TxMessage.RTR = CAN_RTR_REMOTE;
        TxMessage.DLC = 0;
    } 
    else 
    {
        TxMessage.RTR = CAN_RTR_DATA;
        TxMessage.DLC = Frame_Send->Length;
    }
		TxMessage.TransmitGlobalTime=DISABLE;
//		while(Transmit_Mailbox == 0)
//    {
//			switch(canx)
//			{
//				case MYCAN1:
//					Transmit_Mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
//				break;
//				case MYCAN2:
//					Transmit_Mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
//				break;
//			}
//    }
    while ((Transmit_Staus != HAL_OK)&&(Time_Out_Count++ != 0xFF)) 
    {
			switch(canx)
			{
				case MYCAN1:
					Transmit_Staus=HAL_CAN_AddTxMessage(&hcan1, &TxMessage, Data_Send , &Transmit_Mailbox);
				break;
				case MYCAN2:
					Transmit_Staus=HAL_CAN_AddTxMessage(&hcan2, &TxMessage, Data_Send , &Transmit_Mailbox);
				break;
			}
    }
}

void Get_Frame_CAN(CANFrame* Frame_Get, int Canx) 
{
    CAN_RxHeaderTypeDef   RxMessage;
    uint8_t               RxData[8] = {0};
    if (Canx == MYCAN1) 
		{
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage, RxData);
		}
		if (Canx == MYCAN2)
		{
				HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &RxMessage, RxData);
		}
    if (RxMessage.IDE == CAN_ID_EXT) 
        Frame_Get->Id.all     = RxMessage.ExtId;
    else
        Frame_Get->Id.StdID   = RxMessage.StdId;

    Frame_Get->IDE      = RxMessage.IDE;
    Frame_Get->isRemote = RxMessage.RTR == CAN_RTR_DATA ? 0 : 1;
    Frame_Get->Length   = RxMessage.DLC;
    memset(Frame_Get->Data.chars, 0, 8); 
    memcpy(Frame_Get->Data.chars, (char*)RxData, Frame_Get->Length);
}

void (*Find_Method_CAN(uint8_t Prop, CANFunDict* Props_Array, 
    uint8_t Props_Count))(CANFrame* Frm) 
{
    uint8_t i;
    
    for (i = 0; i < Props_Count; ++i) 
    {
        if (Props_Array[i].Prop == Prop) 
            return Props_Array[i].Fun;
    }

    return 0;
}

void Process_Frame_CAN(CANFrame* Frame_Process, int CANx) 
{
    void (*Fun)(CANFrame * Frm);
    CANFunDict* Props_Array;
    uint8_t Props_Count;
    int Protocol;
    EXT_ID_Typedef Frame_ID = Frame_Process->Id.ExtID;

    if (CANx == MYCAN1) 
    {
        Props_Array = g_CAN1_Prop_Array;
        Props_Count = g_CAN1_Prop_Count;
        Protocol    = CAN1_Protocol;
    } 
    else if (CANx == MYCAN2) 
    {
        Props_Array = g_CAN2_Prop_Array;
        Props_Count = g_CAN2_Prop_Count;
        Protocol    = CAN2_Protocol;
    } 
    else 
        return;
    
    if (Frame_Process->IDE == CAN_ID_EXT) 
    {
        if(Protocol == 1)
            Frame_ID.Property = EXTID_NON_PROTOCOL;
    }
    else
    {
        Frame_ID.Property = STDID_NON_PROTOCOL;
    }

    if ((Fun = Find_Method_CAN(Frame_ID.Property, Props_Array, Props_Count)) != 0) 
        Fun(Frame_Process);
		//并没有很理解这段
//    if (Frame_Process->isRemote) 
//    {
//        Frame_ID.DesDeviceId = Frame_ID.SrcDeviceId;
//        if (CANx == Can1) 
//            Frame_ID.SrcDeviceId = DEVICE_ID;
//        else if (CANx == Can2) 
//            Frame_ID.SrcDeviceId = DEVICE_SECOND_ID;
//        Frame_Process->isRemote = 0;
//        Send_Frame_CAN(Frame_Process,CANx);        //远程帧，表明是请求数据，发送对象未知，需咨询前人
//    }

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  /* Get RX message */
  if(CanHandle==&hcan1)
	{
		Get_Frame_CAN(&g_CAN1_Frame_rcv,MYCAN1);
		Process_Frame_CAN(&g_CAN1_Frame_rcv,MYCAN1);
	}
	if(CanHandle==&hcan2)
	{
		Get_Frame_CAN(&g_CAN2_Frame_rcv,MYCAN2);
		Process_Frame_CAN(&g_CAN2_Frame_rcv,MYCAN2);
	}
 
}

