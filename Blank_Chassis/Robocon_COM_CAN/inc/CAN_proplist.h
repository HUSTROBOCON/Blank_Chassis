#ifndef CAN_PROPLIST_
#define CAN_PROPLIST_

#include "can_basic.h"
#include "can.h"
#include "TaskChassis.h"

extern CANFunDict g_CAN1_Prop_Array[];

extern CANFunDict g_CAN2_Prop_Array[];

extern const uint8_t g_CAN1_Prop_Count;

extern const uint8_t g_CAN2_Prop_Count;

/* CAN1 part */

/******************************LOCATE-DRIVER,CAN1*****************************/

/******************************prop属性值,8位*********************************/
#define EXTID_NON_PROTOCOL          0x00
#define STDID_NON_PROTOCOL          0x01
#define GET_TRAIN_INFO				0x02
#define SEND_TRAIN_INFO				0x03
#define STOP_TRAIN					0x04

/******************************prio优先级,4位*********************************/



/* CAN2 part */

/******************************LOCATE-MASTER,CAN2****************************/

/******************************prop,属性值8位*********************************/
#define EXTID_NON_PROTOCOL          0x00
#define STDID_NON_PROTOCOL          0x01

/******************************prio,优先级4位*********************************/

/******************************接收函数*********************************/
void Get_Train_Info(CANFrame*);

/******************************发送函数*********************************/

void Get_Train_Request(void);

void Set_Train_Info(ChassisController*, int);

void Stop_Train(void);

void ExtID_Non_Protocol_CAN1(CANFrame*);

void StdID_Non_Protocol_CAN1(CANFrame*);

void ExtID_Non_Protocol_CAN2(CANFrame*);

void StdID_Non_Protocol_CAN2(CANFrame*);


#endif

