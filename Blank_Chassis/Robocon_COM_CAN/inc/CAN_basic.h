#ifndef CAN_BASIC_H_
#define CAN_BASIC_H_

#include "stm32f4xx_hal.h"
#include "CAN_IDconf.h"

#define MYCAN1                      1
#define MYCAN2                      2

#define CAN1_Protocol               0               //0代表采用团队协议，1代表不采用团队协议
#define CAN2_Protocol               0               //0代表采用团队协议，1代表不采用团队协议

typedef struct
{
	uint32_t DesDeviceId   	:8;     //????ID
	uint32_t Property		    :8;		  //?????
	uint32_t SrcDeviceId   	:8;     //????ID	
	uint32_t Priority				:4;			//???
	uint32_t Permit         :1;         //??
}EXT_ID_Typedef;								  //29???ID

typedef union
{
		uint32_t	 all;
		uint32_t	 StdID		:11;		//??ID
		EXT_ID_Typedef	ExtID ;		//??ID
}ID;
typedef union {
    int8_t    chars[8];                             // 8?char
    int16_t   shorts[4];                            // 4?short
    int32_t   ints[2];                              // 2?int
    int64_t   longs[1];                             // 1?Long
    uint8_t   uchars[8];                            // 8????char
    uint16_t  ushorts[4];                           // 4????short
    uint32_t  uints[2];                             // 2????int
    uint64_t  ulongs[1];                            // 1????Long
    float     floats[2];
} CAN_Data;
typedef struct {
    ID Id;                                          // ID
    uint8_t IDE;                                    // IDE
    char isRemote;                                  // ??????
    char Length;                                    // ????
    CAN_Data Data;                                   // ??
} CANFrame;

typedef struct {
  uint16_t Prop;                                    //属性名称
  void (*Fun)(CANFrame* Frm);                       //此属性对应的处理函数
} CANFunDict;

extern CANFrame g_CAN1_Frame_tsm;
extern CANFrame g_CAN1_Frame_rcv;
extern CANFrame g_CAN2_Frame_tsm;
extern CANFrame g_CAN2_Frame_rcv;

void Can_Init(int Canx);
void Send_Frame_CAN(CANFrame* Frame_Send,int canx) ;
#endif

