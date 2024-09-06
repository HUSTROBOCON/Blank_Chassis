#ifndef ROBOTCOM_PROPLIST_H_
#define ROBOTCOM_PROPLIST_H_

#include "COM_basic.h"

extern COMFunDict g_My_USART1_Prop_Array[];

extern COMFunDict g_My_USART2_Prop_Array[];

extern COMFunDict g_My_USART3_Prop_Array[];

extern COMFunDict g_My_USART4_Prop_Array[];

extern const uint8_t g_My_USART1_Prop_Count;

extern const uint8_t g_My_USART2_Prop_Count;

extern const uint8_t g_My_USART3_Prop_Count;

extern const uint8_t g_My_USART4_Prop_Count;

/* MYUSART1 part */
/******************************prop属性值,8位*********************************/

#define NAV_CHOOSE 0x61
#define CAMERA_CTRL  0x60
#define LARDAR_CALIBRATION 0x63
#define ROTATE_SPEED 0x65
#define STOP_FINDBALL 0x67


//发送函数
#define SEND_CHASSIS_INFO 0x71
#define NAV_FINISH 0x70
#define WAITING_ARRIVE 0x72
#define UPPER_RESET 0x73

/******************************接收函数*********************************/

void Camera_ctrl(COMFrame *);
void Far_Camera_ctrl(COMFrame *);
void Nav_Choose(COMFrame *);
void Lardar_Calibration(COMFrame *);
void Rotate_Speed(COMFrame *);
void Stop_Findball(COMFrame *);
/******************************发送函数*********************************/
void Send_Chassis_Info(void);
void Send_Nav_Finish(int8_t data);
void Send_Waiting(void);
void Send_Upper_Reset(void);

/* MYUSART3 part */
/******************************prop,属性值8位*********************************/
/* 手操功能命令 */
//地址如此之乱是为了兼顾各个手柄版本而导致的遗留问题
#define GET_AIM_VELOCITY 0x50//左手柄
#define GET_AIM_ANGULAR_VELOCITY 0x51//右手柄
#define RUN 0X68
#define ROBOT_RESET 0x60
#define FIRST_BASKET 0x81
#define SECOND_BASKET 0x82
#define THIRD_BASKET 0x83
#define FOURTH_BASKET 0x84
#define FIFTH_BASKET 0x85
#define FIELD_CHOOSE 0x86
#define EMERGENCY_STOP 0x87
#define RETURN_DATA 0x92
#define HELM_TEST 0x93
#define HELM_RESET 0x94
#define GYRO_RESET 0x95
#define COLLIDE_YES 0x96
#define COLLIDE_NO 0x97


/******************************接收函数*********************************/
/* 手操功能函数 */

void Update_Aim_Velocity(COMFrame *);
void Update_Aim_Angular_Velocity(COMFrame *);
void Run(COMFrame *);
void Reset(COMFrame *);
void first_basket(COMFrame *);
void second_basket(COMFrame *);
void third_basket(COMFrame *);
void fourth_basket(COMFrame *);
void fifth_basket(COMFrame *);
void field_choose(COMFrame *);
void emergency_stop(COMFrame *);
void helm_test(COMFrame *);
void helm_reset(COMFrame *);
void return_data(COMFrame *);
void gyro_reset(COMFrame *);
void collide_yes(COMFrame *);
void collide_no(COMFrame *);

/******************************发送函数*********************************/
void Send_Chassis_To_handler(void);
void Upper_Machine(void);


/* MYUSART4 part */
/******************************prop,属性值8位*********************************/
//接收
#define GET_MID360_POS 0x53 //mid360
#define GET_A3_POS 0x52 //A3

//发送

/******************************接收函数*********************************/
void get_mid360_pos(COMFrame *);
void get_a3_pos(COMFrame *);
/******************************发送函数*********************************/

void Run(COMFrame *Frame);
void Reset(COMFrame *Frame);
#endif

