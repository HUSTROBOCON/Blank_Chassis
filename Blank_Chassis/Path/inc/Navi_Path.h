#ifndef DEBUG_PATH_H
#define DEBUG_PATH_H

#include "Path.h"
#define NUM_UPHILL_1 141
#define NUM_AREA_TWO 220
#define NUM_UPHILL_2 68
#define NUM_AREA_THREE_BLUE 226
#define NUM_AREA_THREE_RED 220
#define NUM_collide_ball1 152
#define NUM_collide_ball2_front 179
#define NUM_collide_ball2_middle 188
#define NUM_TEST 411
#define NUM_RESET 202
#define NUM_RESET2 143

extern const NavigationPoints uphill_1_blue[NUM_UPHILL_1];//1区上坡
extern const NavigationPoints uphill_1_red[NUM_UPHILL_1];//1区上坡

extern const NavigationPoints area_two_blue[NUM_AREA_TWO];//2区
extern const NavigationPoints area_two_red[NUM_AREA_TWO];

extern const NavigationPoints uphill_2_blue[NUM_UPHILL_2];
extern const NavigationPoints uphill_2_red[NUM_UPHILL_2];

//三区路径
extern const NavigationPoints area_three_blue[NUM_AREA_THREE_BLUE];
extern const NavigationPoints area_three_red[NUM_AREA_THREE_RED];

//test 直线
extern const NavigationPoints test_blue[NUM_TEST];
extern const NavigationPoints test_red[NUM_TEST];

//撞球
extern const NavigationPoints collide_ball1_blue[NUM_collide_ball1];
extern const NavigationPoints collide_ball1_red[NUM_collide_ball1];

extern const NavigationPoints collide_ball2_front_blue[NUM_collide_ball2_front];
extern const NavigationPoints collide_ball2_middle_blue[NUM_collide_ball2_middle];
extern const NavigationPoints collide_ball2_front_red[NUM_collide_ball2_front];
extern const NavigationPoints collide_ball2_middle_red[NUM_collide_ball2_middle];

//重试上三区路径
extern const NavigationPoints reset_blue[NUM_RESET];
extern const NavigationPoints reset_red[NUM_RESET];
extern const NavigationPoints reset2_blue[NUM_RESET2];
extern const NavigationPoints reset2_red[NUM_RESET2];
#endif

