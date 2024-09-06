#include "Path.h"
#include "navi_Path.h"

/**
 * @brief  将所有路径点索引到Self中
 * @note
 * @param  Self为PathInfoTypedef类型的路径信息全局变量
 * @retval None
 */
void Path_Init(PathInfoTypedef *Self)
{
    for (int Func_Num = 0; Func_Num < FUNCTION; Func_Num++)
        for (int Path_Num = 0; Path_Num < PATH_MAX; Path_Num++)
        {
            Self->PathDotsArray[Func_Num][Path_Num] = NULL;
            Self->PathDotsNum[Func_Num][Path_Num] = 0;
        }

    Self->PathDotsArray[0][1] = uphill_1_blue;
		Self->PathDotsNum[0][1] = NUM_UPHILL_1;
											
    Self->PathDotsArray[0][2] = area_two_blue;
    Self->PathDotsNum[0][2] = NUM_AREA_TWO;
				
    Self->PathDotsArray[0][3] = uphill_2_blue;
    Self->PathDotsNum[0][3] = NUM_UPHILL_2;
		
		Self->PathDotsArray[0][4] = area_three_blue;
    Self->PathDotsNum[0][4] = NUM_AREA_THREE_BLUE;
		
	  Self->PathDotsArray[0][5] = collide_ball1_blue;
    Self->PathDotsNum[0][5] = NUM_collide_ball1;	

//    Self->PathDotsArray[0][6] = collide_ball2_blue;
//    Self->PathDotsNum[0][6] = NUM_collide_ball2;
		
    Self->PathDotsArray[0][7] = test_blue;
    Self->PathDotsNum[0][7] = NUM_TEST;	
		
    Self->PathDotsArray[0][8] = reset_blue;
    Self->PathDotsNum[0][8] = NUM_RESET;
		
    Self->PathDotsArray[0][9] = reset2_blue;
    Self->PathDotsNum[0][9] = NUM_RESET2;		
		//第10条路径是在线规划生成的路径
		Self->PathDotsArray[0][11] = collide_ball2_front_blue;
    Self->PathDotsNum[0][11] = NUM_collide_ball2_front;
		
		Self->PathDotsArray[0][12] = collide_ball2_middle_blue;
    Self->PathDotsNum[0][12] = NUM_collide_ball2_middle;
		
		Self->PathDotsArray[1][1] = uphill_1_red;
		Self->PathDotsNum[1][1] = NUM_UPHILL_1;
											
    Self->PathDotsArray[1][2] = area_two_red;
    Self->PathDotsNum[1][2] = NUM_AREA_TWO;
				
    Self->PathDotsArray[1][3] = uphill_2_red;
    Self->PathDotsNum[1][3] = NUM_UPHILL_2;
		
		Self->PathDotsArray[1][4] = area_three_red;
    Self->PathDotsNum[1][4] = NUM_AREA_THREE_RED;
		
	  Self->PathDotsArray[1][5] = collide_ball1_red;
    Self->PathDotsNum[1][5] = NUM_collide_ball1;	

		
    Self->PathDotsArray[1][7] = test_red;
    Self->PathDotsNum[1][7] = NUM_TEST;	
		
    Self->PathDotsArray[1][8] = reset_red;
    Self->PathDotsNum[1][8] = NUM_RESET;
		
		Self->PathDotsArray[1][9] = reset2_red;
    Self->PathDotsNum[1][9] = NUM_RESET2;
		
		Self->PathDotsArray[1][11] = collide_ball2_front_red;
    Self->PathDotsNum[1][11] = NUM_collide_ball2_front;
		
		Self->PathDotsArray[1][12] = collide_ball2_middle_red;
    Self->PathDotsNum[1][12] = NUM_collide_ball2_middle;
}
