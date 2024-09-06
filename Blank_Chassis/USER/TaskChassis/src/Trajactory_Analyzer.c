/**
 ******************************************************************************
 * @file    Trajectory_Analyzer.c
 * @brief   目标点确定相关函数定义，包含以下内容：
 *          - 路径分析控制器初始化函数
 *          - 路径设置函数
 *          - 目标点搜索函数
 * @verbatim
 * @endverbatim
 * @edit date
 ******************************************************************************
 */

#include "Trajectory_Analyzer.h"
#include "HUST_Math_Lib.h"
#include "TaskChassis.h"
#include <arm_math.h>
/**
 * @brief  路径分析控制器初始化
 * @note
 * @param
 * @retval
 */
TrajectoryAnalyzer g_Trajectory_Analyzer_Instance;
void Trajectory_Analyzer_Init(TrajectoryAnalyzer *Self)
{
    Self->CurrentPointIndex = -1;
    Self->AimPointIndex = 0;
    Self->NavigationPathType = -1;
    Self->NavigationPathNum = -1;
    Self->CurrentPath = NULL;
    Self->MaxPointIndex = 0;
    Self->Dis = 0.0f;
    Self->IsNavigationFinish = false;
    Self->IsFirstNavigation = true;
}

void Trajectory_Analyzer_Reset(TrajectoryAnalyzer *Self)
{
    Self->CurrentPointIndex = -1;
    Self->AimPointIndex = 0;
    Self->NavigationPathType = -1;
    Self->NavigationPathNum = -1;
    Self->CurrentPath = NULL;
    Self->MaxPointIndex = 0;
    Self->Dis = 0.0f;
    Self->IsNavigationFinish = false;
    Self->IsFirstNavigation = true;
}
/**
 * @brief  设置路径
 * @note   每次切换到Navigate状态，需要重新Reset和SetPathNum，以防止出现标志位错误
 * @param
 * @retval
 */
void Trajectory_Analyzer_SetPathNum(TrajectoryAnalyzer *Self,
                                    PathInfoTypedef *Path_Info,
                                    int16_t Path_Type, int16_t Path_Num)
{
    Self->CurrentPointIndex = 0;
    Self->AimPointIndex = 1;
    Self->NavigationPathType = Path_Type;
    Self->NavigationPathNum = Path_Num;
    Self->Dis = 0.0f;
    Self->CurrentPath = (NavigationPoints *)Path_Info->PathDotsArray
                            [Path_Type][Path_Num];
    Self->MaxPointIndex = Path_Info->PathDotsNum[Path_Type][Path_Num] - 1;
    Self->IsFirstNavigation = true;
}

/**
 * @brief  确定路径上的下一个目标点
 * @note   搜索最近点作为当前点，并将最近点的下一个点作为目标点
 * @param
 * @retval
 */
void Trajectory_Analyzer_GetAimPointIndex(TrajectoryAnalyzer *Self)
{
		ChassisController *Chassis_Controller=&g_Chassis_Instance;
    uint8_t Break_Count = 0;
    uint16_t Path_Index = Self->CurrentPointIndex;
    uint16_t Ref_Index_Raw = Path_Index;
    float Min_Dis = 10000000;//取一个相对大的数
    float Dis = 0.0f;
    float Delta_X = 0.0f;
    float Delta_Y = 0.0f;
    int16_t i = Path_Index >= 3 ? (Path_Index - 2) : Path_Index;
    //这里在23时会出现突变现象后续需处理，并且会从0递进到3

    while (++i <= Self->MaxPointIndex && Break_Count <= 2)
    {
        Delta_X = Chassis_Controller->CurrentState.X - Self->CurrentPath[i].X;
        Delta_Y = Chassis_Controller->CurrentState.Y - Self->CurrentPath[i].Y;
        Dis = powf(Delta_X, 2) + powf(Delta_Y, 2);

        if (Dis <= Min_Dis)
        {
            Path_Index = i;
            Min_Dis = Dis;
        }
        else
            ++Break_Count;
    }

    Self->CurrentPointIndex = Path_Index;
    Self->AimPointIndex = Path_Index;
    Self->Dis = Min_Dis;
    Ref_Index_Raw = Path_Index + Self->CurrentPath[Path_Index].Preview;

    if (Self->AimPointIndex >= Self->MaxPointIndex)
        Self->AimPointIndex = Self->MaxPointIndex;
    if (Ref_Index_Raw >= Self->MaxPointIndex)
        Ref_Index_Raw = Self->MaxPointIndex;

    Chassis_Controller->AimState.X =
        Self->CurrentPath[Self->AimPointIndex].X;
    Chassis_Controller->AimState.Y =
        Self->CurrentPath[Self->AimPointIndex].Y;
    Chassis_Controller->AimState.Theta =
        NormalizeAngle(Self->CurrentPath[Self->AimPointIndex].Theta) * RAD2DEG;
         NormalizeAngle(Self->CurrentPath[Ref_Index_Raw].Alpha) * RAD2DEG;
    Chassis_Controller->AimState.Vx =
        Self->CurrentPath[Self->AimPointIndex].V *
        arm_cos_f32(Self->CurrentPath[Self->AimPointIndex].Theta);
    Chassis_Controller->AimState.Vy =
        Self->CurrentPath[Self->AimPointIndex].V *
        OptimizeArm_sin_f32(Self->CurrentPath[Self->AimPointIndex].Theta);
     
		Chassis_Controller->AimState.V=Self->CurrentPath[Self->AimPointIndex].V;
		
    if (Self->AimPointIndex >= Self->MaxPointIndex) 
    {
        Chassis_Controller->AimState.X =
            Self->CurrentPath[Self->MaxPointIndex].X;
        Chassis_Controller->AimState.Y =
            Self->CurrentPath[Self->MaxPointIndex].Y;
        Chassis_Controller->AimState.Theta =
            NormalizeAngle(Self->CurrentPath[Self->MaxPointIndex].Theta) * RAD2DEG;
					Chassis_Controller->AimState.Alpha =
						 NormalizeAngle(Self->CurrentPath[Ref_Index_Raw].Alpha) * RAD2DEG;
        Chassis_Controller->AimState.Vx =
            Self->CurrentPath[Self->MaxPointIndex].V *
            arm_cos_f32(Self->CurrentPath[Self->MaxPointIndex].Theta);
        Chassis_Controller->AimState.Vy =
            Self->CurrentPath[Self->MaxPointIndex].V *
            OptimizeArm_sin_f32(Self->CurrentPath[Self->MaxPointIndex].Theta);
			  Chassis_Controller->AimState.V=Self->CurrentPath[Self->MaxPointIndex].V;
        Self->IsNavigationFinish = true;
    }
}
