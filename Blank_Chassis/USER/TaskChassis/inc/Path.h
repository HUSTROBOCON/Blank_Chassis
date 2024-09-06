#ifndef PATH_H_
#define PATH_H_

/************************* Includes *************************/

#include "main.h"

/************************* Defines *************************/

#define FUNCTION 10  //功能数量
#define PATH_MAX 15 //任意功能下的最大路径数量

/************************* Exported types *************************/

typedef struct
{
    float X;
    float Y;
    float Theta;
    float Alpha;
    float Dis;
    float V;
    uint8_t Preview;
} NavigationPoints;

typedef struct
{
    const NavigationPoints *PathDotsArray[FUNCTION][PATH_MAX];
    uint16_t PathDotsNum[FUNCTION][PATH_MAX];
} PathInfoTypedef;

/************************* Exported constants *************************/

/************************* Exported Functions *************************/

void Path_Init(PathInfoTypedef *Self);

#endif
