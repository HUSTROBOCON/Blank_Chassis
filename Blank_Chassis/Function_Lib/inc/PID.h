#ifndef PID_H_
#define PID_H_

/***PID参数结构***/
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float LimitOutput;                              // Lower Limit for Output limitation
    float LimitIntegral;                            // Upper Limit for Output limitation
    float Integral;                                 //积分项,存储积分误差×KI
    float PreError;
} PIDStructTypedef;

float Pid_Regulate(float, float, PIDStructTypedef*);

#endif
