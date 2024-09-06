#include "PID.h"
 /**
 * @brief PID调节函数
 * @param PID输出，float
 * @retval float 设定值，测量值，PID结构
 */
float Pid_Regulate(float Reference, float Present_Feedback, 
    PIDStructTypedef* PID_Struct)
{
	float Error;
	float Error_Inc;
	float pTerm;
	float iTerm;
	float dTerm;
	float dwAux;
	float Output;
	/*Error computation*/
	Error = Reference - Present_Feedback;
	
	/*proportional term computation*/
	pTerm = Error * PID_Struct->Kp;

	/*Integral term computation*/
	if (PID_Struct->Ki == 0)
		PID_Struct->Integral = 0;           
	else
	{
		iTerm = Error * PID_Struct->Ki;
		dwAux = PID_Struct->Integral + iTerm;

		/*limit integral*/
		if (dwAux > PID_Struct->LimitIntegral)
			PID_Struct->Integral = PID_Struct->LimitIntegral;
		else if (dwAux < -1 * PID_Struct->LimitIntegral)
			PID_Struct->Integral = -1 * PID_Struct->LimitIntegral;
		else
			PID_Struct->Integral = dwAux;
	}

	/*differential term computation*/
	Error_Inc            = Error - PID_Struct->PreError;
	dTerm                = Error_Inc * PID_Struct->Kd;
	PID_Struct->PreError = Error;
	Output               = pTerm + PID_Struct->Integral + dTerm;

	/*limit Output*/
	if (Output >= PID_Struct->LimitOutput)
		return (PID_Struct->LimitOutput);
	else if (Output < -1 * PID_Struct->LimitOutput)
		return (-1 * PID_Struct->LimitOutput);
	else
		return Output;
}
