#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#include "hardware.h"


typedef struct{
    float Kp;
	float Ki;
	float Kd;

    float integrator;// 累计积分值
    // 微分
    float differentiator;

    int prevError;
    int out;

}PIDController;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim1;


void PIDController_Init();

#endif