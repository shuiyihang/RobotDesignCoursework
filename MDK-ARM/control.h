#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#include "hardware.h"
#include "stdlib.h"

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

#define MAX_PWM_OUT	(5000)	// 6900
#define AIN1    GPIO_PIN_12
#define AIN2    GPIO_PIN_13
#define BIN1    GPIO_PIN_14
#define BIN2    GPIO_PIN_15

#endif