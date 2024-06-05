#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#include "hardware.h"

typedef enum{
    RUN,
    STOP
}car_state;

static int car_speed = 0;
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



extern int targ_vel_x;

extern PIDController MOTOR_A,MOTOR_B;

void Incre_PI_Controller(PIDController* pid,int target,int encoder);
void limit_pwm_output(PIDController* pid);
float calc_bias(int cent_pos);
void Kinematic_analysis(float vel_x,float vel_z,float* left_M,float* right_M);
void set_motor_output(int a_out,int b_out);

#define MAX_PWM_OUT	(6900)	// 6900
#define AIN1    GPIO_PIN_12
#define AIN2    GPIO_PIN_13
#define BIN1    GPIO_PIN_14
#define BIN2    GPIO_PIN_15


#define ZONE_NUMS   7
#define PIXELS_TO_REMOVE    5

void car_fuzzy_ctrl();
#endif