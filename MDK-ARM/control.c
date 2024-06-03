#include "control.h"

int ec_left,ec_right;

int targ_vel_x = 5;

int test_tar_speed = 0;

int g_abs_bias;

PIDController MOTOR_A = {
	.Kp = 15,
	.Ki = 12,
	.prevError = 0,
	.out = 0
};
PIDController MOTOR_B = {
	.Kp = 15,
	.Ki = 12,
	.prevError = 0,
	.out = 0
};


#define SET_HIGH(pin)  HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_SET);
#define SET_LOW(pin)   HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_RESET);


void Incre_PI_Controller(PIDController* pid,int target,int encoder)
{
    int error = target - encoder;

    pid->out += pid->Kp * (error - pid->prevError) + pid->Ki * error;

    pid->prevError = error;
}

void limit_pwm_output(PIDController* pid)
{
    int upper = MAX_PWM_OUT	;// max is 7200;

    int pwm_val = pid->out;

    if(pwm_val < -upper) pwm_val = -upper;
    if(pwm_val > upper) pwm_val = upper;
    
    pid->out = pwm_val;
}

float calc_bias(int cent_pos)
{
    float ret_val;
    static int last_cent_pos = 0,bias;

    bias = 64 - cent_pos;

    g_abs_bias = abs(bias);

    ret_val = 0.1f * bias + (last_cent_pos - cent_pos)*1.0f; // pd
    last_cent_pos = cent_pos;

    return ret_val;
}

void Kinematic_analysis(float vel_x,float vel_z,float* left_M,float* right_M)
{
    float factor = 1.0f;
    if(g_abs_bias < 10)
    {
        // little bias
        factor = 1.0f;
    }else if(g_abs_bias < 20)
    {
        factor = 1.6f;
    }else
    {
        factor = 2.6f;
    }
    // maybe instead of const val,dir rely to vel_z
    vel_z = vel_z * factor;
    *left_M = vel_x - vel_z ;//* WIDTH_OF_CAR / 2.0f;  // left target speed
    *right_M = vel_x + vel_z;//* WIDTH_OF_CAR / 2.0f;
}

void set_motor_output(PIDController* Motor_A,PIDController* Motor_B)
{
    if(Motor_A->out > 0)
    {
			// left front
        SET_HIGH(AIN1);
        SET_LOW(AIN2);
    }else
    {
//			Motor_A->out = 0;
        SET_HIGH(AIN2);
        SET_LOW(AIN1);
    }

    if(Motor_B->out > 0)
    {
			// right front
			 SET_HIGH(BIN2);
        SET_LOW(BIN1);
    }else
    {
//			Motor_B->out = 0;
        SET_HIGH(BIN1);
        SET_LOW(BIN2);
    }

    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1, abs(Motor_A->out));
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4, abs(Motor_B->out));
}

// 5ms run_loop
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim3)
    {
#ifndef TSET_MODE
        // read speed
        ec_left = read_encoder(2);
        ec_right = read_encoder(4);
        // read&deal ccd data
        read_ccd_data();
        int center = find_ccd_center_1();
        // calc bias
        float vel_z = calc_bias(center);
        // Kinematic analysis
        float l_speed,r_speed;
        Kinematic_analysis(targ_vel_x,vel_z,&l_speed,&r_speed);
        // pid control
        {
//						Incre_PI_Controller(&MOTOR_A,test_tar_speed,ec_left);
//						Incre_PI_Controller(&MOTOR_B,test_tar_speed,ec_right);
            Incre_PI_Controller(&MOTOR_A,l_speed,ec_left);
            Incre_PI_Controller(&MOTOR_B,r_speed,ec_right);

            limit_pwm_output(&MOTOR_A);
            limit_pwm_output(&MOTOR_B);
						// printf("center:%d,left:%d,right:%d\n",center,MOTOR_A.out,MOTOR_B.out);
            set_motor_output(&MOTOR_A,&MOTOR_B);
						
        }
#endif
    }
}