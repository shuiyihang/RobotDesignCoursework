#include "control.h"

int ec_left,ec_right;

int targ_vel_x = 25;

PIDController MOTOR_A = {
	.Kp = 12,
	.Ki = 12,
	.prevError = 0,
	.out = 0
};
PIDController MOTOR_B = {
	.Kp = 12,
	.Ki = 12,
	.prevError = 0,
	.out = 0
};

void Incre_PI_Controller(PIDController* pid,int target,int encoder)
{
    int error = target - encoder;

    pid->out += pid->Kp * (error - pid->prevError) + pid->Ki * error;

    pid->prevError = error;
}

void limit_pwm_output(PIDController* pid)
{
    int upper = 6900;// max is 7200;

    int pwm_val = pid->out;

    if(pwm_val < -upper) pwm_val = -upper;
    if(pwm_val > upper) pwm_val = upper;
    
    pid->out = pwm_val;
}

float calc_bias(int cent_pos)
{
    float ret_val;
    static float last_bias,bias;
    bias = cent_pos - 64;

    ret_val = 0.1 * bias + (bias - last_bias)*1; // pd
    last_bias = bias;

    return ret_val;
}

void Kinematic_analysis(float vel_x,float vel_z,float* left_M,float* right_M)
{
    *left_M = vel_x - vel_z * WIDTH_OF_CAR / 2.0f;  // left target speed
    *right_M = vel_x + vel_z * WIDTH_OF_CAR / 2.0f;
}

void PIDController_Init()
{
    MOTOR_A.Kp = 12;
    MOTOR_A.Ki = 12;
    MOTOR_A.Kd = 0;
    MOTOR_A.prevError = 0;
    MOTOR_A.out = 0;


    MOTOR_B.Kp = 12;
    MOTOR_B.Ki = 12;
    MOTOR_B.Kd = 0;
    MOTOR_B.prevError = 0;
    MOTOR_B.out = 0;
}

// 5ms run_loop
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim3)
    {
        // read speed
        ec_left = read_encoder(2);// ? -
        ec_right = read_encoder(4);
        // read&deal ccd data
        read_ccd_data();
        int center = find_ccd_center();
        // calc bias
        float vel_z = calc_bias(center);
        // Kinematic analysis
        float l_speed,r_speed;
        Kinematic_analysis(targ_vel_x,vel_z,&l_speed,&r_speed);
        // pid control
        {
            Incre_PI_Controller(&MOTOR_A,l_speed,ec_left);
            Incre_PI_Controller(&MOTOR_B,r_speed,ec_right);

            limit_pwm_output(&MOTOR_A);
            limit_pwm_output(&MOTOR_B);

            __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,MOTOR_A.out);
            __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,MOTOR_B.out);
        }
    }
}