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

void set_motor_output(PIDController* Motor_A,PIDController* Motor_B)
{
    if(Motor_A->out > 0)
    {
			// left front
        SET_HIGH(AIN1);
        SET_LOW(AIN2);
    }else
    {
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
        // read speed
        ec_left = read_encoder(2);// may be inverse
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

            set_motor_output(&MOTOR_A,&MOTOR_B);
        }
    }
}