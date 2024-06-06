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

static uint8_t sensor_data,last_sensor_data;
static car_state car_status = RUN;

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

void set_motor_output(int a_out,int b_out)
{
    if(a_out > 0)
    {
			// left front
        SET_HIGH(AIN1);
        SET_LOW(AIN2);
    }else
    {
        SET_HIGH(AIN2);
        SET_LOW(AIN1);
    }

    if(b_out > 0)
    {
			// right front
        SET_HIGH(BIN2);
        SET_LOW(BIN1);
    }else
    {
        SET_HIGH(BIN1);
        SET_LOW(BIN2);
    }

    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1, abs(a_out));
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4, abs(b_out));
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
            set_motor_output(MOTOR_A.out,MOTOR_B.out);
						
        }
#endif
    }
}



/**
 * step1: 计算阈值
 * step1: 128像素划分为7个块，通过投票，获取7个块的值(0/1)
 * step2: 模糊控制，电机转速
*/
static void split_ccd_data(uint8_t* result)
{
    uint8_t zones[ZONE_NUMS];

    uint8_t data_size = 128 - 2 * PIXELS_TO_REMOVE;
    uint8_t zone_size = data_size/ZONE_NUMS;// 16
    uint8_t add_def = data_size%ZONE_NUMS;

    uint8_t threshold = auto_threshold_1();

    uint8_t start_idx = PIXELS_TO_REMOVE,end_idx;

    uint8_t*  ccd_data = get_data_handle();

    *result = 0;

    for(int i = 0;i < ZONE_NUMS;i++)
    {
        uint8_t black_cnt = 0,white_cnt = 0;
        end_idx = start_idx + zone_size + (i < add_def?1:0);
        for(int j = start_idx;j < end_idx;j++)
        {
            if(ccd_data[j] < threshold)
            {
                black_cnt++;
            }else
            {
                white_cnt++;
            }
        }
        if(black_cnt > white_cnt)
        {
            *result |= (1 << (ZONE_NUMS - i - 1));// 黑色标记
        }
        start_idx = end_idx;
    }

}
void car_fuzzy_ctrl()
{
    int speed_gain;

    // printf("111===========\n");
    read_ccd_data();
    split_ccd_data(&sensor_data);
    printf("sensor_data:%#x\n",sensor_data);
    switch (sensor_data)
    {
    case 0b0000001:
        //需要右转，左轮加速
        speed_gain = 50;
        break;
    case 0b0000010:// TODO:需要测试，不知道黑线能占几个像素
        speed_gain = 40;
        break;
    case 0b0000100:
        speed_gain = 20;
        break;
    // 正中间
    case 0b1000:
        speed_gain = 0;
        break;
    case 0b10000:
        speed_gain = -20;
        break;

    case 0b100000:
        speed_gain = -40;
        break;

    case 0b1000000:
        speed_gain = -50;
        break;
    // 停止线
    case 0b1111111:

        break;
    default:
        break;
    }

    int left_out = car_speed+speed_gain*20;
    if(left_out > MAX_PWM_OUT)left_out = MAX_PWM_OUT;

    int right_out = car_speed-speed_gain*20;
    if(right_out > MAX_PWM_OUT)right_out = MAX_PWM_OUT;

    set_motor_output(left_out,right_out);

    // do other
    switch (car_status)
    {
    case RUN:
        car_speed = 1000;// 设置一个基本值
        break;
    case STOP:

        break;
    default:
        break;
    }
}