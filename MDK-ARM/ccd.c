#include "hardware.h"

#define SET_HIGH(pin)  HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_SET);
#define SET_LOW(pin)   HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_RESET);

static uint8_t  ccd_data[128];


static uint8_t  to_pc_data[200];

static void delay_us(uint16_t us)
{
    uint32_t count = us * (SystemCoreClock / 1000000 / 5);
    while (count--) {
        __NOP();
    }
}

static uint8_t _get_adc()
{
    uint16_t adc_value;
    
    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
    {
			adc_value = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    return adc_value>>4;
}

void show_ccd_data()
{
	for(int i = 0;i < 128;i++)
	{
		printf("%d ",ccd_data[i]);
	}
	printf("\n");
}
/*
 *  CCD:	clk			PA5
			SI			PA6
			ADC(AO)	    PA4

low  :  GPIO_PIN_RESET
high :  GPIO_PIN_SET
*/
void read_ccd_data()
{
    uint8_t tslp=0;
    SET_LOW(TSL_CLK);
    SET_LOW(TSL_SI);
    delay_us(5);
    
    SET_HIGH(TSL_SI);
    delay_us(5);
    
    SET_LOW(TSL_CLK);
    delay_us(5);
        
    SET_HIGH(TSL_CLK);
    delay_us(5);
        
    SET_LOW(TSL_SI);
    delay_us(5);

    for(uint8_t i = 0;i < 128;i++)
    { 
        SET_LOW(TSL_CLK);
        delay_us(5);
        ccd_data[tslp++] = _get_adc();
        SET_HIGH(TSL_CLK);
        delay_us(5);
    } 
}

static uint8_t auto_threshold_1(void)
{
    int val_max,val_min;
    // find min
    val_min = ccd_data[0];
    for(int i = 5;i < 123;i++)
    {
        if(val_min > ccd_data[i])val_min = ccd_data[i];
    }
    // find max
    val_max = ccd_data[0];
    for(int i = 5;i < 123;i++)
    {
        if(val_max < ccd_data[i])val_max = ccd_data[i];
    }

    return (val_min + val_max)/2;
}

// 大津法   otsu
static uint8_t otsu_threshold()
{
    #define GrayScale 256
    int Pixel_Max=0;
    int Pixel_Min=255;

    uint8_t pixelCount[GrayScale];  //各像素GrayScale的个数pixelCount 一维数组
    float pixelPro[GrayScale];  //各像素GrayScale所占百分比pixelPro 一维数组
    int i, j, pixelSum = 128;
    uint8_t threshold = 0;


    uint8_t* data = ccd_data;  //指向像素数据的指针
 
    //清零
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
 
    uint32_t gray_sum=0;  //每次执行到这会将gray_sum清零
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < 128; i+=1)
    {
        pixelCount[data[i]]++;  //将当前的点的像素值作为计数数组的下标
        gray_sum += data[i];       //灰度值总和
        if(data[i] > Pixel_Max) Pixel_Max = data[i];
        if(data[i] < Pixel_Min) Pixel_Min = data[i];
    }
 
    //计算每个像素值的点在整幅图像中的比例
    for (i = Pixel_Min; i < Pixel_Max; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
 
    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
 
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = Pixel_Min; j < Pixel_Max; j++)
    {
        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值
 
        w1 = 1-w0;
        u1tmp = gray_sum / pixelSum - u0tmp;
 
        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = (float)(w0 *w1* (u0 - u1)* (u0 - u1)) ;
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
 
    }
    return threshold;
}


int find_ccd_center_1()
{
		int ret;
    int left,right;
    
    uint8_t threshold;

    // threshold = auto_threshold_1();
    threshold = otsu_threshold();
    // find left edge
    for(int i = 5;i < 118; i++)
	{
		if(ccd_data[i]>threshold&&ccd_data[i+1]>threshold&&ccd_data[i+2]>threshold&&ccd_data[i+3]<threshold&&ccd_data[i+4]<threshold&&ccd_data[i+5]<threshold)
		{	
			left = i;
			break;	
		}
	}
    // find right edge
    for(int i = 118;i > 5; i--)
    {
        if(ccd_data[i]<threshold&&ccd_data[i+1]<threshold&&ccd_data[i+2]<threshold&&ccd_data[i+3]>threshold&&ccd_data[i+4]>threshold&&ccd_data[i+5]>threshold)
        {	
            right = i;
            break;	
        }
    }
		
	ret = (left + right)/2;
    printf("center:%d,left:%d,right:%d,threahold:%d\n",ret,left,right,threshold);

    return ret;
}

/**
 * 另一种寻找中线的方式
*/

static uint8_t auto_threshold_3()
{
    uint8_t threshold;
    int sum = 0,diff_avg = 0,ad_avg = 0;
	for(int i = 6;i < 121 - 3;i++)
	{
		sum += abs(*(ccd_data+i) - *(ccd_data+i+3)); //保存数据 绝对值
	}
	diff_avg = sum / 112; //平均差分
	
    int ad_sum = 0;
    for(int j = 6;j <= 118;j++)
    {
        ad_sum += ccd_data[j];
    }
    ad_avg = ad_sum / 112;

	threshold = ad_avg / 6 + diff_avg; //阈值

    return threshold;
}
int find_ccd_center_2()
{
    int center = -1;// 异常值

    uint8_t mid_flag1 = 0,mid_flag2 = 0;
    uint8_t mid_left1 = 0,mid_left2 = 0;
    uint8_t mid_right1 = 0,mid_right2 = 0;

    uint8_t threshold = auto_threshold_3();

    uint8_t line_width;// 黑线宽度

    uint8_t line_mid1,line_mid2;// 线的位置

    uint8_t LxQ1 = 0,RxQ1 = 0;
    uint8_t LxQ2 = 0,RxQ2 = 0;

    // 从左往右找黑线
    for(int i = 6; i < 121; i++)
    {
        if(ccd_data[i] - ccd_data[i+2] >= threshold)
        {
            mid_flag1++;
            if(mid_flag1 >= 2)
            {
                // 找到黑线左边沿
                mid_left1 = i;
                LxQ1 = 1;

                for(int j = mid_left1; j < 120; j++)
                {
                    if(ccd_data[j+2] - ccd_data[j] >= threshold)
                    {
                        mid_flag2++;
                        if(mid_flag2 >= 2)
                        {
                            // 找到黑线右边沿
                            mid_right1 = j;
                            line_width = mid_right1 - mid_left1;
                            if(line_width >= 1 && line_width <= 8)
                            {
                                line_mid1 = (mid_right1 + mid_left1) >> 1;
                                RxQ1 = 1;
                            }
                            break;
                        }
                    }else
                    {
                        mid_flag2 = 0;
                    }
                }
                break;
            }
            
        }else
        {
            mid_flag1 = 0;
        }
    }

    mid_flag1 = 0;
    mid_flag2 = 0;

    // 从右边往左边扫描
    for(int i = 121; i > 6; i--)
    {
        if(ccd_data[i] - ccd_data[i-2] >= threshold)
        {
            mid_flag1++;
            if(mid_flag1 >= 2)
            {
                // 找到黑线左边沿
                mid_right2 = i;
                RxQ2 = 1;

                for(int j = mid_right2; j > 6; j--)
                {
                    if(ccd_data[j-2] - ccd_data[j] >= threshold)
                    {
                        mid_flag2++;
                        if(mid_flag2 >= 2)
                        {
                            // 找到黑线右边沿
                            mid_left2 = j;
                            line_width = mid_right2 - mid_left2;
                            if(line_width >= 1 && line_width <= 8)
                            {
                                line_mid2 = (mid_right2 + mid_left2) >> 1;
                                LxQ2 = 1;
                            }
                            break;
                        }
                    }else
                    {
                        mid_flag2 = 0;
                    }
                }
                break;
            }
            
        }else
        {
            mid_flag1 = 0;
        }
    }

    if(LxQ1 != 0 && RxQ1 != 0 && LxQ2 != 0 && RxQ2 != 0 && abs(line_mid1 - line_mid2) <= 5)
    {
        center = line_mid1;
    }
    return center;
}




void commission_with_pc()
{
#ifdef TSET_MODE
    read_ccd_data();
    memset(to_pc_data,0,sizeof(to_pc_data));
    memcpy(to_pc_data+4,ccd_data,128);

    printf("*");
    printf("LD");
    for(int i = 0;i < 132;i++)
    {
        printf("%c%c",(to_pc_data[i]>>4)&0x0F + '0',to_pc_data[i]&0x0F + '0');
    }
    printf("00");
    printf("#");
#endif
}