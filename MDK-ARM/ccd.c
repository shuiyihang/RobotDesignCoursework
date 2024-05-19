#include "hardware.h"

#define SET_HIGH(pin)  HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_SET);
#define SET_LOW(pin)   HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_RESET);

static uint8_t  ccd_data[128];
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

int find_ccd_center()
{
    int left,right;
    int val_max,val_min;
    uint8_t threshold;
    
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

    threshold = (val_min + val_max)/2;

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
    return (left + right)/2;
}