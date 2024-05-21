#include "hardware.h"

const float VREF = 3.3f;

float get_battery_vol()
{
    uint16_t adc_value;
    
    HAL_ADC_Start(&hadc2);

    if (HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY) == HAL_OK)
    {
			adc_value = HAL_ADC_GetValue(&hadc2);
    }
    HAL_ADC_Stop(&hadc2);
    
		float voltage = (adc_value/4095.0f)*VREF*11;
		return voltage;
}