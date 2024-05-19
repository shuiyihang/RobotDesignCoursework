#ifndef __HARDWARE_H
#define __HARDWARE_H
#include "sys.h"


extern ADC_HandleTypeDef hadc1;


// Wheel distance unit:m 
#define WIDTH_OF_CAR    (0.12f)
#define TSL_CLK (5)
#define TSL_SI  (6)

void read_ccd_data();
int read_encoder(uint8_t TIMX);
int find_ccd_center();

#endif