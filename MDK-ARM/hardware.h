#ifndef __HARDWARE_H
#define __HARDWARE_H
#include "sys.h"
#include "stdio.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

// Wheel distance unit:m 
#define WIDTH_OF_CAR    (0.12f)
#define TSL_CLK GPIO_PIN_5
#define TSL_SI  GPIO_PIN_6

void read_ccd_data();
int find_ccd_center();
void show_ccd_data();

int read_encoder(uint8_t TIMX);

float get_battery_vol();

#endif