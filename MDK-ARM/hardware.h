#ifndef __HARDWARE_H
#define __HARDWARE_H
#include "sys.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

// Wheel distance unit:m 
#define WIDTH_OF_CAR    (0.13f)
#define TSL_CLK GPIO_PIN_5
#define TSL_SI  GPIO_PIN_6

void read_ccd_data();
int find_ccd_center_1();
int find_ccd_center_2();
void show_ccd_data();

int read_encoder(uint8_t TIMX);

float get_battery_vol();
void commission_with_pc();

uint8_t* get_data_handle();

uint8_t auto_threshold_1(void);
uint8_t otsu_threshold();
uint8_t auto_threshold_3();
#define TSET_MODE

#endif