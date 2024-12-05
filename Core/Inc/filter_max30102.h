/*
 *
 * Filter for sensor
 *
 * Author: Joar Warholm
 * Created:16 15 November 2024
 * 
 *
 * 
 * Adapted/taken from:
 * https://morf.lv/implementing-pulse-oximeter-using-max30100
 */

#ifndef FILTER_MAX30102_H
#define FILTER_MAX30102_H

#include "stm32f1xx_hal.h"
#include <stdint.h>


/* Filter parameters */
#define ALPHA 0.95  //dc filter alpha values = Statistical significance, higher value is consistent and lower is more changable
#define MEAN_FILTER_SIZE 15


typedef struct{
	float w;
	float result;
}DC_FILTER_T;

typedef struct
{
  float v[2];
  float result;
}BUTTERWORTH_FILTER_T;

typedef struct
{
  float values[MEAN_FILTER_SIZE];
  uint8_t index;
  float sum;
  uint8_t count;
}MEAN_DIFF_FILTER_T;

DC_FILTER_T dcRemoval(float input, float prevState, float alpha);
void lowPassButterworthFilter(float x, BUTTERWORTH_FILTER_T * filterResult);
float meanDiff(float M, MEAN_DIFF_FILTER_T* filterValues);


#endif /* FILTER_MAX30102_H */