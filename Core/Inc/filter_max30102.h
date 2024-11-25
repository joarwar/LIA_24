/*
 *
 * Filters for sensor
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

#define ALPHA 0.95  
#define MEAN_FILTER_SIZE 15  

/* DC filter structure */
typedef struct {
    float w;      
    float result; 
} DC_FILTER_T;

/* Butterworth filter structure */
typedef struct {
    float v[2];  
    float result; 
} BUTTERWORTH_FILTER_T;

/* Mean difference filter structure */
typedef struct {
    float values[MEAN_FILTER_SIZE]; 
    uint8_t index;                  
    float sum;                      
    uint8_t count;                  
} MEAN_DIFF_FILTER_T;

/* Function prototypes */
DC_FILTER_T applyDCFilter(float input, float prevState, float alpha);
void applyLowPassFilter(float input, BUTTERWORTH_FILTER_T *filterState);
float calculateMeanDifference(float input, MEAN_DIFF_FILTER_T *filterData);

#endif /* FILTER_MAX30102_H */
