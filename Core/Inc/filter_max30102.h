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
#define ALPHA 0.95  //dc filter alpha values = Statistical significance, higher value is consistent and lower is inconsistent
#define MEAN_FILTER_SIZE 10 //
#define FIR_FILTER_LENGTH 10
#define EMA


typedef struct
{
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
  float v_low[2];
  float v_high[2];
  float result;
}HP_FILTER_T;

typedef struct
{
  float values[MEAN_FILTER_SIZE];
  uint8_t index;
  float sum;
  uint8_t count;
}MEAN_DIFF_FILTER_T;

typedef struct
{
  float buf[FIR_FILTER_LENGTH];
  uint8_t bufIndex;

  float out;
}FIRFilter;

typedef struct
{
  float alpha;
  float out;
}EMA_Low_H;

typedef struct
{
  float beta;
  float inp;
  float out;
}EMA_High_H;


DC_FILTER_T dcRemoval(float input, float prevState, float alpha);
void lowPassButterworthFilter(float x, BUTTERWORTH_FILTER_T * filterResult);
// void lowPassButterworthFilterCoeff(float x, BUTTERWORTH_FILTER_T * filterResult);
float meanDiff(float M, MEAN_DIFF_FILTER_T* filterValues);


// void bandPassFilter( float x, HP_FILTER_T * filterResult);
void FIRFilter_Init(FIRFilter *fir);
float FIRFilter_Update(FIRFilter * fir, float inp);


// void EMA_Low_Init(EMA_Low_H *filt, float alpha);
// void EMA_Low_SetAlpha(EMA_Low_H *filt, float alpha);
// float EMA_Low_Update (EMA_Low_H *filt, float inp);

// void EMA_High_Init(EMA_High_H *filt, float beta);
// void EMA_High_SetBeta(EMA_High_H *filt, float beta);
// float EMA_High_Update(EMA_High_H *filt, float inp);

#endif /* FILTER_MAX30102_H */