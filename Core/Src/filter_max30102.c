/*
 *
 * Filter for sensor
 *
 * Author: Joar Warholm
 * Created: 13 November 2024
 * 
 *
 * 
 * Adapted from:
 * https://morf.lv/implementing-pulse-oximeter-using-max30100
 */
#include "filter_max30102.h"
#include "max_sensor.h"
/**
 *DC removal filter
 */
DC_FILTER_T dcRemoval(float input, float prevState, float alpha)
{
    DC_FILTER_T output;
    output.w = input + alpha * prevState;
    output.result = output.w - prevState;
    return output;
}

/**
 * Butterworth filter.
 */
void lowPassButterworthFilter(float input, BUTTERWORTH_FILTER_T *filterState)
{
    filterState->v[0] = filterState->v[1];
    filterState->v[1] = (0.2452372752527856026 * input) + (0.50952544949442879485 * filterState->v[0]);
    filterState->result = filterState->v[0] + filterState->v[1];
}

/**
 *  Mean difference.
 */
float meanDiff(float input, MEAN_DIFF_FILTER_T *filterData)
{
    filterData->sum -= filterData->values[filterData->index];
    filterData->values[filterData->index] = input;
    filterData->sum += input;
    filterData->index = (filterData->index + 1) % MEAN_FILTER_SIZE;

    if (filterData->count < MEAN_FILTER_SIZE)
        filterData->count++;

    float mean = filterData->sum / filterData->count;
    return mean - input;
}
