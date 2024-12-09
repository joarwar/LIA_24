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
#include "uart.h"

/*Attempt to remove dc*/

DC_FILTER_T dcRemoval(float input, float prevState, float alpha)
{
    

    DC_FILTER_T output;
    output.w = input + alpha * prevState;
    output.result = input - output.w;           
    
    // uart_PrintString("red_dc (input) ");
    // uart_PrintFloat(input);
    // uart_PrintString("prevState: ");
    // uart_PrintFloat(prevState);           
    // uart_PrintString("output.w : ");
    // uart_PrintFloat(output.w);
    // uart_PrintString("output.result: ");
    // uart_PrintFloat(output.result);
    prevState = output.w;

    return output;
}

/*Attempt to add a lowpassButterworth*/
void lowPassButterworthFilter(float input, BUTTERWORTH_FILTER_T *filterState)
{
    // uart_PrintString("LowPass Butterworth: ");
    // uart_PrintFloat(input);
    filterState->v[0] = filterState->v[1];

    filterState->v[1] = (0.2452372752527856026 * input) + (0.50952544949442879485 * filterState->v[0]);  
    filterState->result = filterState->v[0] + filterState->v[1];

    // uart_PrintString("filterState->result: ");
    // uart_PrintFloat(filterState->result);
}


float meanDiff(float input, MEAN_DIFF_FILTER_T *filterData)
{
    float avg = 0;
    filterData->sum -= filterData->values[filterData->index];
    filterData->values[filterData->index] = input;
    filterData->sum += filterData->values[filterData->index];
    
    filterData->index++;
    filterData->index = filterData->index % MEAN_FILTER_SIZE;

    if (filterData->count < MEAN_FILTER_SIZE)
        filterData->count++;

    avg = filterData->sum / filterData->count;
    return avg - input;
}