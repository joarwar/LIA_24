/*
 *
 * Filter for sensor
 *
 * Author: Joar Warholm
 * Created: 13 November 2024
 * 
 *
 * Adapted from:
 * https://morf.lv/implementing-pulse-oximeter-using-max30100
 */
#include "filter_max30102.h"
#include "max_sensor.h"
#include "uart.h"

static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,};
/*Working dc removal*/

DC_FILTER_T dcRemoval(float input, float prevState, float alpha)
{
    DC_FILTER_T output;
    output.w = input + alpha * prevState;
    output.result = output.w - prevState;          
    return output;
}

/*WIP mean filter => Moving average?*/
float meanDiff(float input, MEAN_DIFF_FILTER_T *filterData)
{
    float avg = 0; // average value
    
    // Ta bort gamla värde från pågående summa
    filterData->sum -= filterData->values[filterData->index];
    
    // Ersätt gamla värdet med nytt värde på index
    filterData->values[filterData->index] = input;
    
    // Lägg till nya värdet
    filterData->sum += filterData->values[filterData->index];
    
    // Gå till nästa steg
    filterData->index++;
    filterData->index = filterData->index % MEAN_FILTER_SIZE;

    // Check för kolla om tillräckligt med samples
    if (filterData->count < MEAN_FILTER_SIZE)
        filterData->count++;

    // Summan delat på antal
    avg = filterData->sum / filterData->count;

    // Diffen mellan avg och input
    return avg - input;

}

/*Attempt to add a lowpassButterworth*/
void lowPassButterworthFilter(float input, BUTTERWORTH_FILTER_T *filterState)
{
    filterState->v[0] = filterState->v[1];
    filterState->v[1] = (0.75 * input) + (0.75 * filterState->v[0]);
    //filterState->v[1] = (2.452372752527856026e-1 * input) + (0.50952544949442879485 * filterState->v[0]); //> Kanske bättre men mindre "peaks"
    filterState->result = filterState->v[0] + filterState->v[1];
}

// /*Moving average filter*/
void FIRFilter_Init(FIRFilter * fir)
{
    /*Rensar filter buffer*/
    for(uint8_t n = 0; n < FIR_FILTER_LENGTH; n++)
    {
        fir->buf[n] = 0.0f;
    }
    /*Rensar buffer index*/
    fir->bufIndex = 0;
    /*Rensar filter output*/
    fir->out = 0.0f;
}

float FIRFilter_Update(FIRFilter *fir, float inp) {

    /*Spara senaste värde i buffer*/
    fir->buf[fir->bufIndex] = inp;
    /* Öka index och wrappa runt om det behövs*/
    fir->bufIndex++;

    if (fir->bufIndex == FIR_FILTER_LENGTH)
    {
        fir->bufIndex = 0;
    }
    /*Uppskatta nya värdet med konvulation*/
    fir->out = 0.0f;

    uint8_t sumIndex = fir->bufIndex;

    for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++)
    {
        /*Sänk index om det behövs*/
        if (sumIndex > 0)
        {
            sumIndex--;   
        } 
        else
        {
            sumIndex = FIR_FILTER_LENGTH - 1;
        }
        /*Multiplicera impuls responsen med skiftad input värde och addera till output*/
        fir->out += FIR_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];


    }
    /*Skicka ut filtrerad output*/
    return fir->out;
}
// //LOW EMA
// void EMA_Low_Init(EMA_Low_H *filt, float alpha){
//     EMA_Low_SetAlpha(filt, alpha);
//     filt->out = 0.0f;
// }

// void EMA_Low_SetAlpha(EMA_Low_H *filt, float alpha){
    
//     if (alpha > 1.0f){
//         alpha = 1.0f;
//     }else if (alpha < 0.0f){
//         alpha = 0.0f;
//     }
//     filt->alpha = alpha;
// }

// float EMA_Low_Update(EMA_Low_H *filt, float inp){
    
//     filt->out = filt->alpha * inp + (1.0f - filt->alpha) *filt->out;
//     return filt->out;

// }
// //HIGH EMA
// void EMA_High_Init(EMA_High_H *filt, float beta){
//     EMA_High_SetBeta(filt, beta);
//     filt->inp = 0.0f;
//     filt->out = 0.0f;
// }

// void EMA_High_SetBeta(EMA_High_H *filt, float beta){
    
//     if (beta > 1.0f){
//         beta = 1.0f;
//     }else if (beta < 0.0f){
//         beta = 0.0f;
//     }
//     filt->beta = beta;
// }

// float EMA_High_Update(EMA_High_H *filt, float inp){
//     filt->out = 0.5f * (2.0f - filt->beta) * (inp - filt->inp) + (1.0f - filt->beta) * filt->out;
//     filt->inp = inp;

//     return filt->out;
// }






