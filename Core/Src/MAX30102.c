/*
 *
 * Max30102.c
 *
 * Author: Joar Warholm
 * Created: 13 November 2024
 * 
 *
 * 
 * Adapted from:
 * https://morf.lv/implementing-pulse-oximeter-using-max30100
 */

#include "MAX30102.h"
#include "filter_max30102.h"
#include "stm32f1xx_hal.h"
#include "math.h"
#include "main.h"
#include <stm32f103xb.h>


I2C_HandleTypeDef hi2c1;

MAX30102 max_Sensor = {0};
FIFO_LED_DATA fifoData = {0};

//START SETTINGS

MEASURMENT_MODE measurment_mode = HEART_RATE;
POWER_MODE power_mode = NORMAL;
SAMPLE_RATE sample_rate = _400SPS;

//FILTERS

DC_FILTER_T dcFilterIR = {0};
DC_FILTER_T dcFilterRed = {0};
MEAN_DIFF_FILTER_T meanDiffIR = {0};
BUTTERWORTH_FILTER_T lpbFilterIR = {0};

//BPM 

float currentBPM;
float valuesBPM[PULSE_BPM_SAMPLE_SIZE] = {0};
float valuesBPMSum =0;
uint8_t valuesBPMCount = 0;
uint8_t bpmIndex = 0;
uint32_t lastBeatThreshold = 0;

//IR & RED

float irACValuesSqSum = 0;
float redACValueSqSum = 0;
uint16_t samplesRecorded = 0;
uint16_t pulseDetected = 0;
float currentSpO2Value = 0;

//CURRENT

uint8_t redLEDCurrent = 0;
uint8_t lastREDLedCurrentCheck = 0;

PULSE_STATE currentPulseDetectorState = PULSE_IDLE;

LEDCURRENT irLedCurrent;

void I2C_Init(void)
{
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	// Call the HAL error handler on error
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		errorHandler();
	}
}

int8_t MAX30102_redReg(uint8_t reg, uint8_t* value)
{
    HAL_StatusTypeDef retStatus;
    uint8_t buf[2];

    buf[0] = reg;    
    buf[1] = 0x03;   

    if (HAL_I2C_Master_Transmit(&hi2c1, MAX30102_I2C_ADDR_WRITE, buf, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        return -1;
    }

    if (HAL_I2C_Master_Receive(&hi2c1, MAX30102_I2C_ADDR_READ, buf, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        return -1;
    }

    *value = buf[0]; 

    return 0;
}


HAL_StatusTypeDef MAX30102_writeReg(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef retStatus;
    uint8_t buf[2];

    buf[0] = reg;
    buf[1] = value;

    uint8_t adress = (MAX30102_I2C_ADDR_READ | MAX30102_I2C_ADDR_WRITE);
    retStatus = HAL_I2C_Master_Transmit(&hi2c1, adress, buf, 2, HAL_MAX_DELAY);

    return retStatus;
}

void MAX30102_setMeasMode(MEASURMENT_MODE mode)
{
    int8_t readStatus = 0;
    uint8_t readResult;

    readStatus = MAX30102_readReg(MAX30102_MODE_CONFIG, &readResult);
    if (readStatus == -1){
        return;
    }

    readResult &= ~(0x7 << 0);
    
    switch(mode){
        case HEART_RATE: 
            readResult = readResult | (0x02 << 0); 
            break;
        case SP02:
            readResult = readResult | (0x03 << 0);
            break;
        case MULTI_LED:
            readResult = readResult | (0x07 << 0);
            break;
        default:
            return;
            break;
    }
    if (MAX30102_writeReg(MAX30102_MODE_CONFIG, readResult) != HAL_OK){
        return;
    }
    else {
        measurment_mode = mode;
    }

}

MEASURMENT_MODE MAX30102_getMeasMode(void)
{
    int8_t readStatus = 0;
    uint8_t readResult;

    readStatus = MAX30102_readReg(MAX30102_MODE_CONFIG, &readResult);
    if (readStatus == -1){
        return MEASURMENT_MODE_FAIL;
    }
    
    readResult &= 0x07;

    return (MEASURMENT_MODE)readResult;

    switch(readResult)
    {
        case 2:
            return HEART_RATE;
            break;
        case 3: 
            return SP02;
            break;
        case 7:
            return MULTI_LED;
            break;
        default:
            return HEART_RATE;
            break;
    }
}