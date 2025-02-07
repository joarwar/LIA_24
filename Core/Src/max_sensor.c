/*
 *
 * Max30102
 *
 * Author: Joar Warholm
 * Created: 13 November 2024
 * 
 *
 * 
 * Adapted from:
 * https://morf.lv/implementing-pulse-oximeter-using-max30100
 */
#include "stm32f1xx_hal.h"
#include "max_sensor.h"
#include "filter_max30102.h"
#include "math.h"
#include <stm32f103xb.h>
#include "uart.h"
#include "stm32f1xx_hal_i2c.h"



extern I2C_HandleTypeDef hi2c1;

MAX30102 max_Sensor = {0};
FIFO_LED_DATA fifoData = {0};

//START SETTINGS

MEASURMENT_MODE measurment_mode = HEART_RATE;
POWER_MODE power_mode = NORMAL;
SAMPLE_RATE sample_rate = _100SPS;

//FILTERS

DC_FILTER_T dcFilterIR = {0};
DC_FILTER_T dcFilterRed = {0};
MEAN_DIFF_FILTER_T meanDiffIR = {0};
MEAN_DIFF_FILTER_T meanDiffRed = {0};
BUTTERWORTH_FILTER_T lpbFilterIR = {0};
BUTTERWORTH_FILTER_T lpbFilterRed = {0};
HP_FILTER_T hpbpFilterRed = {0};
FIRFilter filMovAvg;
FIRFilter withButter;
FIRFilter withMean;
EMA_Low_H filLowEmAvg;
EMA_High_H filHighEmAvg;



//BPM 

//Need to be change depending on PEAK value and light in room
#define PEAK_THRESHOLD 600
#define PEAK_MIN_DISTANCE 500
#define BPM_WINDOW 5

static uint32_t lastPeakTimeButterMA = 0;
static uint32_t lastPeakTimeButter = 0;
static uint32_t lastBPMTime = 0;
static uint32_t last30sTime = 0;
static uint32_t last60sTime = 0;

static float peakCount10sButterMA = 0;
static float peakCount30sButterMA = 0;
static float peakCount60sButterMA = 0;

static float peakCount10sButter = 0;
static float peakCount30sButter = 0;
static float peakCount60sButter = 0;

static float bpm10sButterHistory[BPM_WINDOW] = {0};
static float bpm30sButterHistory[BPM_WINDOW] = {0};
static float bpm60sButterHistory[BPM_WINDOW] = {0};

static float bpm10sButterMAHistory[BPM_WINDOW] = {0};
static float bpm30sButterMAHistory[BPM_WINDOW] = {0};
static float bpm60sButterMAHistory[BPM_WINDOW] = {0};

static int bpmIndex10s = 0;
static int bpmIndex30s = 0;
static int bpmIndex60s = 0;

static int count10s = 0;
static int count30s = 0;
static int count60s = 0;


//HRV

#define HRV_WINDOW 50
#define PEAK_THRESHOLD_HRV 1100

float rrIntervals[HRV_WINDOW] = {0};  
static int rrIndex = 0; 
int countHRV = 0;
static uint32_t lastPeakTime = 0;  



//IR & RED

float irACValueSqSum = 0;
float redACValueSqSum = 0;
uint16_t samplesRecorded = 0;
uint16_t pulsesDetected = 0;
float currentSpO2Value = 0;

//CURRENT

uint8_t redLEDCurrent = 0;
uint8_t lastREDLedCurrentCheck = 0;

PULSE_STATE currentPulseDetectorState = PULSE_IDLE;

LEDCURRENT irLedCurrent;
uint16_t counter = 0;

// void I2C_Init(void)
// {
// 	hi2c1.Instance = I2C1;
// 	hi2c1.Init.ClockSpeed = 100000;
// 	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
// 	hi2c1.Init.OwnAddress1 = 0;
// 	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
// 	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
// 	hi2c1.Init.OwnAddress2 = 0;
// 	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
// 	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

// 	// Call the HAL error handler on error
// 	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
// 	{
// 		Error_Handler();
// 	}
// }

int8_t MAX30102_readReg(uint8_t reg, uint8_t* value)
{
    HAL_StatusTypeDef retStatus;
    uint8_t buf[2];

    buf[0] = reg;    
    buf[1] = 0x03;
    
    uint8_t address = (MAX30102_I2C_ADDR_S | MAX30102_I2C_ADDR_WRITE);
    retStatus = HAL_I2C_Master_Transmit(&hi2c1, address, buf, 1, HAL_MAX_DELAY);
    if (retStatus != HAL_OK)
    {
        return -1;
    }
    address = (MAX30102_I2C_ADDR_S | MAX30102_I2C_ADDR_READ);
    
    retStatus = HAL_I2C_Master_Receive(&hi2c1, address, buf, 1, HAL_MAX_DELAY);

    if (retStatus != HAL_OK)
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

    uint8_t adress = (MAX30102_I2C_ADDR_S | MAX30102_I2C_ADDR_WRITE);
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
    if(readStatus == -1)
    {
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
    }
}

void MAX30102_setPowerMode(POWER_MODE mode)
{
    int8_t readStatus = 0;
    uint8_t readResult;

    readStatus = MAX30102_readReg(MAX30102_MODE_CONFIG, &readResult);
    if (readStatus == -1)
    {
        return;
    }
    
    readResult &= ~(0x80 << 0);
    switch(mode)
    {
        case NORMAL:
            readResult = readResult | (0x00 << 0 );
            break;
        case LOW_POWER:
            readResult = readResult | (0x80 << 0);
            break;
        default:
            return;
            break;
    }

    if (MAX30102_writeReg(MAX30102_MODE_CONFIG, readResult) != HAL_OK){
        return;
    }
    else{
        measurment_mode = mode;
    }
}

POWER_MODE MAX30102_getPowerMode(void)
{
    int8_t readStatus = 0;
    uint8_t readResult;

    readStatus = MAX30102_readReg(MAX30102_MODE_CONFIG, &readResult);
    if (readStatus == -1)
    {
        return POWER_MODE_FAIL;
    }
    
    readResult &= 0x80;

    switch(readResult)
    {
        case 0:
            return NORMAL;
            break;
        case 0x80:
            return LOW_POWER;
            break;
        default:
            return NORMAL;
            break;
        
    }
}

void MAX30102_resetRegister(void)
{
    int8_t readStatus;
    uint8_t readResult;

    readStatus = MAX30102_readReg(MAX30102_MODE_CONFIG, &readResult);
    if (readStatus == -1)
    {
        return;
    }
    readResult &= ~(0x01 << 6);
    readResult = readResult | (0x01 << 6);
    if( MAX30102_writeReg( MAX30102_MODE_CONFIG, readResult) != HAL_OK){
        return;
    }
}

void MAX30102_setLedCurrent(uint8_t led, float currentLevel)
{
    uint8_t value = 0;
    uint8_t ledRegister = 0;

    switch(led)
    {
        case RED_LED:
            ledRegister = MAX30102_LED1_PULSE;
            break;
        case IR_LED:
            ledRegister = MAX30102_LED2_PULSE;
            break;
    }

    value = (uint8_t)(5.0 * currentLevel);

    if (MAX30102_writeReg(ledRegister, value) != HAL_OK)
    {
        return;
    }
    else{

    }
}

float MAX30102_getLedCurrent(uint8_t led)
{
    uint8_t readStatus = 0;
    float currentLevel;
    uint8_t ledRegister = 0;
    uint8_t readResult;

    switch(led){
        case RED_LED:
            ledRegister = MAX30102_LED1_PULSE;
            break;
        case IR_LED:
            ledRegister = MAX30102_LED2_PULSE;
            break;
    }

    readStatus = MAX30102_readReg(ledRegister, &readResult);
    if (readStatus == -1)
    {
        return -1;
    } 
    currentLevel = readResult / 5.0;

    return currentLevel;
}

void MAX30102_setSampleRate(uint8_t rate)
{
    int8_t readStatus = 0;
    uint8_t readResult;

    readStatus = MAX30102_readReg(MAX30102_SPO2_CONFIG, &readResult);
    if (readStatus == -1)
    {
        return;
    }
    readResult &= ~(0x1C << 0);

    readResult = readResult | (rate << 2);

    if (MAX30102_writeReg (MAX30102_SPO2_CONFIG, readResult) != HAL_OK)
    {
        return;
    }
    else{

    }
}

SAMPLE_RATE MAX30102_getSampleRate(void)
{
    int8_t readStatus = 0;
    uint8_t result;
    uint8_t readResult;

    readStatus = MAX30102_readReg(MAX30102_SPO2_CONFIG, &readResult);
    if (readStatus == -1)
    {
        return _SAMPLE_FAIL;
    }
    
    result = readResult;
    result &= 0x1C;
    result = result >> 2;

    return (SAMPLE_RATE)result;
}

void MAX30102_setPulseWidth(uint8_t width)
{
    int8_t readStatus = 0;
    uint8_t readResult;

    readStatus = MAX30102_readReg(MAX30102_SPO2_CONFIG, &readResult);
    if (readStatus == -1)
    {
        return;
    }

    readResult &= ~(0x03 << 0);
    
    switch(width)
    {
        case _69_US:
            readResult = readResult | 0;
            break;
        case _118_US:
            readResult = readResult | (0x01 << 0);
            break;
        case _215_US:
            readResult = readResult | (0x02 << 0);
            break;
        case _411_US:
            readResult = readResult | (0x03 << 0);
            break;
        case _PULSE_WIDTH_FAIL:
            break;
    }
    if (MAX30102_writeReg (MAX30102_SPO2_CONFIG, readResult ) != HAL_OK)
    {
        return;
    }
    else{

    }

}

PULSE_WIDTH MAX30102_getPulseWidth(void)
{
    int8_t readStatus = 0;
    uint8_t result;
    uint8_t readResult;

    readStatus = MAX30102_readReg(MAX30102_SPO2_CONFIG, &readResult);
    if (readStatus == -1)
    {
        return _PULSE_WIDTH_FAIL;

    }
    result = readResult;
    result &= 0x03;

    return (PULSE_WIDTH)result;

}

void MAX30102_resetFIFO(void)
{
    MAX30102_writeReg(MAX30102_FIFO_WRITE_POINTER, 0);
    MAX30102_writeReg(MAX30102_FIFO_READ_POINTER, 0);
    MAX30102_writeReg(MAX30102_OVERFLOW_COUNTER, 0);
}

void MAX30102_initFIFO(void)
{
    if (INTERRUPT == 1 )
    {
        MAX30102_writeReg(MAX30102_FIFO_CONFIG, 0x0F);

        MAX30102_writeReg(MAX30102_INTERRUPT_ENABLE_1, 0x40);

        MAX30102_clearInterrupt();

    }else{
        MAX30102_writeReg(MAX30102_FIFO_CONFIG, 0x0F);
        MAX30102_writeReg(MAX30102_INTERRUPT_ENABLE_1, 0x00);
    }
}

FIFO_LED_DATA MAX30102_read_FIFO(void)
{
    uint8_t address;
    uint8_t buf[12];
    uint8_t numBytes = 6;
    
    buf[0] = MAX30102_FIFO_DATA_REGISTER;
    address = (MAX30102_I2C_ADDR_S | MAX30102_I2C_ADDR_WRITE);
    HAL_I2C_Master_Transmit(&hi2c1, address, buf, 1, HAL_MAX_DELAY);

    address = (MAX30102_I2C_ADDR_S| MAX30102_I2C_ADDR_READ);
    HAL_I2C_Master_Receive(&hi2c1, address, buf, numBytes, HAL_MAX_DELAY);

    fifoData.ir_led_raw = 0;
    fifoData.red_led_raw = 0;

    fifoData.ir_led_raw = (buf[4] << 8) | (buf[5] << 0);
    fifoData.red_led_raw = (buf[1] << 8) | (buf[0] << 0);

    return fifoData;
}

int8_t MAX30102_readFIFO(uint8_t* dataBuf, uint8_t numBytes)
{
    HAL_StatusTypeDef retStatus;
    uint8_t buf[12];

    buf[0] = MAX30102_FIFO_DATA_REGISTER;

    uint8_t address = (MAX30102_I2C_ADDR_S | MAX30102_I2C_ADDR_WRITE);
    
    retStatus = HAL_I2C_Master_Transmit(&hi2c1, address, buf, 1, HAL_MAX_DELAY);
    if (retStatus != HAL_OK)
    {
        return -1;
    }

    address = (MAX30102_I2C_ADDR_S | MAX30102_I2C_ADDR_READ);
    retStatus = HAL_I2C_Master_Receive(&hi2c1, address, buf, numBytes, HAL_MAX_DELAY);
    if (retStatus != HAL_OK ){
		return -1;
	}

	for(int i = 0; i < numBytes; i++)
	{
		*dataBuf = buf[i];
		dataBuf++;
	}

	return 0;

}

void MAX30102_clearInterrupt(void)
{
    uint8_t readResult;
    MAX30102_readReg(MAX30102_INTERRUPT_STATUS_1, &readResult);
}

float MAX30102_readTemp(void)
{
    uint8_t tempReady = 1;
    uint8_t readResult;
    int8_t tempFraction;
    uint8_t tempInteger;
    float temperature;

    MAX30102_writeReg(MAX30102_DIE_TEMP_CONFIG, 1);

    while (tempReady != 0)
    {
        MAX30102_readReg(MAX30102_DIE_TEMP_CONFIG, &tempReady);
    }

    MAX30102_readReg(MAX30102_DIE_TINT, &readResult);
    tempInteger = readResult;

    MAX30102_readReg(MAX30102_DIE_TFRAC, &readResult);
    tempFraction = readResult;

    temperature = tempInteger + (tempFraction * MAX30102_DIE_TFRAC_INCREMENT);

    return temperature;
}

float calculateSDNN() {
    if (countHRV < 2) return 0.0;  
    
    float sum = 0.0;
    float mean = 0.0;
    for (int i = 0; i < countHRV; i++) {
        sum += rrIntervals[i];
    }
    mean = sum / countHRV;

    float varianceSum = 0.0;
    for (int i = 0; i < countHRV; i++) {
        varianceSum += (rrIntervals[i] - mean) * (rrIntervals[i] - mean);
    }
    float sdnn = sqrt(varianceSum / countHRV);  
    return sdnn;
}

float calculateRMSSD() {
    if (countHRV < 2) return 0.0;

    float sum = 0.0;
    for (int i = 1; i < countHRV; i++) {
        float diff = rrIntervals[i] - rrIntervals[i - 1];
        sum += diff * diff;
    }

    return sqrt(sum / (countHRV - 1));
}


void updateRRInterval(uint32_t peakTime) {
    uint32_t rrInterval = peakTime - lastPeakTime;  

    rrIntervals[rrIndex] = rrInterval;
    rrIndex = (rrIndex + 1) % HRV_WINDOW;
    if (countHRV < HRV_WINDOW) {
        countHRV++;  
    }

    lastPeakTime = peakTime;  
}

MAX30102 MAX30102_update(FIFO_LED_DATA m_fifoData) {
    MAX30102 result = {0};
    result.temperature = MAX30102_readTemp();
    static float currentLevel = 5.0;
    if (m_fifoData.red_led_raw > 9000) {
        if (m_fifoData.red_led_raw < 30000)
        {
            float currentIncrease = 0.05;     
            currentLevel += currentIncrease;  

            MAX30102_setLedCurrent(RED_LED, currentLevel);
        }

        else if (m_fifoData.red_led_raw > 55000)
        {
            float currentDecrease = 0.05;
            currentLevel -= currentDecrease;

            MAX30102_setLedCurrent(RED_LED, currentLevel);        
        }
        
        else
        {
            dcFilterRed = dcRemoval((float)m_fifoData.red_led_raw, dcFilterRed.w, 0.95);
            float meanDiffResRed = meanDiff(dcFilterRed.result, &meanDiffRed);
            lowPassButterworthFilter(meanDiffResRed, &lpbFilterRed);
            FIRFilter_Update(&withButter, lpbFilterRed.result); 

            uint32_t now = HAL_GetTick();

            if (lpbFilterRed.result > PEAK_THRESHOLD && (now - lastPeakTimeButter > PEAK_MIN_DISTANCE)) {
                peakCount10sButter++;
                peakCount30sButter++;
                peakCount60sButter++;
                lastPeakTimeButter = now;
                updateRRInterval(now);
            }

            if (withButter.out > PEAK_THRESHOLD && (now - lastPeakTimeButterMA > PEAK_MIN_DISTANCE)) {
                peakCount10sButterMA++;
                peakCount30sButterMA++;
                peakCount60sButterMA++;
                lastPeakTimeButterMA = now;
                updateRRInterval(now);
            }

            if (now - lastBPMTime >= 10000) {
                float bpm10sButterMA = (peakCount10sButterMA * 6.0);  
                float bpm10sButter = (peakCount10sButter * 6.0);  
                peakCount10sButterMA = 0;
                peakCount10sButter = 0;
                lastBPMTime = now;

                bpm10sButterHistory[bpmIndex10s] = bpm10sButter;
                bpm10sButterMAHistory[bpmIndex10s] = bpm10sButterMA;
                bpmIndex10s = (bpmIndex10s + 1) % BPM_WINDOW;
                if (count10s < BPM_WINDOW) count10s++;


                float avgBpm10sButter = 0, avgBpm10sButterMA = 0;
                for (int i = 0; i < count10s; i++) {
                    avgBpm10sButter += bpm10sButterHistory[i];
                    avgBpm10sButterMA += bpm10sButterMAHistory[i];
                }
                avgBpm10sButter /= count10s;
                avgBpm10sButterMA /= count10s;

                float sdnn = calculateSDNN();
                float rmssd = calculateRMSSD();

                uart_PrintString(" BPM (10s ButterMA): ");
                uart_PrintFloat(bpm10sButterMA);
                uart_PrintString(" | Smoothed: ");
                uart_PrintFloat(avgBpm10sButterMA);
                uart_PrintString("\n");

                uart_PrintString(" BPM (10s Butter): ");
                uart_PrintFloat(bpm10sButter);
                uart_PrintString(" | Smoothed: ");
                uart_PrintFloat(avgBpm10sButter);
                uart_PrintString("\n");

                uart_PrintString(" HRV (SDNN): ");
                uart_PrintFloat(sdnn);
                uart_PrintString("\n");

                uart_PrintString(" HRV (RMSSD): ");
                uart_PrintFloat(rmssd);
                uart_PrintString("\n");
            }


            if (now - last30sTime >= 30000) {
                float bpm30sButterMA = (peakCount30sButterMA * 2);  
                float bpm30sButter = (peakCount30sButter * 2);  
                peakCount30sButter = 0;
                last30sTime = now;


                bpm30sButterHistory[bpmIndex30s] = bpm30sButter;
                bpm30sButterMAHistory[bpmIndex30s] = bpm30sButterMA;
                bpmIndex30s = (bpmIndex30s + 1) % BPM_WINDOW;
                if (count30s < BPM_WINDOW) count30s++;

                float avgBpm30sButter = 0, avgBpm30sButterMA = 0;
                for (int i = 0; i < count30s; i++) {
                    avgBpm30sButter += bpm30sButterHistory[i];
                    avgBpm30sButterMA += bpm30sButterMAHistory[i];
                }
                avgBpm30sButter /= count30s;
                avgBpm30sButterMA /= count30s;


                uart_PrintString(" BPM (30s ButterMA): ");
                uart_PrintFloat(bpm30sButterMA);
                uart_PrintString(" | Smoothed: ");
                uart_PrintFloat(avgBpm30sButterMA);
                uart_PrintString("\n");

                uart_PrintString(" BPM (30s Butter): ");
                uart_PrintFloat(bpm30sButter);
                uart_PrintString(" | Smoothed: ");
                uart_PrintFloat(avgBpm30sButter);
                uart_PrintString("\n");
            }


            if (now - last60sTime >= 60000) {
                float bpm60sButterMA = (peakCount60sButterMA * 1);  
                float bpm60sButter = (peakCount60sButter * 1);  
                peakCount60sButterMA = 0;
                peakCount60sButter = 0;
                last60sTime = now;


                bpm60sButterHistory[bpmIndex60s] = bpm60sButter;
                bpm60sButterMAHistory[bpmIndex60s] = bpm60sButterMA;
                bpmIndex60s = (bpmIndex60s + 1) % BPM_WINDOW;
                if (count60s < BPM_WINDOW) count60s++;


                float avgBpm60sButter = 0, avgBpm60sButterMA = 0;
                for (int i = 0; i < count60s; i++) {
                    avgBpm60sButter += bpm60sButterHistory[i];
                    avgBpm60sButterMA += bpm60sButterMAHistory[i];
                }
                avgBpm60sButter /= count60s;
                avgBpm60sButterMA /= count60s;


                uart_PrintString(" BPM (60s ButterMA): ");
                uart_PrintFloat(bpm60sButterMA);
                uart_PrintString(" | Smoothed: ");
                uart_PrintFloat(avgBpm60sButterMA);
                uart_PrintString("\n");

                uart_PrintString(" BPM (60s Butter): ");
                uart_PrintFloat(bpm60sButter);
                uart_PrintString(" | Smoothed: ");
                uart_PrintFloat(avgBpm60sButter);
                uart_PrintString("\n");
            }
            float filMovInverse = (filMovAvg.out * -1);
            float dummyGain = (lpbFilterRed.result * 3);
            uart_PrintString("$");
            uart_PrintFloat(m_fifoData.red_led_raw);
            uart_PrintString(" ");
            uart_PrintFloat(dcFilterRed.result);
            uart_PrintString(" ");
            uart_PrintFloat(meanDiffResRed);
            uart_PrintString(" ");
            //uart_PrintFloat(filMovAvg.out);
            uart_PrintFloat(filMovInverse);
            uart_PrintString(" ");
            uart_PrintFloat(lpbFilterRed.result);
            uart_PrintString(" ");
            uart_PrintFloat(withButter.out);
            uart_PrintString(" ");
            uart_PrintFloat(withMean.out);
            uart_PrintString(" ");
            uart_PrintFloat(lpbFilterRed.result );
            uart_PrintString(" ");
            uart_PrintFloat(dummyGain);
            uart_PrintString(";");
        } 
    } 
    else {
         uart_PrintString("$");
        uart_PrintFloat(m_fifoData.red_led_raw);
        uart_PrintString(";");
    }

    return result;
}



// void balanceIntensity(float redLedDC, float IRLedDC)
// {
//     uint32_t currentTime = HAL_GetTick();
//     if (currentTime - lastREDLedCurrentCheck >= RED_LED_CURRENT_ADJUTSMENT_MS) {
//         if (IRLedDC - redLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent < MAX30102_LED_CURRENT_51MA) {
//             redLEDCurrent++;
//             MAX30102_setLedCurrent(RED_LED, redLEDCurrent);
//         } else if (redLedDC - IRLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent > 0) {
//             redLEDCurrent--;
//             MAX30102_setLedCurrent(RED_LED, redLEDCurrent);
//         }

//         lastREDLedCurrentCheck = HAL_GetTick();
//     }
// }

void MAX30102_registerData(int voixd)
{
    int8_t readStatus;
    uint8_t readResult;

    readStatus = MAX30102_readReg(MAX30102_INTERRUPT_STATUS_1, &readResult);
    if (readStatus == -1)
    {
        uart_PrintString("I2C read issue");
        return;
    }

    uart_PrintString(" MAX30102_INTERRUPT_STATUS_1: 0x");
    uart_PrintInt(readResult, 16);

    readStatus = MAX30102_readReg(MAX30102_INTERRUPT_STATUS_2, &readResult);
    if (readStatus == -1)
    {
        uart_PrintString("I2C read issue");
        return;
    }

	uart_PrintString(" MAX30102_INTERRUPT_STATUS_2: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_INTERRUPT_ENABLE_1, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_INTERRUPT_ENABLE_1: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_INTERRUPT_ENABLE_2, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_INTERRUPT_ENABLE_2: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_FIFO_WRITE_POINTER, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_FIFO_WRITE_POINTER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_OVERFLOW_COUNTER, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_OVERFLOW_COUNTER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_FIFO_READ_POINTER, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_FIFO_READ_POINTER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_FIFO_DATA_REGISTER, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_FIFO_DATA_REGISTER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_FIFO_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_FIFO_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_MODE_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_MODE_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_SPO2_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_SPO2_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_LED1_PULSE, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_LED1_PULSE: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_LED2_PULSE, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_LED2_PULSE: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_MULTI_LED_CTRL_1, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_MULTI_LED_CTRL_1: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_MULTI_LED_CTRL_2, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_MULTI_LED_CTRL_2: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_DIE_TINT, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_DIE_TINT: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_DIE_TFRAC, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_DIE_TFRAC: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_DIE_TEMP_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_DIE_TEMP_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_REV_ID, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_REV_ID: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_PART_ID, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString(" MAX30102_PART_ID: 0x");
	uart_PrintInt(readResult, 16);

	uart_PrintString("\r\n");

}

void MAX30102_displayData(void)
{

    // uart_PrintString("TEMP ");
    // uart_PrintFloat(max_Sensor.temperature);


    //uart_PrintString("BPM ");
    //uart_PrintFloat(max_Sensor.heart_BPM);


    // uart_PrintString("ir_dc,");
    // uart_PrintFloat(max_Sensor.ir_Dc_Value);
    
    // uart_PrintString("dc_filter_ir ");
    // uart_PrintFloat(max_Sensor.ir_Dc_Value);

    // uart_PrintString("red_dc ");
    // uart_PrintFloat(max_Sensor.red_Dc_Value);


    // uart_PrintString("red_filter ");
    // uart_PrintFloat(max_Sensor.dc_Filtered_Red);


    // uart_PrintString("IR_cardiogram ");
    // uart_PrintFloat(max_Sensor.ir_Cardiogram);

}





