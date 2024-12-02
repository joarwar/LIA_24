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


extern I2C_HandleTypeDef hi2c1;

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
		Error_Handler();
	}
}

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

    if (HAL_I2C_Master_Receive(&hi2c1, address, buf, 1, HAL_MAX_DELAY) != HAL_OK)
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

MAX30102 MAX30102_update(FIFO_LED_DATA m_fifoData)
{
    MAX30102 result = {
        .pulse_Detected = false,
        .heart_BPM = 0.0,
        .ir_Cardiogram = 0.0,
        .ir_Dc_Value = 0.0,
        .red_Dc_Value = 0.0,
        .SpO2 = max_Sensor.SpO2,
        .last_Beat_Threshold = 0,
        .dc_Filtered_IR = 0.0,
        .dc_Filtered_Red = 0.0,
        .temperature = 0.0
    };

    result.temperature = MAX30102_readTemp();

    // Apply DC Removal filter
    uart_PrintString("dcFilterIR ");
    uart_PrintFloat(dcFilterIR.w);
    uart_PrintString("ALPHA ");
    uart_PrintFloat(ALPHA);
    uart_PrintString("fifoData_IR ");
    uart_PrintFloat(m_fifoData.ir_led_raw);

    dcFilterIR = dcRemoval((float)m_fifoData.ir_led_raw, dcFilterIR.w, 0.95);  // alpha = 0.95
    dcFilterRed = dcRemoval((float)m_fifoData.red_led_raw, dcFilterRed.w, 0.95);  // alpha = 0.95
    


    uart_PrintString("Filtered_IR ");
    uart_PrintFloat(dcFilterIR.result );


    // Apply mean difference filter
    float meanDiffResIR = meanDiff(dcFilterIR.result, &meanDiffIR);
    lowPassButterworthFilter(meanDiffResIR, &lpbFilterIR);

    irACValueSqSum += dcFilterIR.result * dcFilterIR.result;
    redACValueSqSum += dcFilterRed.result * dcFilterRed.result;
    samplesRecorded++;

    // Detect pulse (check if the heart rate is fluctuating)
    if (detectPulse(lpbFilterIR.result) && samplesRecorded > 0) {
        result.pulse_Detected = true;
        pulsesDetected++;

        //float ratioRMS = log(sqrt(redACValueSqSum / samplesRecorded)) / log(sqrt(irACValueSqSum / samplesRecorded));
        //ax_Sensor.SpO2 = 110.0 - 18.0 * ratioRMS;
        //result.SpO2 = max_Sensor.SpO2;

        // Reset after a certain number of pulses
        if (pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0) {
            irACValueSqSum = 0;
            redACValueSqSum = 0;
            samplesRecorded = 0;
        }
    }

    balanceIntensity(dcFilterRed.w, dcFilterIR.w);

    // Final heart BPM calculation (from pulse detection logic)
    result.heart_BPM = max_Sensor.heart_BPM;
    result.ir_Cardiogram = lpbFilterIR.result;
    result.ir_Dc_Value = dcFilterIR.w;
    result.red_Dc_Value = dcFilterRed.w;
    result.last_Beat_Threshold = max_Sensor.last_Beat_Threshold;
    result.dc_Filtered_IR = dcFilterIR.result;
    result.dc_Filtered_Red = dcFilterRed.result;

    return result;
}

bool detectPulse(float sensor_value)
{
    static float prev_sensor_value = 0;
    static uint8_t values_went_down = 0;
    static uint32_t currentBeat = 0;
    static uint32_t lastBeat = 0;

    if (sensor_value > PULSE_MAX_THRESHOLD) {
        currentPulseDetectorState = PULSE_IDLE;
        prev_sensor_value = 0;
        lastBeat = 0;
        currentBeat = 0;
        values_went_down = 0;
        max_Sensor.last_Beat_Threshold = 0;
        return false;
    }

    switch (currentPulseDetectorState) {
        case PULSE_IDLE:
            if (sensor_value >= PULSE_MIN_THRESHOLD) {
                currentPulseDetectorState = PULSE_TRACE_UP;
                values_went_down = 0;
            }
            break;

        case PULSE_TRACE_UP:
            if (sensor_value > prev_sensor_value) {
                currentBeat = HAL_GetTick();
                max_Sensor.last_Beat_Threshold = sensor_value;
            } else {
                uint32_t beatDuration = currentBeat - lastBeat;
                lastBeat = currentBeat;

                float rawBPM = (beatDuration > 0) ? (60000.0 / (float)beatDuration) : 0;

                valuesBPM[bpmIndex] = rawBPM;
                valuesBPMSum = 0;

                for (int i = 0; i < PULSE_BPM_SAMPLE_SIZE; i++) {
                    valuesBPMSum += valuesBPM[i];
                }

                bpmIndex = (bpmIndex + 1) % PULSE_BPM_SAMPLE_SIZE;

                if (valuesBPMCount < PULSE_BPM_SAMPLE_SIZE) {
                    valuesBPMCount++;
                }

                max_Sensor.heart_BPM = valuesBPMSum / valuesBPMCount;

                currentPulseDetectorState = PULSE_TRACE_DOWN;

                return true;
            }
            break;

        case PULSE_TRACE_DOWN:
            if (sensor_value < prev_sensor_value) {
                values_went_down++;
            }

            if (sensor_value < PULSE_MIN_THRESHOLD) {
                currentPulseDetectorState = PULSE_IDLE;
            }
            break;
    }

    prev_sensor_value = sensor_value;
    return false;
}

void balanceIntensity(float redLedDC, float IRLedDC)
{
    uint32_t currentTime = HAL_GetTick();
    if (currentTime - lastREDLedCurrentCheck >= RED_LED_CURRENT_ADJUTSMENT_MS) {
        if (IRLedDC - redLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent < MAX30102_LED_CURRENT_51MA) {
            redLEDCurrent++;
            MAX30102_setLedCurrent(RED_LED, redLEDCurrent);
        } else if (redLedDC - IRLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent > 0) {
            redLEDCurrent--;
            MAX30102_setLedCurrent(RED_LED, redLEDCurrent);
        }

        lastREDLedCurrentCheck = HAL_GetTick();
    }
}

void MAX30102_registerData(void)
{
    int8_t readStatus;
    uint8_t readResult;

    readStatus = MAX30102_readReg(MAX30102_INTERRUPT_STATUS_1, &readResult);
    if (readStatus == -1)
    {
        uart_PrintString("I2C read issue");
        return;
    }

    uart_PrintString("MAX30102_INTERRUPT_STATUS_1: 0x");
    uart_PrintInt(readResult, 16);

    readStatus = MAX30102_readReg(MAX30102_INTERRUPT_STATUS_2, &readResult);
    if (readStatus == -1)
    {
        uart_PrintString("I2C read issue");
        return;
    }

	uart_PrintString("MAX30102_INTERRUPT_STATUS_2: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_INTERRUPT_ENABLE_1, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_INTERRUPT_ENABLE_1: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_INTERRUPT_ENABLE_2, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_INTERRUPT_ENABLE_2: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_FIFO_WRITE_POINTER, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_FIFO_WRITE_POINTER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_OVERFLOW_COUNTER, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_OVERFLOW_COUNTER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_FIFO_READ_POINTER, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_FIFO_READ_POINTER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_FIFO_DATA_REGISTER, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_FIFO_DATA_REGISTER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_FIFO_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_FIFO_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_MODE_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_MODE_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_SPO2_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_SPO2_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_LED1_PULSE, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_LED1_PULSE: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_LED2_PULSE, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_LED2_PULSE: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_MULTI_LED_CTRL_1, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_MULTI_LED_CTRL_1: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_MULTI_LED_CTRL_2, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_MULTI_LED_CTRL_2: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_DIE_TINT, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_DIE_TINT: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_DIE_TFRAC, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_DIE_TFRAC: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_DIE_TEMP_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_DIE_TEMP_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_REV_ID, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_REV_ID: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = MAX30102_readReg(MAX30102_PART_ID, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MAX30102_PART_ID: 0x");
	uart_PrintInt(readResult, 16);

	uart_PrintString("\r\n");

}

void MAX30102_displayData(void)
{

    // uart_PrintString("TEMP ");
    // uart_PrintFloat(max_Sensor.temperature);
    // uart_PrintString(" ----- \n");

    // uart_PrintString("BPM ");
    // uart_PrintFloat(max_Sensor.heart_BPM);


    // uart_PrintString("ir_dc  ");
    // uart_PrintFloat(max_Sensor.ir_Dc_Value);
    // uart_PrintString(" ----- \n");
    
    // uart_PrintString("dc_filter_ir ");
    // uart_PrintFloat(max_Sensor.ir_Dc_Value);
    // uart_PrintString(" ----- \n");

    // uart_PrintString("red_dc ");
    // uart_PrintFloat(max_Sensor.red_Dc_Value);


    // uart_PrintString("red_filter ");
    // uart_PrintFloat(max_Sensor.dc_Filtered_Red);


    // uart_PrintString("IR_cardiogram ");
    // uart_PrintFloat(max_Sensor.ir_Cardiogram);

}




