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

#ifndef MAX30102_I2C_DRIVER_H
#define MAX30102_I2C_DRIVER_H

#include "stm32f1xx_hal.h" 
#include <stdbool.h>
#include <stdio.h>


/* PULSE DETECTION PARAMETERS */
#define PULSE_MIN_THRESHOLD 250 // 300 for finger and around 20 for wrist
#define PULSE_MAX_THRESHOLD 800
#define PULSE_GO_DOWN_THRESHOLD 1

#define PULSE_BPM_SAMPLE_SIZE 10

/*SpO2*/
#define RESET_SPO2_EVERY_N_PULSES 4

/*RED LED CURRENT BALANCE*/
#define MAGIC_ACCEPTABLE_INTENSITY_DIFF 65000
#define RED_LED_CURRENT_ADJUTSMENT_MS 500

//p.23 
#define DEFAULT_SAMPLING_RATE MAX30102_SAMPLING_RATE_100HZ 
#define DEFAULT_PULSE_WIDTH MAX30102_PULSE_WIDTH_411US_ADC_18

//p.20
#define DEFAULT_IR_LED_CURRENT MAX30102_LED_CURRENT_50MA
#define STARTING_RED_LED_CURRENT MAX30102_CURRENT_25_4MA

/*
 * DEFINES
 */

#define MAX30102_I2C_ADDR_READ 0x01
#define MAX30102_I2C_ADDR_WRITE 0x00
#define MAX30102_I2C_ADDR_S 0xAE
#define MAX30102_I2C_ADDR 0x57 

/*
 * PART ID 
 */

#define MAX30102_PART_ID 0xFF
#define MAX30102_REV_ID 0xFE

/*
 * REGISTERS (p.10-11)
 */
// STATUS
#define MAX30102_INTERRUPT_STATUS_1 0x00
#define MAX30102_INTERRUPT_STATUS_2 0x01
#define MAX30102_INTERRUPT_ENABLE_1 0x02
#define MAX30102_INTERRUPT_ENABLE_2 0x03
#define MAX30102_INTERRUPT_A_FULL 7
#define MAX30102_INTERRUPT_PPG_RDY 6
#define MAX30102_INTERRUPT_ALC_OVF 5
#define MAX30102_INTERRUPT_DIE_TEMP_RDY 1

//FIFO
#define MAX30102_FIFO_WRITE_POINTER 0x04
#define MAX30102_OVERFLOW_COUNTER 0x05
#define MAX30102_FIFO_READ_POINTER 0x06

#define MAX30102_FIFO_DATA_REGISTER 0x07

#define MAX30102_FIFO_CONFIG 0x08
#define MAX30102_FIFO_CONFIG_SMP_AVE 5
#define MAX30102_FIFO_CONFIG_ROLL_OVER_EN 4
#define MAX30102_FIFO_CONFIG_FIFO_A_FULL 0

//CONFIG
#define MAX30102_MODE_CONFIG 0x9
#define MAX30102_MODE_CONFIG_RESET 6
#define MAX30102_MODE_CONFIG_SHDN 7
#define MAX30102_MODE_CONFIG_MODE 0

#define MAX30102_SPO2_CONFIG 0x0A
#define MAX30102_SPO2_CONFIG_ADC_RGE 5
#define MAX30102_SPO2_CONFIG_SR 2
#define MAX30102_SPO2_CONFIG_LED_PW 0

#define MAX30102_LED1_PULSE 0x0C
#define MAX30102_LED2_PULSE 0x0D

#define MAX30102_MULTI_LED_CTRL_1 0x11
#define MAX30102_MULTI_LED_CTRL_SLOT2 4
#define MAX30102_MULTI_LED_CTRL_SLOT1 0
#define MAX30102_MULTI_LED_CTRL_2 0x12
#define MAX30102_MULTI_LED_CTRL_SLOT4 4
#define MAX30102_MULTI_LED_CTRL_SLOT3 0

//DIE TEMP
#define MAX30102_DIE_TINT 0x1F
#define MAX30102_DIE_TFRAC 0x20
#define MAX30102_DIE_TFRAC_INCREMENT 0.0625f
#define MAX30102_DIE_TEMP_CONFIG 0x21
#define MAX30102_DIE_TEMP_EN 1

#define RED_LED 1
#define IR_LED 2

#define INTERRUPT 0

#define DATA_READ_FAIL 0xFFFFFFFF

/*
 * SENSOR STRUCT * ENUMS
 */

typedef enum{
    HEART_RATE,
    SP02,
    MULTI_LED,
    MEASURMENT_MODE_FAIL
} MEASURMENT_MODE;

typedef enum{
    NORMAL,
    LOW_POWER,
    POWER_MODE_FAIL
} POWER_MODE;

typedef enum{
	_50SPS,
	_100SPS,
	_200SPS,
	_400SPS,
	_800SPS,
	_1000SPS,
	_1600SPS,
	_3200SPS,
	_SAMPLE_FAIL
}SAMPLE_RATE;

typedef enum{
	_69_US,
	_118_US,
	_215_US,
	_411_US,
	_PULSE_WIDTH_FAIL
}PULSE_WIDTH;

extern MEASURMENT_MODE measurment_mode;
extern POWER_MODE power_mode;
extern SAMPLE_RATE sample_rate;
extern PULSE_WIDTH pulse_width;

typedef struct {
    uint32_t ir_led_raw;
    uint32_t red_led_raw;
}FIFO_LED_DATA;

typedef enum LEDCURRENT {
    MAX30102_LED_CURRENT_0MA = 0x00,
    MAX30102_LED_CURRENT_0_2MA = 0x01,
    MAX30102_LED_CURRENT_0_4MA = 0x02,
    MAX30102_LED_CURRENT_3MA = 0x0F,
    MAX30102_LED_CURRENT_6_2MA = 0x1F,
    MAX30102_LED_CURRENT_12_6MA = 0x3F,
    MAX30102_LED_CURRENT_25_4MA = 0x7F,
    MAX30102_LED_CURRENT_51MA = 0xFF

}LEDCURRENT;

typedef struct {
    /* I2C Handle*/
    I2C_HandleTypeDef *i2cHandle;
    bool pulse_Detected;
    float heart_BPM;
    float ir_Cardiogram;
    float ir_Dc_Value;
    float red_Dc_Value;
    float SpO2;
    uint32_t last_Beat_Threshold;
    float dc_Filtered_IR;
    float dc_Filtered_Red;
    float temperature;
} MAX30102;

extern MAX30102 max_Sensor;

typedef enum {
    PULSE_IDLE,
    PULSE_TRACE_UP,
    PULSE_TRACE_DOWN
}PULSE_STATE;

extern PULSE_STATE currentPulseDetectorState;

/*
 * INITIALISATION
 */
void I2C_Init(void);

//

/*
 * TEMPLATES FUNC
 */
void MAX30102_setMeasMode(uint8_t mode);
MEASURMENT_MODE MAX30102_getMeasMode(void);

void MAX30102_setPowerMode (uint8_t mode);
POWER_MODE MAX30102_getPowerMode(void);

void MAX30102_setSampleRate (uint8_t rate);
SAMPLE_RATE MAX30102_getSampleRate (void);

void MAX30102_setPulseWidth(uint8_t width);
PULSE_WIDTH MAX30102_getPulseWidth(void);

void MAX30102_setLedCurrent(uint8_t led, float currentLevel);
float MAX30102_getLedCurrent(uint8_t);

int8_t MAX30102_readFIFO(uint8_t* dataBuf, uint8_t numBytes);
void MAX30102_resetRegister(void);
void MAX30102_resetFIFO(void);
FIFO_LED_DATA MAX30102_read_FIFO(void);
void MAX30102_initFIFO(void);

float MAX30102_readTemp(void);
void MAX30102_clearInterrupt(void);
MAX30102 MAX30102_update(FIFO_LED_DATA m_fifoData);
bool detectPulse(float sensor_value);
void balanceIntensity(float redLedDC, float IRLedDC);

void MAX30102_displayData(void);

int8_t MAX30102_readReg(uint8_t reg, uint8_t* value);
HAL_StatusTypeDef MAX30102_writeReg(uint8_t reg, uint8_t value);

//HAL_StatusTypeDef MAX30102_readRegister ( MAX30102 *dev, uint8_t reg, uint8_t *data);
//HAL_StatusTypeDef MAX30102_readRegisters ( MAX30102 *dev, uint8_t reg, uint8_t * data, uint8_t length);
//HAL_StatusTypeDef MAX30102_writeRegister( MAX30102 *dev, uint8_t reg, uint8_t *data);

#endif