/*
 *
 * Max30102 I2C Driver
 *
 * Author: Joar Warholm
 * Created: 13 November 2024
 *
 */

#ifndef MAX30102_I2C_DRIVER_H
#define MAX30102_I2C_DRIVER_H

#include "stm32f1xx_hal.h" /*Needed for I2C*/
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* Pulse detection parameters */
#define PULSE_MIN_THRESHOLD         300 // 300 for finger and around 20 for wrist
#define PULSE_MAX_THRESHOLD         2000
#define PULSE_GO_DOWN_THRESHOLD     1

/*
 * DEFINES
 */

#define MAX30102_I2C_ADDR_READ 0xAE
#define MAX30102_I2C_ADDR_WRITE 0xAF
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

#define MAX30102_LED1_PULsE 0x0C
#define MAX30102_LED2_PULSE 0x0D

#define MAX30102_MULTI_LED_CTRL_1 0x11
#define MAX30102_MULTI_LED_CTRL_SLOT2 4
#define MAX30102_MULTI_LED_CTRL_SLOT1 0
#define MAX30102_MULTI_LED_CTRL_2 0x12
#define MAX30102_MULTI_LED_CTRL_SLOT4 4
#define MAX30102_MULTI_LED_CTRL_SLOT3 0

//DIE TEMP
#define MAX30102_DIE_TINT 0x1f
#define MAX30102_DIE_TFRAC 0x20
#define MAX30102_DIE_TFRAC_INCREMENT 0.0625f
#define MAX30102_DIE_TEMP_CONFIG 0x21
#define MAX30102_DIE_TEMP_EN 1

/*
 * SENSOR STRUCT
 */

typedef struct {
    /* I2C Handle*/
    I2C_HandleTypeDef *i2cHandle;
    
    /* IR and red sample data*/
    uint32_t _ir_samples[32];
    uint32_t _red_samples[32];

    /* Interrupt flag*/
    uint8_t _interrupt_flag;

} MAX30102;

/*
 * INITIALISATION
 */
uint8_t MAX30102_Initialise ( MAX30102 * dev, I2C_HandleTypeDef *i2cHandle);

/*
 * DATA ACQUISITION
 */

HAL_StatusTypeDef MAX30102_ReadIR ( MAX30102 *dev);
HAL_StatusTypeDef MAX30102_ReadRED ( MAX30102 * dev);

/*
 * LOW-LEVEL FUNCTIONS
 */
HAL_StatusTypeDef MAX30102_ReadRegister ( MAX30102 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef MAX30102_ReadRegisters ( MAX30102 *dev, uint8_t reg, uint8_t * data, uint8_t length);

HAL_StatusTypeDef MAX30102_WriteRegister( MAX30102 *dev, uint8_t reg, uint8_t *data);

#endif