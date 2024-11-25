#include "MAX30102_barebone_test.h"
#include "stm32f1xx_hal.h"

I2C_HandleTypeDef hi2c1;

// MAX30102 Registers
#define MAX30102_I2C_ADDR 0xAE

// Register Addresses
#define MAX30102_MODE_CONFIG 0x06
#define MAX30102_FIFO_DATA_REGISTER 0x07
#define MAX30102_INTERRUPT_STATUS_1 0x00
#define MAX30102_FIFO_READ_POINTER 0x04
#define MAX30102_FIFO_WRITE_POINTER 0x05

// Measurement Mode
#define HEART_RATE_MODE 0x02  // Mode for heart rate

// Heart rate variables
float currentBPM = 0.0f;
uint8_t fifoData[6];  // FIFO data buffer (6 bytes for 2 LED channels)

// I2C Init function
void I2C_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        // Error handling
    }
}

// Write to MAX30102 register
HAL_StatusTypeDef MAX30102_writeReg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(&hi2c1, MAX30102_I2C_ADDR, data, 2, HAL_MAX_DELAY);
}

// Read from MAX30102 register
HAL_StatusTypeDef MAX30102_readReg(uint8_t reg, uint8_t* value) {
    return HAL_I2C_Master_TransmitReceive(&hi2c1, MAX30102_I2C_ADDR, &reg, 1, value, 1, HAL_MAX_DELAY);
}

// Set MAX30102 to heart rate measurement mode
void MAX30102_setHeartRateMode(void) {
    uint8_t modeConfig = HEART_RATE_MODE;
    MAX30102_writeReg(MAX30102_MODE_CONFIG, modeConfig);
}

// Read the FIFO data (IR and Red LED data)
HAL_StatusTypeDef MAX30102_readFIFO(void) {
    uint8_t readPointer;
    uint8_t fifoDataReg = MAX30102_FIFO_DATA_REGISTER;
    HAL_StatusTypeDef status;

    // Get current read pointer
    status = MAX30102_readReg(MAX30102_FIFO_READ_POINTER, &readPointer);
    if (status != HAL_OK) {
        return HAL_ERROR;
    }

    // Read 6 bytes from FIFO data register
    status = HAL_I2C_Master_TransmitReceive(&hi2c1, MAX30102_I2C_ADDR, &fifoDataReg, 1, fifoData, 6, HAL_MAX_DELAY);
    return status;
}

// Read the heart rate (BPM) from FIFO data (simplified)
void MAX30102_readHeartRate(void) {
    // Read FIFO data (6 bytes: 3 for red LED and 3 for IR LED)
    if (MAX30102_readFIFO() == HAL_OK) {
        uint16_t irData = (fifoData[0] << 8) | fifoData[1];  // Combine two bytes for IR data
        uint16_t redData = (fifoData[2] << 8) | fifoData[3];  // Combine two bytes for Red data
        
        // Calculate BPM (simplified approach, usually would require pulse detection algorithms)
        // This is a placeholder calculation, in reality, you'd need algorithms to detect pulses
        currentBPM = (float)(irData + redData) / 2.0f;  // A simple average for demonstration
    }
}

// Main loop
int main(void) {
    HAL_Init();
    I2C_Init();

    // Set MAX30102 to Heart Rate Mode
    MAX30102_setHeartRateMode();

    while (1) {
        // Read and calculate heart rate
        MAX30102_readHeartRate();
        
        // Display the heart rate
        printf("Current Heart Rate: %.2f BPM\n", currentBPM);

        HAL_Delay(1000);  // Delay for 1 second
    }
}
