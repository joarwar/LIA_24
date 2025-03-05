/*
 ******************************************************************************
 * @file    orientation.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to detect 6D orientation from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-MKI179V1
 * - NUCLEO_F401RE + X_NUCLEO_IKS01A3
 * - DISCOVERY_SPC584B + X_NUCLEO_IKS01A3
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(supported)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */

// #define STEVAL_MKI109V3  /* little endian */
// #define NUCLEO_F401RE    /* little endian */
// #define SPC584B_DIS      /* big endian */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#define NUCLEO_F103RB

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lis2dw12_reg.h"
#include "stm32f1xx_hal.h"
#include "uart.h"
#include <stdint.h>
#include <math.h> 

/* Private macro -------------------------------------------------------------*/
#define BOOT_TIME 20 // ms
#define RAD_TO_DEG 57.29577951308232  //radians till degrees
#define MOVING_AVG_SIZE 10
#define PRINT_DELAY_MS 100
/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;
/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint16_t ms);

// Buffer to store the last N accelerometer values
int16_t ax_buffer[MOVING_AVG_SIZE] = {0};
int16_t ay_buffer[MOVING_AVG_SIZE] = {0};
int16_t az_buffer[MOVING_AVG_SIZE] = {0};

//Global variables
float current_roll = 0.0f;
float current_pitch = 0.0f;
// Variable to keep track of the index for storing the next sample
uint8_t sample_index = 0;

// Function to calculate the average of the buffer
float calculate_average(int16_t* buffer) {
    int32_t sum = 0;
    for (int i = 0; i < MOVING_AVG_SIZE; i++) {
        sum += buffer[i];
    }
    return (float)sum / MOVING_AVG_SIZE;
}

// Function to update the buffer with new accelerometer data
void update_buffer(int16_t ax, int16_t ay, int16_t az) {
    // Update the buffer with the new readings
    ax_buffer[sample_index] = ax;
    ay_buffer[sample_index] = ay;
    az_buffer[sample_index] = az;

    // Move the index to the next position in the buffer
    sample_index = (sample_index + 1) % MOVING_AVG_SIZE;
}

/* Main Example --------------------------------------------------------------*/
void lis2dw12_orientation(void)
{
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  lis2dw12_reg_t int_route;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &hi2c2; // Assuming hi2c2 is the I2C handle

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  uint8_t whoamI;
  lis2dw12_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LIS2DW12_ID)
  {
    while (1)
    {
      /* Handle device not found */
    }
  }

  /* Restore default configuration */
  lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);
  uint8_t rst;
  do
  {
    lis2dw12_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Set full scale */
  lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_2g);

  /* Configure power mode */
  lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit);

  /* Set threshold to 60 degrees */
  lis2dw12_6d_threshold_set(&dev_ctx, 0x02);

  /* LPF2 on 6D function selection. */
  lis2dw12_6d_feed_data_set(&dev_ctx, LIS2DW12_LPF2_FEED);

  /* Enable interrupt generation on 6D INT1 pin. */
  lis2dw12_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
  int_route.ctrl4_int1_pad_ctrl.int1_6d = PROPERTY_ENABLE;
  lis2dw12_pin_int1_route_set(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);

  /* Set Output Data Rate */
  lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_200Hz);

  /* Wait Events. */
  while (1)
  {
    lis2dw12_all_sources_t all_source;
    int16_t ax, ay, az;
    float pitch, roll;

    // Read raw accelerometer data for each axis
    lis2dw12_acceleration_raw_get(&dev_ctx, &ax);  // Get X axis
    lis2dw12_acceleration_raw_get(&dev_ctx, &ay);  // Get Y axis
    lis2dw12_acceleration_raw_get(&dev_ctx, &az);  // Get Z axis

    // Update the buffer with new data
    update_buffer(ax, ay, az);

    // Calculate the average values of the last N samples
    float avg_ax = calculate_average(ax_buffer);
    float avg_ay = calculate_average(ay_buffer);
    float avg_az = calculate_average(az_buffer);

    // Convert raw values to g (assuming 2g full scale)
    float x = avg_ax * 2.0f / 32768.0f;
    float y = avg_ay * 2.0f / 32768.0f;
    float z = avg_az * 2.0f / 32768.0f;

    // Calculate pitch and roll
    pitch = atan2f(y, sqrtf(x * x + z * z)) * RAD_TO_DEG;
    roll = atan2f(-x, sqrtf(y * y + z * z)) * RAD_TO_DEG;

    current_pitch = pitch;
    current_roll = roll;

    // Check for NaN or infinite values
    if (isnan(pitch) || isnan(roll) || !isfinite(pitch) || !isfinite(roll))
    {
      continue; // Skip invalid prints
    }

    // Print the calculated pitch and roll, limiting the output to every 500ms
    uart_PrintString("Pitch: ");
    uart_PrintFloat(pitch);
    uart_PrintString("°, Roll: ");
    uart_PrintFloat(roll);
    uart_PrintString("°\r\n");

    // Delay to slow down the print rate
    platform_delay(PRINT_DELAY_MS);
  }
}
/*
   * @brief  Write generic device register (platform dependent)
   *
   * @param  handle    customizable argument. In this examples is used in
   *                   order to select the correct sensor bus handler.
   * @param  reg       register to write
   * @param  bufp      pointer to data to write in register reg
   * @param  len       number of consecutive register to write
   *
   */
  static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                                uint16_t len)
  {
    HAL_I2C_Mem_Write(handle, LIS2DW12_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 1000);

    return 0;
  }

  /*
   * @brief  Read generic device register (platform dependent)
   *
   * @param  handle    customizable argument. In this examples is used in
   *                   order to select the correct sensor bus handler.
   * @param  reg       register to read
   * @param  bufp      pointer to buffer that store the data read
   * @param  len       number of consecutive register to read
   *
   */
  static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                               uint16_t len)
  {
    HAL_I2C_Mem_Read(handle, LIS2DW12_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

    return 0;
  }

  /*
   * @brief  Write generic device register (platform dependent)
   *
   * @param  tx_buffer     buffer to transmit
   * @param  len           number of byte to send
   *
   */

  /*
   * @brief  platform specific delay (platform dependent)
   *
   * @param  ms        delay in ms
   *
   */
  static void platform_delay(uint16_t ms)
  {
    HAL_Delay(ms);
  }
