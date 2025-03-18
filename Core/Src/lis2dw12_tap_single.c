/*
 ******************************************************************************
 * @file    single_tap.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to detect single tap from sensor.
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

//#define STEVAL_MKI109V3  /* little endian */
//#define NUCLEO_F401RE    /* little endian */
//#define SPC584B_DIS      /* big endian */
#define NUCLEO_F103RB

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lis2dw12_reg.h"
#include "stm32f1xx_hal.h"
#include "uart.h"
#include <stdint.h>


/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            20 //ms

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


/* Main Example --------------------------------------------------------------*/
void lis2dw12_single_tap(void)
{
  /* Initialize mems driver interface. */
  stmdev_ctx_t dev_ctx;
  lis2dw12_reg_t int_route;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &hi2c2;
  /* Initialize platform specific hardware */
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lis2dw12_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LIS2DW12_ID)
    while (1) {
      /* manage here device not found */
    }

  /* Restore default configuration */
  lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lis2dw12_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Set full scale */
  lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_2g);
  /* Configure power mode */
  lis2dw12_power_mode_set(&dev_ctx,
                          LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit);
  /* Set Output Data Rate */
  lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_400Hz);
  /* Enable Tap detection on X, Y, Z */
  lis2dw12_tap_detection_on_z_set(&dev_ctx, PROPERTY_ENABLE);
  lis2dw12_tap_detection_on_y_set(&dev_ctx, PROPERTY_ENABLE);
  lis2dw12_tap_detection_on_x_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Tap threshold on all axis */
  lis2dw12_tap_threshold_x_set(&dev_ctx, 9);
  lis2dw12_tap_threshold_y_set(&dev_ctx, 9);
  lis2dw12_tap_threshold_z_set(&dev_ctx, 9);
  /* Configure Single Tap parameter */
  lis2dw12_tap_quiet_set(&dev_ctx, 1);
  lis2dw12_tap_shock_set(&dev_ctx, 2);
  /* Enable Single Tap detection only */
  lis2dw12_tap_mode_set(&dev_ctx, LIS2DW12_ONLY_SINGLE);
  /* Enable single tap detection interrupt */
  lis2dw12_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
  int_route.ctrl4_int1_pad_ctrl.int1_single_tap = PROPERTY_ENABLE;
  lis2dw12_pin_int1_route_set(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);

  /* Wait Events */
  while (1) {
    lis2dw12_all_sources_t all_source;
    /* Check Single Tap events */
    lis2dw12_all_sources_get(&dev_ctx, &all_source);

    if (all_source.tap_src.single_tap) {
      uart_PrintString("Tap detected on: "); 
      uart_PrintInt(all_source.tap_src.tap_sign, 10);
      uart_PrintString("\r\n");

      if (all_source.tap_src.x_tap) {
        uart_PrintString("x");
        //uart_PrintInt(all_source.tap_src.x_tap, 10);
        uart_PrintString("\r\n");
      }

      if (all_source.tap_src.y_tap) {
        uart_PrintString("y");
        //uart_PrintInt(all_source.tap_src.y_tap, 10);
        uart_PrintString("\r\n");
      }

      if (all_source.tap_src.z_tap) {
        uart_PrintString("z");
        //uart_PrintInt(all_source.tap_src.z_tap, 10);
        uart_PrintString("\r\n");
      }

      //tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
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
I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);

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

