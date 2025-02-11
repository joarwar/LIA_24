/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "max_sensor.h"
#include "string.h"
#include "stm32f1xx_hal.h"
#include "uart.h"
#include <stdint.h> 
#include "lis2dw12_reg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
stmdev_ctx_t dev_ctx;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
static uint8_t whoamI;
UART_HandleTypeDef huart2;
int16_t raw_accel_data[3]; // 0 - X, 1 - Y, 2 - Z
/* USER CODE BEGIN PV */

volatile uint8_t MAX30102_Flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t millis(void)
{
	return HAL_GetTick();
}

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{

    int ret = HAL_I2C_Mem_Write(handle, LIS2DW12_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 100);
    return (ret == HAL_OK) ? 0 : -1;  
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    int ret = HAL_I2C_Mem_Read(handle, LIS2DW12_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
    return (ret == HAL_OK) ? 0 : -1;  
}


void platform_delay(uint32_t millisec)
{
    HAL_Delay(millisec); 
}

int32_t read_accel_data(const stmdev_ctx_t *ctx) {
  int32_t ret;
  uint8_t data[6]; // 2 bytes per axis (X, Y, Z)

  ret = platform_read(ctx, LIS2DW12_OUT_X_L, data, 6);
  if (ret != 0) {
      return ret; // Return error code if read fails
  }

  raw_accel_data[0] = (int16_t)((data[1] << 8) | data[0]); // X-axis
  raw_accel_data[1] = (int16_t)((data[3] << 8) | data[2]); // Y-axis
  raw_accel_data[2] = (int16_t)((data[5] << 8) | data[4]); // Z-axis

  return 0; // Success
}


void print_accel_data(){
  uart_PrintString("Accelerometer Data:\n");
  uart_PrintString("X: ");
  uart_PrintInt(raw_accel_data[0], 10);
  uart_PrintString("\n");
  uart_PrintString("Y: ");
  uart_PrintInt(raw_accel_data[1], 10);
  uart_PrintString("\n");
  uart_PrintString("Z: ");
  uart_PrintInt(raw_accel_data[2], 10);
  uart_PrintString("\n");  
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  //I2C_Init();
  uart_Init();
  FIFO_LED_DATA fifoledData;
  //long currentMillis = 0;
  //long lastMillis = 0;
  MAX30102_resetRegister();
  MAX30102_initFIFO();
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  //dev_ctx.handle = &platform_handle;  // Optional, if a handle is required

  //Sampling & pulse width
  MAX30102_setSampleRate(_100SPS);
  MAX30102_setPulseWidth(_411_US);

  // LED current (mA)
  MAX30102_setLedCurrent(RED_LED, 5); // Saturates pretty easily depending on surrounding lights
  MAX30102_setLedCurrent(IR_LED, 1); // Shouldn't be needed

  MAX30102_resetFIFO();

  //Mode
  MAX30102_setMeasMode(HEART_RATE);
  //currentMillis = millis();
  uint8_t ctrl1 = 0x50; 
  platform_write(&dev_ctx, 0x20, &ctrl1, 1);
  
  uint8_t ctrl2 = 0x00;
  platform_write(&dev_ctx, 0x21, &ctrl2, 1);
  
  uint8_t ctrl3 = 0x40;
  platform_write(&dev_ctx, 0x22, &ctrl3, 1);
  
  uint8_t ctrl6 = 0x00;
  platform_write(&dev_ctx, 0x25, &ctrl6, 1);
  


  // uint8_t who_am_i;
  // int32_t ret = platform_read(&hi2c2, LIS2DW12_WHO_AM_I, &who_am_i, 1);
  // if (ret == 0) {
  //     uart_PrintString("WHO_AM_I value: ");
  //     uart_PrintInt(who_am_i, 10);
  //     uart_PrintString("\n");
  
  //     if (who_am_i == LIS2DW12_ID) {
  //         uart_PrintString("Device ID matches! It's a LIS2DW12.\n");
  //     } else {
  //         uart_PrintString("Device ID mismatch.\n");
  //     }
  // }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t who_am_i;
    int32_t ret = platform_read(&hi2c2, LIS2DW12_WHO_AM_I, &who_am_i, 1);
    if (ret == 0){
      uart_PrintString("WHO_AM_I: ");
      uart_PrintInt(who_am_i, 10);
      uart_PrintString("\n");
      uart_PrintString("LIS2DW12 ID: ");
      uart_PrintInt(LIS2DW12_ID, 10);
      uart_PrintString("\n");
    }
    
    uint8_t status;
    lis2dw12_read_reg(&dev_ctx, 0x27, &status, 1);
    uart_PrintString("STATUS: ");
    uart_PrintInt(status, 10);
    uart_PrintString("\n");

    if (status & 0x01) { 
        read_accel_data(&dev_ctx);
        print_accel_data(); 
    } else {
        uart_PrintString("No new data available\n");
    }
    HAL_Delay(500); 
    if (INTERRUPT == 1)
    {
      if (MAX30102_Flag)
      {
        MAX30102_Flag = 0;
        fifoledData = MAX30102_read_FIFO();

        //max_Sensor = MAX30102_update(fifoledData);

        MAX30102_clearInterrupt();

      }

    }else{
      fifoledData = MAX30102_read_FIFO();
      
      //max_Sensor = MAX30102_update(fifoledData);

      MAX30102_resetFIFO();

      //HAL_Delay(10);
    }

    // currentMillis = millis();
    // if (currentMillis - lastMillis > 1000)
    // {
    MAX30102_displayData();
    //   lastMillis = currentMillis;
    //   MAX30102_registerData();
    // }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
