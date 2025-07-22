/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
  * 
  * @details This file handles all interrupt routines for the STM32G0xx microcontroller,
  *			 with special focus on LCD communication handling via external interrupts.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g0xx_it.h"
#include "lcd_if.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// Command processing control flag
uint8_t skip_next = 0;           // Number of bytes to skip processing

// Data handling
uint8_t datalines = 0;           // Data lines value for output

// Initialization and synchronization
volatile uint8_t display_init_complete = 0;      // Flag indicating display initialization complete

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  * 
  * This handler processes LCD display communication by:
  * 1. Capturing data on GPIO pins when strobe occurs
  * 2. Filtering out initialization/configuration commands
  * 3. Processing addressing commands and data bytes
  * 4. Sending allowed commands and data to the LCD
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);  // REMOVED - PA9 no longer generates interrupts
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);    // Only handle PA10 (strobe)
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  // Read current GPIO state
  uint16_t in_data = GPIOA->IDR;
  uint8_t datacmd = ((in_data >> 11) & 1);  // 0 = command, 1 = data
  uint8_t erd = ((in_data >> 9) & 1);       // Enable Read signal (active low)
  uint8_t data_byte = in_data & 0xFF;       // Data byte on PA0-PA7
  
  // Skip processing if in skip mode (used for multi-byte commands)
  if(skip_next)
  {
      skip_next--;
      return;
  }

  // Skip all processing during readback operations (when ERD is low)
  if(!erd)
  {
      return;
  }

  // Process incoming data based on mode (command vs data)
  if(datacmd == 0)  // Command mode
  {
      // Filter commands - only allow addressing commands through
      // Block all initialization/configuration commands
      
      // Column address low nibble (0x00-0x0F)
      if((data_byte & 0xF0) == 0x00)
      {
          // Prevent columns 0-1 which might cause display rolling issues
          if(data_byte < 0x02)
          {
              data_byte = 0x02;  // Force minimum column to 2
          }
          datalines = data_byte;
      }
      // Column address high nibble (0x10-0x1F)
      else if((data_byte & 0xF0) == 0x10)
      {
          datalines = data_byte;
      }
      // Page address command (0xB0-0xB7)
      else if((data_byte & 0xF8) == 0xB0)
      {
          datalines = data_byte;
      }
      // Memory addressing mode - block and skip parameter
      else if(data_byte == 0x20)
      {
          skip_next = 1;
          return;
      }
      // Column address range - block and skip both parameters
      else if(data_byte == 0x21)
      {
          skip_next = 2;
          return;
      }
      // Page address range - block and skip both parameters
      else if(data_byte == 0x22)
      {
          skip_next = 2;
          return;
      }
      // Display start line (0x40-0x7F) - block
      else if((data_byte & 0xC0) == 0x40)
      {
          return;
      }
      // Contrast control - block and skip parameter
      else if(data_byte == 0x81)
      {
          skip_next = 1;
          return;
      }
      // Charge pump setting - block and skip parameter
      else if(data_byte == 0x8D)
      {
          skip_next = 1;
          return;
      }
      // Segment remap commands - block
      else if(data_byte == 0xA0 || data_byte == 0xA1)
      {
          return;
      }
      // Display all on/resume - block
      else if(data_byte == 0xA4 || data_byte == 0xA5)
      {
          return;
      }
      // Normal/inverse display mode - block
      else if(data_byte == 0xA6 || data_byte == 0xA7)
      {
          return;
      }
      // Multiplex ratio - block and skip parameter
      else if(data_byte == 0xA8)
      {
          skip_next = 1;
          return;
      }
      // Display on/off commands - block
      else if(data_byte == 0xAE || data_byte == 0xAF)
      {
          return;
      }
      // COM scan direction - block
      else if(data_byte == 0xC0 || data_byte == 0xC8)
      {
          return;
      }
      // Display offset - block and skip parameter
      else if(data_byte == 0xD3)
      {
          skip_next = 1;
          return;
      }
      // Display clock divide - block and skip parameter
      else if(data_byte == 0xD5)
      {
          skip_next = 1;
          return;
      }
      // Pre-charge period - block and skip parameter
      else if(data_byte == 0xD9)
      {
          skip_next = 1;
          return;
      }
      // COM pins configuration - block and skip parameter
      else if(data_byte == 0xDA)
      {
          skip_next = 1;
          return;
      }
      // VCOMH deselect level - block and skip parameter
      else if(data_byte == 0xDB)
      {
          skip_next = 1;
          return;
      }
      // NOP command - block
      else if(data_byte == 0xE3)
      {
          return;
      }
      // Unknown/unsupported command - block for safety
      else
      {
          return;
      }
  }
  else  // Data mode - pass through all data bytes
  {
      datalines = data_byte;
  }

  // Send allowed data/commands to display
  
  // Set A0 line based on data/command mode
  if(datacmd)
      GPIOB->ODR |= (1 << LCD_A0);     // Data mode
  else
      GPIOB->ODR &= ~(1 << LCD_A0);    // Command mode

  // Setup time delay
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");

  // Begin write cycle (active low)
  GPIOB->ODR &= ~(1 << LCD_WRITE_EN);

  // Additional setup time
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");

  // Set data lines (PB0-PB7)
  uint16_t odrbak = GPIOB->ODR;
  odrbak &= 0xFF00;                    // Clear data bits
  odrbak |= datalines;                 // Set new data bits
  GPIOB->ODR = odrbak;

  // Hold time before latch
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");

  // End write cycle (latch data)
  GPIOB->ODR |= (1 << LCD_WRITE_EN);

  // Hold time after latch
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
 * @brief Small delay function using NOP instructions
 * 
 * Provides a short, deterministic delay for timing-critical operations
 */
void sleep_some(void)
{
    asm("NOP"); asm("NOP");
    asm("NOP"); asm("NOP");
}

/* USER CODE END 1 */