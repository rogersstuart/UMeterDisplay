/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
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

uint8_t skip_next = 0;
uint8_t last_command = 0;
uint8_t state_tracker = 0;
uint8_t skip_data = 0;
uint8_t col_val = 0;
uint8_t old_col_val = 0;
uint8_t ovf = 0;
uint8_t col_val_ctr = 0;
uint8_t datalines = 0;

// New variables for better tracking
uint8_t current_page = 0;
uint8_t current_col_low = 0;
uint8_t current_col_high = 0;
uint8_t readback_active = 0;
uint8_t command_state = 0;
uint8_t data_count = 0;
uint8_t column_address = 0;
uint8_t expecting_col_high = 0;
uint8_t expecting_col_low = 0;
uint8_t column_set_complete = 0;
uint8_t readback_page = 0xFF;

// Better readback tracking
uint8_t last_erd_state = 1;
uint8_t erd_transition_count = 0;
uint8_t consecutive_data_bytes = 0;
uint8_t last_was_command = 1;

// Startup protection and synchronization
volatile uint8_t display_init_complete = 0;
volatile uint8_t startup_protection_active = 1;
volatile uint8_t startup_sync_needed = 0;

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
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);  // REMOVED - PA9 no longer generates interrupts
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);    // Only handle PA10 (strobe)
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  uint16_t in_data = GPIOA->IDR;
  uint8_t datacmd = ((in_data >> 11) & 1);
  uint8_t erd = ((in_data >> 9) & 1);     // Still read ERD, just don't interrupt on it
  uint8_t data_byte = in_data & 0xFF;
  
  // Skip if we're told to
  if(skip_next)
  {
      skip_next--;
      return;
  }

  // SIMPLIFIED APPROACH: Just skip when ERD is low
  if(!erd)
  {
      return; // Skip all processing during readback
  }

  // For SSD1309 in horizontal mode, we need to handle column wrapping
  if(datacmd == 0)  // Command mode
  {
      // Check for column address commands that might cause rolling
      if((data_byte & 0xF0) == 0x00)  // Column low nibble (0x00-0x0F)
      {
          // Check if this is part of a problematic sequence
          if(data_byte < 0x02)  // Columns 0 or 1 might cause rolling
          {
              data_byte = 0x02;  // Force minimum column to 2
          }
      }
      else if((data_byte & 0xF0) == 0x10)  // Column high nibble (0x10-0x1F)
      {
          // Leave high nibble unchanged
      }
      else if((data_byte & 0xF8) == 0xB0)  // Page command
      {
          // Page commands are OK
      }
      else if(data_byte == 0x21)  // Column address range command
      {
          // This command sets column range - might be causing issues
          // Skip this command and its parameters
          skip_next = 2;  // Skip next 2 bytes (start and end column)
          return;
      }
      else if(data_byte == 0x22)  // Page address range command
      {
          // Skip this command and its parameters
          skip_next = 2;  // Skip next 2 bytes (start and end page)
          return;
      }
      
      datalines = data_byte;
  }
  else  // Data mode
  {
      datalines = data_byte;
  }

  // Output section - write to display
  if(datacmd)
      GPIOB->ODR |= (1 << LCD_A0);
  else
      GPIOB->ODR &= ~(1 << LCD_A0);

  // Add small delay for setup time
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");

  GPIOB->ODR &= ~(1 << LCD_WRITE_EN);

  // More setup time
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");

  uint16_t odrbak = GPIOB->ODR;
  odrbak &= 0xFF00;
  odrbak |= datalines;
  GPIOB->ODR = odrbak;

  // Hold time before latch
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");

  GPIOB->ODR |= (1 << LCD_WRITE_EN);  // Latch data

  // Hold time after latch
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void sleep_some()
{

	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");

}

/* USER CODE END 1 */