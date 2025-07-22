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

<<<<<<< Updated upstream
uint8_t skip_next = 0;
uint8_t last_command = 0;
uint8_t state_tracker = 0;
uint8_t skip_data = 0;
uint8_t col_val = 0;
uint8_t old_col_val = 0;
uint8_t ovf = 0;
uint8_t col_val_ctr = 0;
uint8_t datalines = 0;
=======
// Command processing control flag
uint8_t skip_next = 0;           // Number of bytes to skip processing

// Data handling
uint8_t datalines = 0;           // Data lines value for output
uint8_t col_val = 0;             // Current column value
uint8_t old_col_val = 0;         // Previous column value for tracking
uint8_t ovf = 0;                 // Overflow flag for column adjustment
uint8_t col_val_ctr = 0;         // Column value counter

// State tracking for command sequences
uint8_t last_command = 0;        // Last command received
uint8_t state_tracker = 0;       // State machine for command sequence detection
uint8_t skip_data = 0;           // Flag to skip data writes after certain commands

// Initialization and synchronization
volatile uint8_t display_init_complete = 0;      // Flag indicating display initialization complete
>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
<<<<<<< Updated upstream
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
=======
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);   // Handle PA9 (ERD) interrupts
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);  // Handle PA10 (strobe)
>>>>>>> Stashed changes
=======
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);   // Handle PA9 (ERD) interrupts
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);  // Handle PA10 (strobe)
>>>>>>> Stashed changes
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  uint16_t in_data = GPIOA->IDR;
<<<<<<< Updated upstream
  uint8_t datacmd = ((in_data >> 11) & 1);
  uint8_t erd = ((in_data >> 9) & 1);
  datalines = in_data;

  if(!erd)
  	    {
  	  	  GPIOB->ODR &= ~(1<<13);
  	  	  //while(1);
  	    }

  if(skip_next)
  {
	  skip_next = 0;
	  return;
  }

  if(datacmd == 0)
  {
	  if(skip_data)
	  {
		  //skip data was enabled. this means that the low power readback command occurred.
		  //set the inputs to their normal state
		  //set erd to normal

		  GPIOA->PUPDR &= ~0x0000FFFF;
		  GPIOB->ODR |= (1<<10);


		  state_tracker = 0;
		  skip_data = 0;
	  }


	  //if((uint8_t)in_data == 0xD9 || (uint8_t)in_data == 0xDB || (uint8_t)in_data == 0xD5 || (uint8_t)in_data == 0xDC || (uint8_t)in_data == 0xDA || (uint8_t)in_data == 0xD3 || (uint8_t)in_data == 0xA8 || (uint8_t)in_data == 0x81 || (uint8_t)in_data == 0x20)
	  //{
		//  skip_next = 1;
		//  return;
	  //}
	  //else
		  if(((uint8_t)in_data & 0b11111000) == 0b10110000) //page
		  {
			  uint16_t temp = in_data;
		  }
		  else
			  if(((uint8_t)in_data & 0b11110000) == 0b00000000) //lower column
			  {


				  col_val = datalines; //take the lower nibble
				  col_val &= 0xF;

				  old_col_val = col_val;

				  //ssd1309
				  if(DISPLAY_TYPE == SSD1309){
					  if(col_val > 1)
						  ovf = 0;
					  else
						  ovf = 1;

					  col_val -= 2;
				  }
				  //

				  col_val &= 0xF; //trim if it overflowed

				  datalines = col_val; //set it

			  }

			  else
				  if(((uint8_t)in_data & 0b11110000) == 0b00010000) //upper column (on this system it always comes second)
				  {

					  uint8_t local_col_val = datalines & 0xF; //take the lower nibble
					  col_val &= 0xF; //clear upper

					  old_col_val |= local_col_val << 4;

					  //ssd1309
					  if(DISPLAY_TYPE == SSD1309){
						  if(ovf == 1 && local_col_val > 0)
						  {
							  local_col_val -= 1;
							  ovf = 0;
						  }
					  }

						  col_val_ctr = 0;

					  col_val |= (local_col_val << 4);

					  datalines = 0b00010000 | local_col_val;

				  }

				  else
					  if(((uint8_t)in_data & 0b11000000) == 0b01000000)
					  {

					  }
					  else
					  {
						  uint8_t temp = in_data;

					  }
  }
  else
  {
	  if(skip_data)
	  {





		  //if RD# is low and DC is high and skip data is active, write data to meter for read back.
		  //if(!erd)
		  //{
			//  GPIOB->ODR &= ~(1<<10);

		  //}

		  return;
	  }

	  if(old_col_val++ < 2)
	  {
		datacmd = 0;
		datalines = 0;

		if(datacmd)
			GPIOB->ODR |= 1<<12;
		else
			GPIOB->ODR &= ~(1<<12);

		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");

		GPIOB->ODR &= ~(1<<11);


		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");

		uint16_t odrbak = GPIOB->ODR;
		odrbak &= 0xFF00;
		odrbak |= datalines;
		GPIOB->ODR = odrbak;


		GPIOB->ODR |= 1<<11; //wr latch


		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");

		return;
	  }
  }

	if(datacmd)
		GPIOB->ODR |= 0x1000;
	else
		GPIOB->ODR &= 0b1110111111111111;

	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");

	GPIOB->ODR &= 0b1111011111111111;


	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");

	uint16_t odrbak = GPIOB->ODR;
	odrbak &= 0xFF00;
	odrbak |= datalines;
	GPIOB->ODR = odrbak;


	GPIOB->ODR |= 0x800; //wr latch


	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");



	//watch for command sequence B0 01 1F and skip data until next command

	if(datacmd == 0)
	{
		if(last_command == 0xB0)
		{
			state_tracker = 1;
		}
		else
			if(last_command == 0x1 && ((in_data & 0xFF) == 0x1f) && state_tracker == 1)
			{


				skip_data = 1;
				state_tracker = 0;
			}
			else
				state_tracker = 0;

		last_command = in_data;
	}


=======
  uint8_t datacmd = ((in_data >> 11) & 1);  // 0 = command, 1 = data
  uint8_t erd = ((in_data >> 9) & 1);       // Enable Read signal (active low)
  datalines = in_data;                       // Capture full data value first
  
  // Handle readback operation - clear control signal when ERD is low
  if(!erd)
  {
      GPIOB->ODR &= ~(1 << 13);  // Clear PB13 during readback
      return;  // Skip all other processing during readback
  }
  
  // Skip processing if in skip mode (used for multi-byte commands)
  if(skip_next)
  {
      skip_next = 0;  // Match reference behavior (bug or intentional?)
      return;
  }

  // Process incoming data based on mode (command vs data)
  if(datacmd == 0)  // Command mode
  {
      // Reset skip_data flag if active
      if(skip_data)
      {
          GPIOA->PUPDR &= ~0x0000FFFF;  // Clear pull-up/pull-down on PA0-PA7
          GPIOB->ODR |= (1 << 10);      // Set PB10 high
          state_tracker = 0;
          skip_data = 0;
      }
      
      // Filter commands - minimal approach, block everything except addressing
      
      // Page address command (0xB0-0xB7) - REQUIRED for positioning
      if(((uint8_t)in_data & 0b11111000) == 0b10110000)
      {
          // Pass through as-is
      }
      // Column address low nibble (0x00-0x0F) - REQUIRED for positioning
      else if(((uint8_t)in_data & 0b11110000) == 0b00000000)
      {
          col_val = datalines;
          col_val &= 0xF;
          old_col_val = col_val;
          
          if(DISPLAY_TYPE == SSD1309)
          {
              if(col_val > 0)  // Changed from > 1
                  ovf = 0;
              else
                  ovf = 1;
              col_val -= 1;  // Changed from -= 2 to -= 1
          }
          
          col_val &= 0xF;
          datalines = col_val;
      }
      // Column address high nibble (0x10-0x1F) - REQUIRED for positioning
      else if(((uint8_t)in_data & 0b11110000) == 0b00010000)
      {
          uint8_t local_col_val = datalines & 0xF;
          col_val &= 0xF;
          old_col_val |= local_col_val << 4;
          
          if(DISPLAY_TYPE == SSD1309)
          {
              if(ovf == 1 && local_col_val > 0)
              {
                  local_col_val -= 1;
                  ovf = 0;
              }
          }
          
          col_val_ctr = 0;
          col_val |= (local_col_val << 4);
          datalines = 0b00010000 | local_col_val;
      }
      // BLOCK EVERYTHING ELSE
      else
      {
          // Check for multi-byte commands that need parameter skipping
          if(datalines == 0x20)  // Memory addressing mode
          {
              skip_next = 1;
          }
          else if(datalines == 0x21 || datalines == 0x22)  // Column/page range
          {
              skip_next = 2;
          }
          else if(datalines == 0x81)  // Contrast
          {
              skip_next = 1;
          }
          else if(datalines == 0x8D)  // Charge pump
          {
              skip_next = 1;
          }
          else if(datalines == 0xA8)  // Multiplex ratio
          {
              skip_next = 1;
          }
          else if(datalines == 0xD3)  // Display offset
          {
              skip_next = 1;
          }
          else if(datalines == 0xD5)  // Clock divide
          {
              skip_next = 1;
          }
          else if(datalines == 0xD9)  // Pre-charge
          {
              skip_next = 1;
          }
          else if(datalines == 0xDA)  // COM pins
          {
              skip_next = 1;
          }
          else if(datalines == 0xDB)  // VCOMH
          {
              skip_next = 1;
          }
          
          // Block the command itself
          return;
      }
  }
  else  // Data mode
  {
      // Skip data if flag is set
      if(skip_data)
      {
          return;
      }
      
      // Special handling for first two columns
      if(old_col_val++ < 1)  // Changed from < 2 to < 1
      {
          datacmd = 0;  // Force command mode
          datalines = 0;  // Send 0x00
          
          // Write the blank data
          if(datacmd)
              GPIOB->ODR |= 1<<12;
          else
              GPIOB->ODR &= ~(1<<12);
              
          asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
          asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
          
          GPIOB->ODR &= ~(1<<11);
          
          asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
          asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
          
          uint16_t odrbak = GPIOB->ODR;
          odrbak &= 0xFF00;
          odrbak |= datalines;
          GPIOB->ODR = odrbak;
          
          GPIOB->ODR |= 1<<11;
          
          asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
          asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
          
          return;
      }
  }

  // Send allowed data/commands to display
  
  // Set A0 line based on data/command mode
  if(datacmd)
      GPIOB->ODR |= 0x1000;
  else
      GPIOB->ODR &= 0b1110111111111111;

  // Setup time delay
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");

  // Begin write cycle (active low)
  GPIOB->ODR &= 0b1111011111111111;

  // Additional setup time
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");

  // Set data lines (PB0-PB7)
  uint16_t odrbak = GPIOB->ODR;
  odrbak &= 0xFF00;
  odrbak |= datalines;
  GPIOB->ODR = odrbak;

  // End write cycle (latch data)
  GPIOB->ODR |= 0x800;

  // Hold time after latch
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");

  // Track command sequences at the end
  if(datacmd == 0)
  {
      if(last_command == 0xB0)
      {
          state_tracker = 1;
      }
      else if(last_command == 0x1 && ((in_data & 0xFF) == 0x1f) && state_tracker == 1)
      {
          skip_data = 1;
          state_tracker = 0;
      }
      else
      {
          state_tracker = 0;
      }
      last_command = in_data;
  }
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes

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
