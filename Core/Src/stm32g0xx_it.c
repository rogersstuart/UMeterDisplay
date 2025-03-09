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
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  uint16_t in_data = GPIOA->IDR;
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
