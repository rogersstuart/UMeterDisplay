#include "lcd_if.h"

void displayInit(){


	//check LCD mode switch
	uint8_t display_option = SSD1309_MODE;

	resetDisplay();

	switch(display_option)
	{
		case SSD1309_MODE:
		case SSD1305_MODE: oledInit(); break;
		case LCD_MODE: lcdInit(); break;
	}

	//common display init commands

	//lcd_write(0,0xAE); //display on
	//lcd_write(0,0xA4); //all pixels on (normal display)

	//if PD1 or PD0
	if((GPIOD->IDR & 0b11) != 0b11)
		displayWrite(0,0xA7);


	displayWrite(0,0xAF); //display on

	//if pd0 low halt
	if((GPIOD->IDR & 1) == 0)
	{
		uint8_t page_addr = 0xB0;
		__disable_irq();

		clearDisplay();

		while(1);
	}
}

void oledInit(){

	displayWrite(0, 0xA1); //oled
}

void lcdInit(){

	//write 0xA2 10100010
	displayWrite(0, 0xA2);

	//xA0
	displayWrite(0, 0xA0);

	//0xC0
	displayWrite(0, 0xC0);

	//0x24
	displayWrite(0, 0x24);

	//0x81
	displayWrite(0, 0x81);

	//0x20
	displayWrite(0, 0x10);

	//0x2C
	displayWrite(0, 0x2C);

	//0x2E
	displayWrite(0, 0x2E);

	//0x2F
	displayWrite(0, 0x2F);

	uint8_t page_addr = 0xB0;

	for(int y = 0; y < 8; y++)
	{
		displayWrite(0,0b10110000+y);
		displayWrite(0,0b00010000);
		displayWrite(0,0b00000000);

		for(int k = 0; k < 131; k++)
		{
			GPIOB->ODR |= ((uint16_t)1 << LCD_A0);
			GPIOB->ODR &= ~((uint16_t)1 << LCD_WRITE_EN);

			uint16_t gpio_data = GPIOB->ODR;
			gpio_data &= ((uint16_t)0xFF << 8);
			gpio_data |= 0x00;
			GPIOB->ODR = gpio_data;

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

			GPIOB->ODR |= ((uint16_t)1 << LCD_WRITE_EN);
		}
	}
}

void resetDisplay()
{
	GPIOB->ODR |= ((uint16_t)1 << LCD_READ_EN);
	GPIOB->ODR &= ~((uint16_t)1 << LCD_WRITE_EN);
	GPIOB->ODR &= ~((uint16_t)1 << LCD_A0);

	//de-select display by setting CS to 1
	GPIOB->ODR |= ((uint16_t)1 << LCD_CS);

	//put display into reset
	GPIOB->ODR &= ~((uint16_t)1 << LCD_RST);

	//wait some
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

	//take display out of reset
	GPIOB->ODR |= ((uint16_t)1 << LCD_RST);

	//wait some
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

	//select display
	GPIOB->ODR &= ~((uint16_t)1 << LCD_CS);

	//wait some
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
}

void clearDisplay(){

	//use page write mode
	for(int y = 0; y < 8; y++)
	{
		displayWrite(0,0b10110000+y);
		displayWrite(0,0b00010000);
		displayWrite(0,0b00000000);

		for(int k = 0; k < 131; k++)
		{
			GPIOB->ODR |= ((uint16_t)1 << LCD_A0);
			GPIOB->ODR &= ~((uint16_t)1 << LCD_WRITE_EN);

			uint16_t gpio_data = GPIOB->ODR;
			gpio_data &= ((uint16_t)0xFF << 8);
			gpio_data |= 0x00;
			GPIOB->ODR = gpio_data;

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

			GPIOB->ODR |= ((uint16_t)1 << LCD_WRITE_EN);
		}
	}
}

void displayWrite(uint8_t cmd_dat, uint8_t data)
{
	//for the 8080 parallel interface RWR should be low when writing to the display

	if(cmd_dat)
		GPIOB->ODR |= ((uint16_t)1 << LCD_A0);
	else
		GPIOB->ODR &= ~((uint16_t)1 << LCD_A0);

	//signals will be latched on the rising edge of wr
	//when rd is low data lines are in output mode

	GPIOB->ODR &= ~((uint16_t)1 << LCD_WRITE_EN);

	uint16_t gpio_data = GPIOB->ODR;
	gpio_data &= ((uint16_t)0xFF << 8);
	gpio_data |= data;
	GPIOB->ODR = gpio_data;


	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");

	GPIOB->ODR |= ((uint16_t)1 << LCD_WRITE_EN);
}
