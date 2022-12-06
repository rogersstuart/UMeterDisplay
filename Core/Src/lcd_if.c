#include "lcd_if.h"

void lcd_init(){

	GPIOB->ODR |= ((uint16_t)1 << LCD_READ_EN);
	GPIOB->ODR &= ~((uint16_t)1 << LCD_WRITE_EN);
	GPIOB->ODR &= ~((uint16_t)1 << LCD_A0);

	//lcd_cs_low
	//lcd_rst high
	GPIOB->ODR |= ((uint16_t)1 << LCD_CS);

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

	GPIOB->ODR |= ((uint16_t)1 << LCD_RST);
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

	GPIOB->ODR &= ~((uint16_t)1 << LCD_RST);
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

	GPIOB->ODR |= ((uint16_t)1 << LCD_RST);
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

	GPIOB->ODR &= ~((uint16_t)1 << LCD_CS);
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

	//write 0xA2 10100010
	lcd_write(0, 0xA2);

	//xA0
	lcd_write(0, 0xA0);

	//0xC0
	lcd_write(0, 0xC0);

	//0x24
	lcd_write(0, 0x24);

	//0x81
	lcd_write(0, 0x81);

	//0x20
	lcd_write(0, 0x10);

	//0x2C
	lcd_write(0, 0x2C);

	//0x2E
	lcd_write(0, 0x2E);

	//0x2F
	lcd_write(0, 0x2F);

	uint8_t page_addr = 0xD0;

	for(int y = 0; y < 8; y++)
	{
		lcd_write(0,0b10110000+y);
		lcd_write(0,0b00010000);
		lcd_write(0,0b00000000);
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

		lcd_write(0,0xA4); //all pixels on (normal display)
		lcd_write(0,0xAF); //display on
	}
}

void lcd_write(uint8_t cmd_dat, uint8_t data)
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
