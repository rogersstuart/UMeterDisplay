#include "lcd_if.h"

void displayInit(){

	resetDisplay();

	switch(DISPLAY_TYPE)
	{
		case SSD1309:
		case SSD1305: oledInit(); break;
		case LCD: lcdInit(); break;
	}

	//common display init commands

	//lcd_write(0,0xAE); //display on
	//lcd_write(0,0xA4); //all pixels on (normal display)

	//if PD1 or PD0
	if((GPIOD->IDR & 0b11) != 0b11)
		displayWrite(0,0xA7);


	displayWrite(0,0xAF); //display on

	// Mark initialization as complete
	extern volatile uint8_t display_init_complete;
	display_init_complete = 1;

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

	clearDisplay();
	
	// Proper SSD1309 initialization sequence
	displayWrite(0, 0xAE); // Display off
	displayWrite(0, 0xD5); // Set display clock divide ratio
	displayWrite(0, 0x80); // Default value
	displayWrite(0, 0xA8); // Set multiplex ratio
	displayWrite(0, 0x3F); // 64 MUX
	displayWrite(0, 0xD3); // Set display offset
	displayWrite(0, 0x00); // No offset
	displayWrite(0, 0x40); // Set start line
	displayWrite(0, 0x8D); // Charge pump
	displayWrite(0, 0x14); // Enable charge pump
	displayWrite(0, 0xA1); // Segment remap (reverse direction)
	displayWrite(0, 0xC0); // Normal COM scan direction
	displayWrite(0, 0xDA); // COM pins configuration
	displayWrite(0, 0x12); // Alternative COM pin configuration
	displayWrite(0, 0x81); // Set contrast
	displayWrite(0, 0xCF); // High contrast
	displayWrite(0, 0xD9); // Set pre-charge period
	displayWrite(0, 0xF1); // Pre-charge: 15 clocks, discharge: 1 clock
	displayWrite(0, 0xDB); // VCOMH deselect level
	displayWrite(0, 0x40); // VCOMH level
	displayWrite(0, 0xA4); // Resume display with RAM content
	displayWrite(0, 0xA6); // Normal display (not inverted)
	
	// CRITICAL: Set horizontal addressing mode explicitly
	displayWrite(0, 0x20); // Memory addressing mode command
	displayWrite(0, 0x00); // Horizontal addressing mode
	
	// Define the column address range to account for the offset
	displayWrite(0, 0x21); // Column address range command
	displayWrite(0, 0x02); // Start from column 2 (to center 128 pixels in 132 columns)
	displayWrite(0, 0x81); // End at column 129 (2+127)
	
	// Define the page address range
	displayWrite(0, 0x22); // Page address range command  
	displayWrite(0, 0x00); // Start page 0
	displayWrite(0, 0x07); // End page 7
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
	displayWrite(0, 0x20);

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
