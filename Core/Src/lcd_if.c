/**
 * @file    lcd_if.c
 * @brief   LCD/OLED Display Interface Functions
 * @details Provides hardware interface and initialization for SSD1305/1309 OLED
 *          and compatible LCD displays using parallel 8080-series interface.
 * @date    July 21, 2025
 * @author  Stuart
 */

#include "lcd_if.h"

/**
 * @brief   Small delay function using NOPs
 * @details Used for timing-critical display operations
 */
static inline void delayNOP(void) {
    // ~1μs delay at 64MHz (16 NOPs)
    asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
    asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
    asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
    asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
}

/**
 * @brief   Main display initialization function
 * @details Initializes the display based on configured type and applies
 *          common settings. Checks configuration jumpers for special modes.
 */
void displayInit(void) {
    // Reset the display hardware
    resetDisplay();

    // Initialize based on the configured display type
    switch(DISPLAY_TYPE) {
        case SSD1309:
        case SSD1305: 
            oledInit(); 
            break;
        case LCD: 
            lcdInit(); 
            break;
    }

    // Common display initialization commands
    // Note: These commands are commented out as they're already included in type-specific init
    // displayWrite(0,0xAE); // Display off
    // displayWrite(0,0xA4); // Normal display (RAM content)

    // Check jumper settings on PD0/PD1 for display inversion mode
    if((GPIOD->IDR & 0b11) != 0b11)
        displayWrite(0, 0xA7); // Inverted display mode

    // Turn display on
    displayWrite(0, 0xAF); // Display on

    // Mark initialization as complete for interrupt handler synchronization
    extern volatile uint8_t display_init_complete;
    display_init_complete = 1;

    // Special test mode: if PD0 is grounded, clear display and halt
    if((GPIOD->IDR & 1) == 0) {
        uint8_t page_addr = 0xB0; // Unused variable preserved for compatibility
        __disable_irq();
        clearDisplay();
        while(1); // Infinite loop - system halt for testing
    }
}

/**
 * @brief   OLED-specific initialization sequence
 * @details Sets up SSD1305/SSD1309 OLED controllers with proper settings
 *          for the hardware configuration.
 */
void oledInit(void) {
    clearDisplay();
    
    // Standard SSD1305/1309 initialization sequence
    displayWrite(0, 0xAE); // Display off during initialization
    displayWrite(0, 0xD5); // Set display clock divide ratio
    displayWrite(0, 0x80); // Default ratio (recommended value)
    displayWrite(0, 0xA8); // Set multiplex ratio (screen height)
    displayWrite(0, 0x3F); // 64 MUX for 128x64 display
    displayWrite(0, 0xD3); // Set display offset
    displayWrite(0, 0x00); // No offset (start at line 0)
    displayWrite(0, 0x40); // Set display start line to 0
    displayWrite(0, 0x8D); // Charge pump setting
    displayWrite(0, 0x14); // Enable internal charge pump for OLED
    displayWrite(0, 0xA1); // Segment remap (column address 127 mapped to SEG0)
    displayWrite(0, 0xC0); // COM scan direction (normal direction)
    displayWrite(0, 0xDA); // COM pins hardware configuration
    displayWrite(0, 0x12); // Alternative COM pin configuration (for 128x64)
    displayWrite(0, 0x81); // Set contrast control
    displayWrite(0, 0xCF); // High contrast value
    displayWrite(0, 0xD9); // Set pre-charge period
    displayWrite(0, 0xF1); // Pre-charge: 15 clocks, discharge: 1 clock
    displayWrite(0, 0xDB); // Set VCOMH deselect level
    displayWrite(0, 0x40); // VCOMH voltage level
    displayWrite(0, 0xA4); // Resume display with RAM content
    displayWrite(0, 0xA6); // Normal display mode (not inverted)
    
    // CRITICAL: Set horizontal addressing mode explicitly
    displayWrite(0, 0x20); // Memory addressing mode command
    displayWrite(0, 0x00); // Horizontal addressing mode (auto-increment column)
    
    // Define the column address range to account for the offset
    // Note: SSD1309 has 132 columns but we use 128 centered by starting at column 2
    displayWrite(0, 0x21); // Column address range command
    displayWrite(0, 0x02); // Start from column 2 (to center 128 pixels in 132 columns)
    displayWrite(0, 0x81); // End at column 129 (2+127)
    
    // Define the page address range (8 pages for 64-pixel height)
    displayWrite(0, 0x22); // Page address range command  
    displayWrite(0, 0x00); // Start page 0
    displayWrite(0, 0x07); // End page 7 (8 pages total, 0-7)
}

/**
 * @brief   LCD-specific initialization sequence
 * @details Sets up standard LCD controller (likely ST7565 or similar)
 */
void lcdInit(void) {
    // Standard LCD initialization sequence (likely ST7565 or compatible)
    displayWrite(0, 0xA2); // LCD bias setting (1/9)
    displayWrite(0, 0xA0); // ADC select (normal)
    displayWrite(0, 0xC0); // Common output mode select (normal)
    displayWrite(0, 0x24); // Voltage regulator internal resistor ratio
    displayWrite(0, 0x81); // Electronic volume mode set
    displayWrite(0, 0x20); // Electronic volume value (contrast)
    displayWrite(0, 0x2C); // Power control setting - VC on
    displayWrite(0, 0x2E); // Power control setting - VR on
    displayWrite(0, 0x2F); // Power control setting - VF on (all on)

    // Clear the display - page by page, column by column
    uint8_t page_addr = 0xB0;

    for(int y = 0; y < 8; y++) {
        // Set page address (0xB0-0xB7)
        displayWrite(0, 0b10110000 + y);
        // Set column address high nibble (0x1)
        displayWrite(0, 0b00010000);
        // Set column address low nibble (0x0)
        displayWrite(0, 0b00000000);

        // Fill page with zeros
        for(int k = 0; k < 131; k++) {
            // Set data mode (A0 high)
            GPIOB->ODR |= ((uint16_t)1 << LCD_A0);
            // Begin write cycle (WR low)
            GPIOB->ODR &= ~((uint16_t)1 << LCD_WRITE_EN);

            // Set data lines to zero (clear)
            uint16_t gpio_data = GPIOB->ODR;
            gpio_data &= ((uint16_t)0xFF << 8);  // Keep high byte unchanged
            gpio_data |= 0x00;                   // Set low byte (data) to 0
            GPIOB->ODR = gpio_data;

            // Setup time delay
            delayNOP();

            // End write cycle (latch data)
            GPIOB->ODR |= ((uint16_t)1 << LCD_WRITE_EN);

            // Hold time after latch
            delayNOP();
        }
    }
}

/**
 * @brief   Reset the display hardware
 * @details Performs hardware reset sequence by toggling RST pin
 *          and configuring the control signals appropriately
 */
void resetDisplay(void) {
    // Initialize control signals to safe states
    GPIOB->ODR |= ((uint16_t)1 << LCD_READ_EN);  // Read disabled (high)
    GPIOB->ODR &= ~((uint16_t)1 << LCD_WRITE_EN); // Write disabled for now (low)
    GPIOB->ODR &= ~((uint16_t)1 << LCD_A0);      // Command mode (low)

    // Deselect display by setting CS high
    GPIOB->ODR |= ((uint16_t)1 << LCD_CS);

    // Put display into reset by pulling RST low
    GPIOB->ODR &= ~((uint16_t)1 << LCD_RST);

    // Wait for reset to take effect (~1μs)
    delayNOP();

    // Take display out of reset by setting RST high
    GPIOB->ODR |= ((uint16_t)1 << LCD_RST);

    // Wait for reset recovery
    delayNOP();

    // Select display by setting CS low
    GPIOB->ODR &= ~((uint16_t)1 << LCD_CS);

    // Wait before sending commands
    delayNOP();
}

/**
 * @brief   Clear the entire display
 * @details Fills all pages with zeros using page addressing mode
 */
void clearDisplay(void) {
    // Clear display using page addressing mode
    for(int y = 0; y < 8; y++) {
        // Set page address (0xB0-0xB7)
        displayWrite(0, 0b10110000 + y);
        // Set column address high nibble (0x1)
        displayWrite(0, 0b00010000);
        // Set column address low nibble (0x0)
        displayWrite(0, 0b00000000);

        // Fill page with zeros
        for(int k = 0; k < 131; k++) {
            // Set data mode (A0 high)
            GPIOB->ODR |= ((uint16_t)1 << LCD_A0);
            // Begin write cycle (WR low)
            GPIOB->ODR &= ~((uint16_t)1 << LCD_WRITE_EN);

            // Set data lines to zero (clear)
            uint16_t gpio_data = GPIOB->ODR;
            gpio_data &= ((uint16_t)0xFF << 8);  // Keep high byte unchanged
            gpio_data |= 0x00;                   // Set low byte (data) to 0
            GPIOB->ODR = gpio_data;

            // Setup time delay
            delayNOP();

            // End write cycle (latch data)
            GPIOB->ODR |= ((uint16_t)1 << LCD_WRITE_EN);

            // Hold time after latch
            delayNOP();
        }
    }
}

/**
 * @brief   Write command or data to the display
 * @param   cmd_dat   0 for command, 1 for data
 * @param   data      Byte to write
 * @details Low-level function for display communication using 8080 parallel interface
 */
void displayWrite(uint8_t cmd_dat, uint8_t data) {
    // Set A0 pin based on command/data mode
    if(cmd_dat)
        GPIOB->ODR |= ((uint16_t)1 << LCD_A0);  // Data mode (A0 high)
    else
        GPIOB->ODR &= ~((uint16_t)1 << LCD_A0); // Command mode (A0 low)

    // Begin write cycle (WR low) - 8080 interface latches on rising edge of WR
    GPIOB->ODR &= ~((uint16_t)1 << LCD_WRITE_EN);

    // Set data pins with the byte to send
    uint16_t gpio_data = GPIOB->ODR;
    gpio_data &= ((uint16_t)0xFF << 8);  // Keep high byte unchanged
    gpio_data |= data;                   // Set low byte to data value
    GPIOB->ODR = gpio_data;

    // Setup time delay (~500ns)
    asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
    asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");

    // End write cycle (latch data on rising edge of WR)
    GPIOB->ODR |= ((uint16_t)1 << LCD_WRITE_EN);

    // Hold time after latch
    delayNOP();
}
