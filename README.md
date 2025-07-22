# UMeterLCD - STM32G0 LCD/OLED Display Interface Controller

## Overview

**UMeterLCD** is an embedded firmware project for STM32G030C8T microcontrollers that provides an intelligent LCD/OLED display interface controller. The system acts as a bridge between a host device and LCD/OLED displays, with built-in command filtering, power management, and interrupt-driven communication.

---

## Key Features

- **Multi-Display Support**  
  Compatible with SSD1305/SSD1309 OLED displays and standard LCD controllers

- **Interrupt-Driven Communication**  
  High-speed parallel 8080-series interface with external interrupt handling

- **Command Filtering**  
  Intelligent filtering system that blocks initialization commands while allowing display data and addressing commands

- **Power Management**  
  Sleep mode operation with wake-on-interrupt for energy efficiency

- **Hardware Reset Control**  
  Built-in display reset and initialization sequences

- **Configuration Jumpers**  
  Hardware configuration via GPIO jumpers for display inversion and test modes

- **Column Address Translation**  
  Automatic column address adjustment for SSD1309 displays (132 columns → 128 columns)

---

## Hardware Requirements

### Target Microcontroller

- **STM32G030C8T**  
  Cortex-M0+, 64MHz, 64KB Flash, 8KB RAM

### Pin Configuration

| Pin      | Function                             |
|----------|--------------------------------------|
| PA0-PA7  | Input data bus from host             |
| PA9      | ERD (Enable Read) signal input       |
| PA10     | Strobe signal input (interrupt)      |
| PA11     | Command/Data control input           |
| PB0-PB7  | Output data bus to LCD/OLED          |
| PB8      | LCD CS (Chip Select)                 |
| PB9      | LCD RST (Reset)                      |
| PB10     | LCD READ_EN (Read Enable)            |
| PB11     | LCD WRITE_EN (Write Enable)          |
| PB12     | LCD A0 (Command/Data select)         |
| PB13     | Status/Debug output                  |
| PD0-PD1  | Configuration jumpers                |

---

## Supported Display Types

The firmware supports three display configurations via compile-time selection:

- **SSD1309 OLED (Default)**  
  128x64 OLED with 132-column controller

- **SSD1305 OLED**  
  128x64 OLED display

- **LCD**  
  Standard LCD controller (ST7565 compatible)

---

## Operation Modes

### Normal Operation

- System enters sleep mode to conserve power  
- Wakes on strobe interrupt (PA10 rising edge)  
- Processes incoming commands and data from host  
- Filters out initialization commands to prevent display corruption  
- Forwards allowed addressing commands and display data to LCD/OLED  

### Configuration Modes

- **Display Inversion**: Jumper settings on PD0/PD1 control normal/inverted display  
- **Test Mode**: Grounding PD0 enables test mode (clears display and halts)  

---

## Technical Details

### Communication Protocol

- **Interface**: 8080-series parallel interface  
- **Data Width**: 8-bit parallel  
- **Control Signals**: CS, RST, RD, WR, A0  
- **Timing**: Hardware-optimized with NOP-based delays  

### Command Filtering

The system intelligently filters commands to prevent display corruption:

- **Allowed**:  
  - Page addressing: `0xB0–0xB7`  
  - Column addressing: `0x00–0x1F`

- **Blocked**:  
  - Initialization commands  
  - Configuration commands  
  - Contrast settings

- **Multi-byte Handling**:  
  Automatic parameter skipping for blocked multi-byte commands

### Power Management

- Sleep mode between interrupts  
- Wake-on-interrupt capability  
- Optimized for battery-powered applications  

---

## Building and Deployment

This project is designed for **STM32CubeIDE** or a compatible **ARM GCC toolchain**:

1. Import project into STM32CubeIDE  
2. Select target configuration (`SSD1309`, `SSD1305`, or `LCD`) in `lcd_if.h`  
3. Build and flash to STM32G030C8T target  
4. Connect display and host according to the pin configuration  

---

## File Structure

```
Core/
├── Inc/
│   ├── lcd_if.h                # LCD interface definitions and configuration
│   ├── main.h                  # Main application header
│   ├── stm32g0xx_hal_conf.h    # HAL configuration
│   └── stm32g0xx_it.h          # Interrupt handler declarations
├── Src/
│   ├── lcd_if.c                # LCD/OLED interface implementation
│   ├── main.c                  # Main application and system initialization
│   ├── stm32g0xx_it.c          # Interrupt service routines
│   ├── stm32g0xx_hal_msp.c     # HAL MSP functions
│   └── system_stm32g0xx.c      # System clock configuration
└── Startup/
    └── startup_stm32g030c8tx.s  # Startup assembly code
```
