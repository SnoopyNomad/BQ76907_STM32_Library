# BQ76907 STM32 Library

A lightweight, easy-to-integrate C library for interfacing with the Texas Instruments BQ76907 battery management system IC. This library provides functions to monitor and protect battery cells, handle cell balancing, and manage battery safety features—allowing you to build robust battery management systems on STM32 microcontrollers.

## Table of Contents
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
  - [Initialization](#initialization)
  - [Reading Battery Data](#reading-battery-data)
  - [Security Key Handling](#security-key-handling)
  - [Cell Balancing](#cell-balancing)
  - [Safety Features](#safety-features)
- [Error Handling](#error-handling)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Features

- Initialize and configure BQ76907 via I²C
- Monitor up to 6 cell voltages
- Temperature monitoring with NTC thermistor support (returns `int8_t` Celsius)
- Security key read/write (16-bit keys)
- Cell balancing functionality
- Safety features:
  - Cell Under-Voltage (CUV) protection
  - Over-Current (OC) protection
  - Short-Circuit protection
  - Temperature monitoring
- FET control for charge/discharge
- Deep sleep mode support
- Alarm and status monitoring
- Compatible with STM32 HAL drivers

## Prerequisites

- STM32 microcontroller
- STM32Cube HAL drivers installed
- I²C peripheral configured in your project
- Power supply and wiring to BQ76907 per datasheet
- Basic understanding of STM32 development

## Installation

1. Clone this repository into your project directory:  
```bash
git clone https://github.com/SnoopyNomad/BQ76907_STM32_Library.git
```

2. Copy the following files into your project:
   - `bq76907.c` → Your project's source folder
   - `bq76907.h` → Your project's include folder

3. Configure your pin definitions in `main.h`:
```c
#define BQ76907_ALERT_Pin        GPIO_PIN_0
#define BQ76907_ALERT_GPIO_Port  GPIOA
```

## Configuration

The library is configured with the following default settings:
- Cell Under-Voltage (CUV) threshold: 2.5V
- Cell Over-Voltage (COV) threshold: 4.2V
- Over-Current (OC) thresholds:
  - OCC: 4A
  - OCD1: 24A
  - OCD2: 24A
- Temperature monitoring enabled
- Cell balancing enabled
- Safety protections enabled
- Sleep mode disabled

## Usage

### Initialization

1. Configure I²C in your STM32 project:
```c
/* In your main.c or system initialization */
I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_I2C1_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_I2C1_Init();
   /* ... rest of your initialization code */
}
```

2. **Include the Library**: Add the library header to your project.
```c
#include "bq76907.h"
```

3. Initialize the BQ76907:
```c
ErrorCode_t errorCode;

errorCode = BQ76907_Init();
if(errorCode != BQ76907_OK){
    Error_Handler();
}
```

### Reading Battery Data

```c
uint16_t cellVoltages[6];
int8_t temperature;
uint16_t alarmStatus;
uint8_t safetyStatusA, safetyStatusB;

// Read cell voltages
errorCode = BQ76907_ReadCellVoltages(cellVoltages);
if(errorCode != BQ76907_OK){
    Error_Handler();
}

// Read temperature (Celsius)
errorCode = BQ76907_ReadBatteryTemperature(&temperature);
if(errorCode != BQ76907_OK){
    Error_Handler();
}

// Read alarm status
errorCode = BQ76907_ReadAlarmStatus(&alarmStatus);
if(errorCode != BQ76907_OK){
    Error_Handler();
}

// Read safety status
errorCode = BQ76907_ReadSafetyStatusA(&safetyStatusA);
errorCode = BQ76907_ReadSafetyStatusB(&safetyStatusB);
if(errorCode != BQ76907_OK){
    Error_Handler();
}
```

### Security Key Handling

```c
uint16_t key1, key2;
// Read security keys
errorCode = BQ76907_ReadSecurityKeys(&key1, &key2);
if(errorCode != BQ76907_OK){
    Error_Handler();
}

// Write security keys (example)
uint8_t keys[4];
keys[0] = key1 & 0xFF; keys[1] = (key1 >> 8) & 0xFF;
keys[2] = key2 & 0xFF; keys[3] = (key2 >> 8) & 0xFF;
errorCode = BQ76907_WriteKeys(keys);
if(errorCode != BQ76907_OK){
    Error_Handler();
}
```

### Cell Balancing

```c
// Balance cells with minimum and maximum voltages
errorCode = BQ76907_BalanceMinMaxCells(cellVoltages);
if(errorCode != BQ76907_OK){
    Error_Handler();
}

// Read active balancing cells
uint8_t activeCells;
errorCode = BQ76907_ReadCBActiveCells(&activeCells);
if(errorCode != BQ76907_OK){
    Error_Handler();
}
```

### Safety Features

```c
// Enable/disable charge FET
errorCode = BQ76907_CHGFETOn();  // Enable charging
errorCode = BQ76907_CHGFETOff(); // Disable charging

// Enable/disable discharge FET
errorCode = BQ76907_DSGFETOn();  // Enable discharging
errorCode = BQ76907_DSGFETOff(); // Disable discharging

// Enter/exit deep sleep mode
// (see code for power management and protection recovery functions)
```

## Error Handling

All functions return `BQ76907_ErrorCode_t`:
```c
typedef enum {
    BQ76907_OK = 0,                           // Operation successful
    BQ76907_ERROR_I2C = 1,                    // I²C communication error
    BQ76907_ERROR_BUSY = 2,                   // Device is busy
    BQ76907_ERROR_TIMEOUT = 3,                // I²C timeout
    BQ76907_ERROR_UNKNOWN = 4,                // Unknown error
    BQ76907_ERROR_UNDERTEMPERATURE_CHARGE = 5, // Battery temperature too low for charging
    BQ76907_ERROR_UNDERTEMPERATURE_DISCHARGE = 6, // Battery temperature too low for discharging
    BQ76907_ERROR_INT_OVERTEMPERATURE = 7,    // Internal temperature too high
    BQ76907_ERROR_OVERTEMPERATURE_CHARGE = 8,  // Battery temperature too high for charging
    BQ76907_ERROR_OVERTEMPERATURE_DISCHARGE = 9, // Battery temperature too high for discharging
    BQ76907_ERROR_OVERCURRENT_CHARGE = 10,     // Charging current too high
    BQ76907_ERROR_OVERCURRENT_DISCHARGE_2 = 11, // Discharge current too high (level 2)
    BQ76907_ERROR_OVERCURRENT_DISCHARGE_1 = 12, // Discharge current too high (level 1)
    BQ76907_ERROR_SHORT_CIRCUIT_DISCHARGE = 13, // Short circuit detected during discharge
    BQ76907_ERROR_CELL_UNDERVOLTAGE = 14,     // Cell voltage below minimum threshold
    BQ76907_ERROR_CELL_OVERVOLTAGE = 15       // Cell voltage above maximum threshold
} BQ76907_ErrorCode_t;
```

Common error scenarios:
- I²C communication failure
- Invalid device address
- Sensor not responding
- Configuration errors
- Safety protection triggers:
  - Temperature limits exceeded
  - Current limits exceeded
  - Cell voltage limits exceeded
  - Short circuit conditions

## Contributing

Contributions are welcome! Please follow these steps:
1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

MIT - see [LICENSE](LICENSE).

## Acknowledgments

- Texas Instruments (BQ76907)
- STMicroelectronics (HAL drivers)
- Community contributors
