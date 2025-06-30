# BQ76907 STM32 Library

A lightweight, easy-to-integrate C library for interfacing with the Texas Instruments BQ76907 battery management system IC. This library provides functions to monitor and protect battery cells, handle cell balancing, manage battery safety features, and track state of charge (SoC)—allowing you to build robust battery management systems on STM32 microcontrollers.

## Table of Contents
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
  - [Initialization](#initialization)
  - [Reading Battery Data](#reading-battery-data)
  - [State of Charge (SoC)](#state-of-charge-soc)
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
- State of Charge (SoC) tracking (voltage-based and coulomb counting)
- Cell balancing (automatic and manual)
- Safety features:
  - Cell Under-Voltage (CUV) protection
  - Cell Over-Voltage (COV) protection
  - Over-Current (OC) protection
  - Short-Circuit protection
  - Temperature monitoring
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

2. **Include the Library**:  
```c
#include "bq76907.h"
```

3. **Initialize the BQ76907**:  
```c
BQ76907_ErrorCode_t errorCode;
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

errorCode = BQ76907_ReadCellVoltages(cellVoltages);
errorCode = BQ76907_ReadBatteryTemperature(&temperature);
errorCode = BQ76907_ReadAlarmStatus(&alarmStatus);
errorCode = BQ76907_ReadSafetyStatusA(&safetyStatusA);
errorCode = BQ76907_ReadSafetyStatusB(&safetyStatusB);
```

### State of Charge (SoC)

```c
// Initialize SoC (should be called after BQ76907_Init)
errorCode = BQ76907_SoCInit();
if(errorCode != BQ76907_OK){
    Error_Handler();
}

// Periodically update SoC (e.g., in main loop)
errorCode = BQ76907_SoCUpdate();
if(errorCode != BQ76907_OK){
    Error_Handler();
}

// Get current SoC (percentage)
uint8_t soc = BQ76907_GetSoC();
```

### Cell Balancing

```c
// Automatic balancing (call periodically)
errorCode = BQ76907_ManageCellBalancing();

// Manual control
errorCode = BQ76907_EnableCellBalancing(0x06); // Example: balance cells 2 and 3
errorCode = BQ76907_DisableCellBalancing();

uint8_t activeCells;
errorCode = BQ76907_GetCellBalancingStatus(&activeCells);
```

### Safety Features

```c
// Protection recovery
errorCode = BQ76907_ProtRecovery();
```

## Error Handling

All functions return `BQ76907_ErrorCode_t`:
```c
typedef enum {
    BQ76907_OK = 0,
    BQ76907_ERROR_I2C = 1,
    BQ76907_ERROR_BUSY = 2,
    BQ76907_ERROR_TIMEOUT = 3,
    BQ76907_ERROR_UNKNOWN = 4,
    BQ76907_ERROR_UNDERTEMPERATURE_CHARGE = 5,
    BQ76907_ERROR_UNDERTEMPERATURE_DISCHARGE = 6,
    BQ76907_ERROR_INT_OVERTEMPERATURE = 7,
    BQ76907_ERROR_OVERTEMPERATURE_CHARGE = 8,
    BQ76907_ERROR_OVERTEMPERATURE_DISCHARGE = 9,
    BQ76907_ERROR_OVERCURRENT_CHARGE = 10,
    BQ76907_ERROR_OVERCURRENT_DISCHARGE_2 = 11,
    BQ76907_ERROR_OVERCURRENT_DISCHARGE_1 = 12,
    BQ76907_ERROR_SHORT_CIRCUIT_DISCHARGE = 13,
    BQ76907_ERROR_CELL_UNDERVOLTAGE = 14,
    BQ76907_ERROR_CELL_OVERVOLTAGE = 15
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
