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
  - [State of Charge](#state-of-charge)
  - [Cell Balancing](#cell-balancing)
  - [Safety Features](#safety-features)
- [Error Handling](#error-handling)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Features

- Initialize and configure BQ76907 via I²C
- Monitor up to 6 cell voltages
- Temperature monitoring with NTC thermistor support
- State of Charge (SoC) tracking
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
   #define BQ76907_INT_Pin        GPIO_PIN_0
   #define BQ76907_INT_GPIO_Port  GPIOA
   ```

## Configuration

The library is configured with the following default settings:
- Cell Under-Voltage (CUV) threshold: 3.0V
- Over-Current (OC) thresholds:
  - OCC: 4A
  - OCD1: 24A
  - OCD2: 24A
- Temperature monitoring enabled
- Cell balancing enabled
- Safety protections enabled
- Deep sleep mode disabled

## Usage

### Initialization

1. Configure I²C in your STM32 project:
```c
// In your main.c or system initialization
I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_I2C1_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_I2C1_Init();
    // ... rest of your initialization code
}
```

2. **Include the Library**: Add the library header to your project.
```c
#include "bq76907.h"
```

3. Initialize the BQ76907:
```c
typedef enum{
  BQ76907_OK = 0,
  BQ76907_ERROR_I2C = 1,
  BQ76907_ERROR_BUSY = 2,
  BQ76907_ERROR_UNDERTEMPERATURE_CHARGE = 3,
  BQ76907_ERROR_UNDERTEMPERATURE_DISCHARGE = 4,
  BQ76907_ERROR_INT_OVERTEMPERATURE = 5,
  BQ76907_ERROR_OVERTEMPERATURE_CHARGE = 6,
  BQ76907_ERROR_OVERTEMPERATURE_DISCHARGE = 7,
  BQ76907_ERROR_OVERCURRENT_CHARGE = 8,
  BQ76907_ERROR_OVERCURRENT_DISCHARGE_2 = 9,
  BQ76907_ERROR_OVERCURRENT_DISCHARGE_1 = 10,
  BQ76907_ERROR_SHORT_CIRCUIT_DISCHARGE = 11,
  BQ76907_ERROR_CELL_UNDERVOLTAGE = 12,
  BQ76907_ERROR_CELL_OVERVOLTAGE = 13
} ErrorCode_t;

ErrorCode_t errorCode;

// Basic initialization
errorCode = BQ76907_Init();
if(errorCode != BQ76907_OK){
    Error_Handler();
}

// Enable alarms
errorCode = BQ76907_EnableAlarms();
if(errorCode != BQ76907_OK){
    Error_Handler();
}
```

### Reading Battery Data

```c
uint16_t cellVoltages[6];
float temperature;
uint16_t alarmStatus;
uint8_t safetyStatusA, safetyStatusB;
float soc;

// Read cell voltages
errorCode = BQ76907_ReadCellVoltages(cellVoltages);
if(errorCode != BQ76907_OK){
    Error_Handler();
}

// Read temperature
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

### State of Charge

The library provides basic State of Charge (SoC) tracking functionality.

```c
// Initialize SoC
errorCode = BQ76907_SoCInit();
if(errorCode != BQ76907_OK){
    Error_Handler();
}

while(1){
    errorCode = BQ76907_SoCUpdate();
    if(errorCode != BQ76907_OK){
        Error_Handler();
    }
    soc = BQ76907_GetSoC();
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
errorCode = BQ76907_EnterDeepSleep();
errorCode = BQ76907_ExitDeepSleep();
```

## Error Handling

All functions return `ErrorCode_t`:
```c
typedef enum {
    BQ76907_OK = 0,                           // Operation successful
    BQ76907_ERROR_I2C = 1,                    // I²C communication error
    BQ76907_ERROR_BUSY = 2,                   // Device is busy
    BQ76907_ERROR_UNDERTEMPERATURE_CHARGE = 3, // Battery temperature too low for charging
    BQ76907_ERROR_UNDERTEMPERATURE_DISCHARGE = 4, // Battery temperature too low for discharging
    BQ76907_ERROR_INT_OVERTEMPERATURE = 5,    // Internal temperature too high
    BQ76907_ERROR_OVERTEMPERATURE_CHARGE = 6,  // Battery temperature too high for charging
    BQ76907_ERROR_OVERTEMPERATURE_DISCHARGE = 7, // Battery temperature too high for discharging
    BQ76907_ERROR_OVERCURRENT_CHARGE = 8,     // Charging current too high
    BQ76907_ERROR_OVERCURRENT_DISCHARGE_2 = 9, // Discharge current too high (level 2)
    BQ76907_ERROR_OVERCURRENT_DISCHARGE_1 = 10, // Discharge current too high (level 1)
    BQ76907_ERROR_SHORT_CIRCUIT_DISCHARGE = 11, // Short circuit detected during discharge
    BQ76907_ERROR_CELL_UNDERVOLTAGE = 12,     // Cell voltage below minimum threshold
    BQ76907_ERROR_CELL_OVERVOLTAGE = 13       // Cell voltage above maximum threshold
} ErrorCode_t;

ErrorCode_t HandleBQ76907Alarms(void){
	ErrorCode_t errorCode = BQ76907_OK;
	uint8_t data;

	errorCode = BQ76907_ReadAlarmStatus(&alarms);
	if(errorCode != BQ76907_OK){
		LEDOn();
		return errorCode;
	}

	if(alarms & 0x1000){
		LEDOn();
		errorCode = BQ76907_ReadSafetyAlertB(&data);
		if(errorCode != BQ76907_OK){
			LEDOn();
			return errorCode;
		}
		errorCode = BQ76907_ClearAlarmStatus(0x1000);
		if(errorCode != BQ76907_OK){
			LEDOn();
			return errorCode;
		}
		switch(data){
			case 0x08: return BQ76907_ERROR_INT_OVERTEMPERATURE;
			case 0x10: return BQ76907_ERROR_UNDERTEMPERATURE_CHARGE;
			case 0x20: return BQ76907_ERROR_UNDERTEMPERATURE_DISCHARGE;
			case 0x40: return BQ76907_ERROR_OVERTEMPERATURE_CHARGE;
			case 0x80: return BQ76907_ERROR_OVERTEMPERATURE_DISCHARGE;
		}
	}
	if(alarms & 0x2000){
		LEDOn();
		errorCode = BQ76907_ReadSafetyAlertA(&data);
		if(errorCode != BQ76907_OK) {
			LEDOn();
			return errorCode;
		}
		errorCode = BQ76907_ClearAlarmStatus(0x2000);
		if(errorCode != BQ76907_OK){
			LEDOn();
			return errorCode;
		}
		switch(data){
			case 0x04: return BQ76907_ERROR_OVERCURRENT_CHARGE;
			case 0x08: return BQ76907_ERROR_OVERCURRENT_DISCHARGE_2;
			case 0x10: return BQ76907_ERROR_OVERCURRENT_DISCHARGE_1;
			case 0x20: return BQ76907_ERROR_SHORT_CIRCUIT_DISCHARGE;
			case 0x40: return BQ76907_ERROR_CELL_UNDERVOLTAGE;
			case 0x80: return BQ76907_ERROR_CELL_OVERVOLTAGE;
	  }
	}
	if(alarms & 0x4000){
		LEDOn();
		errorCode = BQ76907_ReadSafetyStatusB(&data);
		if(errorCode != BQ76907_OK) {
			LEDOn();
			return errorCode;
		}
		errorCode = BQ76907_ClearAlarmStatus(0x4000);
		if(errorCode != BQ76907_OK){
			LEDOn();
			return errorCode;
		}
    	switch(data){
			case 0x08: return BQ76907_ERROR_INT_OVERTEMPERATURE;
			case 0x10: return BQ76907_ERROR_UNDERTEMPERATURE_CHARGE;
			case 0x20: return BQ76907_ERROR_UNDERTEMPERATURE_DISCHARGE;
			case 0x40: return BQ76907_ERROR_OVERTEMPERATURE_CHARGE;
			case 0x80: return BQ76907_ERROR_OVERTEMPERATURE_DISCHARGE;
		}
	}
	if(alarms & 0x8000){
		LEDOn();
		errorCode = BQ76907_ReadSafetyStatusA(&data);
		if(errorCode != BQ76907_OK) {
			LEDOn();
			return errorCode;
		}
		errorCode = BQ76907_ClearAlarmStatus(0x8000);
		if(errorCode != BQ76907_OK){
			LEDOn();
			return errorCode;
		}
    	switch(data){
    		case 0x04: return BQ76907_ERROR_OVERCURRENT_CHARGE;
    		case 0x08: return BQ76907_ERROR_OVERCURRENT_DISCHARGE_2;
    		case 0x10: return BQ76907_ERROR_OVERCURRENT_DISCHARGE_1;
    		case 0x20: return BQ76907_ERROR_SHORT_CIRCUIT_DISCHARGE;
    		case 0x40: return BQ76907_ERROR_CELL_UNDERVOLTAGE;
    		case 0x80: return BQ76907_ERROR_CELL_OVERVOLTAGE;
    	}
	}

	return BQ76907_OK;
}
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
