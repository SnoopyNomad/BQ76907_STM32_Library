#include "bq76907.h"

/* BQ76907 I2C Address */
#define BQ76907_ADDR		0x08

/* Register Addresses */
#define SYS_STAT    		0x00
#define SYS_CTRL1   		0x04
#define SYS_CTRL2   		0x05
#define PROTECT1    		0x06
#define PROTECT2    		0x07
#define PROTECT3   			0x08
#define OV_TRIP     		0x09
#define UV_TRIP     		0x0A
#define CC_CFG      		0x0B
#define CC_HI       		0x0C
#define CC_LO       		0x0D
#define ADC_CONFIG      	0x35
#define ADCGAIN1    		0x50
#define ADCGAIN2    		0x51
#define VC1_HI      		0x3A
#define VC1_LO      		0x3B
#define VC2_HI      		0x3C
#define VC2_LO      		0x3D
#define VC3_HI      		0x3E
#define VC3_LO      		0x3F
#define VC4_HI      		0x40
#define VC4_LO      		0x41
#define VC5_HI      		0x42
#define VC5_LO      		0x43
#define VC6_HI      		0x44
#define VC6_LO      		0x45
#define VC7_HI      		0x46
#define VC7_LO      		0x47
#define VC8_HI      		0x48
#define VC8_LO      		0x49
#define VC9_HI      		0x4A
#define VC9_LO      		0x4B
#define SECONDS     		0x4C
#define TS1_HI      		0x52
#define TS1_LO      		0x53
#define TS2_HI      		0x54
#define TS2_LO      		0x55
#define TS3_HI      		0x56
#define TS3_LO      		0x57

#define ADC_CONFIG_ADC_ENABLE 0x01
#define ADC_CONFIG_ADC_MODE 0x02
#define ADC_CONFIG_ADC_MODE_CONTINUOUS 0x00
#define ADC_CONFIG_ADC_MODE_SINGLE_SHOT 0x01
#define ADC_CONFIG_ADC_MODE_SHUTDOWN 0x02
#define ADC_CONFIG_ADC_MODE_STANDBY 0x03

#define CELL1_GAIN_REG         0x4002
#define CELL1_GAIN_DELTA_REG   0x4004

/* Register Bit Definitions */
/* SYS_CTRL1 (0x04) */
#define SYS_CTRL1_SHUT_A    0x01
#define SYS_CTRL1_SHUT_B    0x02
#define SYS_CTRL1_SHUT_C    0x04
#define SYS_CTRL1_ADC_EN    0x08
#define SYS_CTRL1_TEMP_SENS 0x10

/* SYS_CTRL2 (0x05) */
#define SYS_CTRL2_SHORT_DELAY 0x08
#define SYS_CTRL2_AFE_PWR     0x20
#define SYS_CTRL2_LDO_PWR     0x40

/* Direct Commands */
#define CMD_SAFETY_ALERT_A				0x02
#define CMD_SAFETY_STATUS_A	 			0x03
#define CMD_SAFETY_ALERT_B				0x04
#define CMD_SAFETY_STATUS_B				0x05
#define CMD_BATTERY_STATUS				0x12
#define CMD_CELL_1_VOLTAGE				0x14
#define CMD_CELL_2_VOLTAGE				0x16
#define CMD_CELL_3_VOLTAGE				0x18
#define CMD_CELL_4_VOLTAGE				0x1A
#define CMD_CELL_5_VOLTAGE				0x1C
#define CMD_CELL_6_VOLTAGE				0x1E
#define CMD_CELL_7_VOLTAGE				0x20
#define CMD_REG18_VOLTAGE				0x22
#define CMD_VSS_VOLTAGE					0x24
#define CMD_STACK_VOLTAGE				0x26
#define CMD_INT_TEMPERATURE 			0x28
#define CMD_TS_MEASUREMENT				0x2A
#define CMD_RAW_CURRENT					0x36
#define CMD_CURRENT						0x3A
#define CMD_CC1_CURRENT					0x3C
#define CMD_ALARM_STATUS	 			0x62
#define CMD_ALARM_RAW_STATUS			0x64
#define CMD_ALARM_ENABLE				0x66
#define CMD_FET_CONTROL					0x68
#define CMD_REGOUT_CONTROL				0x69
#define CMD_DSG_FET_DRIVER_PWM_CONTROL	0x6A
#define CMD_CHG_FET_DRIVER_PWM_CONTROL	0x6C

/* Charge Integration Subcommands */
#define CMD_PASSQ_LSB                     0x0004
#define CMD_RESET_PASSQ                   0x0005

/* Cell Balancing Commands */
#define CMD_CB_ACTIVE_CELLS              0x0083
#define CB_TIMEOUT_MS                    20000  // 20 seconds timeout

/* Cell Balancing Configuration */
#define MIN_TEMP_THRESHOLD -20.0f   // Minimum temperature for balancing (°C)
#define MAX_TEMP_THRESHOLD 60.0f    // Maximum temperature for balancing (°C)
#define MAX_INTERNAL_TEMP 60.0f     // Maximum internal temperature for balancing (°C)
#define CELL_VOLTAGE_DIFF_THRESHOLD 50  // 50mV difference threshold for balancing
#define MAX_BALANCING_CELLS 2       // Maximum number of cells to balance simultaneously

extern I2C_HandleTypeDef hi2c1;

/**
 * @brief Maps HAL I2C status to BQ76907 error codes.
 * @param status HAL_StatusTypeDef returned by HAL I2C functions.
 * @return ErrorCode_t Corresponding BQ76907 error code.
 */
static inline ErrorCode_t I2C_MapErrorCode(HAL_StatusTypeDef status){
	if(status == HAL_OK) return BQ76907_OK;
	if(status == HAL_ERROR) return BQ76907_ERROR_I2C;
	if(status == HAL_BUSY) return BQ76907_ERROR_BUSY;
}

/**
 * @brief Reads data from a BQ76907 register over I2C.
 * @param address Register address to read from.
 * @param data Pointer to buffer to store read data.
 * @param length Number of bytes to read.
 * @return ErrorCode_t Status of the operation.
 */
static inline ErrorCode_t BQ76907_ReadRegister(uint8_t address, uint8_t *data, uint8_t length){
    ErrorCode_t errorCode = BQ76907_OK;
    errorCode = I2C_MapErrorCode(HAL_I2C_Mem_Read(&hi2c1, (BQ76907_ADDR << 1), address, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY));
    return errorCode;
}

/**
 * @brief Writes data to a BQ76907 register over I2C.
 * @param address Register address to write to.
 * @param data Pointer to buffer containing data to write.
 * @param length Number of bytes to write.
 * @return ErrorCode_t Status of the operation.
 */
static inline ErrorCode_t BQ76907_WriteRegister(uint8_t address, uint8_t *data, uint8_t length){
    ErrorCode_t errorCode = BQ76907_OK;
    errorCode = I2C_MapErrorCode(HAL_I2C_Mem_Write(&hi2c1, (BQ76907_ADDR << 1), address, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY));
    return errorCode;
}

/**
 * @brief Unseal the BQ76907 using the SECURITY_KEYS subcommand (0x0035)
 * @param key1 First 16-bit word of the security key
 * @param key2 Second 16-bit word of the security key
 * @return ErrorCode_t Status of the operation
 */
static inline ErrorCode_t BQ76907_Unseal(uint16_t key1, uint16_t key2) {
    ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x35, 0x00};  // SECURITY_KEYS subcommand (0x0035)
    uint8_t data[4];  // Buffer for 4 bytes of key data
    uint8_t readback[4];
    uint8_t checksum = 0;
    uint8_t length = 0x08;  // Length includes command, data, checksum, and length bytes

    // 1. Write subcommand (0x0035) to 0x3E and 0x3F
    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);  // Write 0x35
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);  // Write 0x00
    if(errorCode != BQ76907_OK) return errorCode;

    // 2. Read back 4 bytes from transfer buffer (0x40-0x43)
    errorCode = BQ76907_ReadRegister(0x40, readback, 4);
    if(errorCode != BQ76907_OK) return errorCode;

    // 3. Prepare data in little-endian format
    data[0] = (uint8_t)(key1 & 0xFF);        // LSB of key1
    data[1] = (uint8_t)((key1 >> 8) & 0xFF); // MSB of key1
    data[2] = (uint8_t)(key2 & 0xFF);        // LSB of key2
    data[3] = (uint8_t)((key2 >> 8) & 0xFF); // MSB of key2

    // 4. Write data to transfer buffer (0x40-0x43)
    errorCode = BQ76907_WriteRegister(0x40, data, 4);
    if(errorCode != BQ76907_OK) return errorCode;

    // 5. Calculate checksum (invert modulo-256 sum of data and command bytes)
    for(uint8_t i = 0; i < 4; i++) {
        checksum += data[i];
    }
    checksum += subcmd[0] + subcmd[1];
    checksum = ~checksum + 1;  // Two's complement

    // 6. Write checksum to 0x60
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // 7. Write length (0x08) to 0x61
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // Wait a bit for the unseal operation to complete
    HAL_Delay(50);

    return errorCode;
}

ErrorCode_t BQ76907_Init(void){
    ErrorCode_t errorCode = BQ76907_OK;
    uint8_t data;

    // Unseal the BQ76907 using SECURITY_KEYS subcommand
    // Replace 0x0414 and 0x2727 with your actual security keys if different
    errorCode = BQ76907_Unseal(0x0414, 0x2727);
    if(errorCode != BQ76907_OK) return errorCode;

    // Wait a bit for unseal to complete
    HAL_Delay(50);

    // Read device ID from SYS_STAT register
    errorCode = BQ76907_ReadRegister(SYS_STAT, &data, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // Enable voltage, current measurements, ADC, and temperature sensor in SYS_CTRL1
    data = SYS_CTRL1_SHUT_A | SYS_CTRL1_SHUT_B | SYS_CTRL1_ADC_EN | SYS_CTRL1_TEMP_SENS;
    errorCode = BQ76907_WriteRegister(SYS_CTRL1, &data, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // Configure alert pin for active low in SYS_CTRL2
    data = SYS_CTRL2_SHORT_DELAY | SYS_CTRL2_LDO_PWR;
    errorCode = BQ76907_WriteRegister(SYS_CTRL2, &data, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    data = ADC_CONFIG_ADC_ENABLE | ADC_CONFIG_ADC_MODE_CONTINUOUS; // 0x01
    errorCode = BQ76907_WriteRegister(ADC_CONFIG, &data, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return errorCode;
}

ErrorCode_t BQ76907_EnableAlerts(void){
    ErrorCode_t errorCode = BQ76907_OK;
    uint16_t data = 0xF000; // Mask for alerts

    errorCode = BQ76907_WriteRegister(CMD_ALARM_ENABLE, (uint8_t*)&data , 2);
    if(errorCode != BQ76907_OK) return errorCode;

    return errorCode;
}

ErrorCode_t BQ76907_ReadAlerts(uint16_t *alerts){
    ErrorCode_t errorCode = BQ76907_OK;
    uint8_t data[2];

    errorCode = BQ76907_ReadRegister(CMD_ALARM_RAW_STATUS, data, 2);
    if(errorCode != BQ76907_OK) return errorCode;

    *alerts = ((uint16_t)data[1] << 8) | data[0];
    return errorCode;
}

ErrorCode_t BQ76907_ClearAlerts(void){
	ErrorCode_t errorCode = BQ76907_OK;
	uint16_t data = 0xFFFF;

	errorCode = BQ76907_WriteRegister(CMD_ALARM_STATUS, (uint8_t*)data, 2);
    if(errorCode != BQ76907_OK) return errorCode;

    return errorCode;
}

/**
 * @brief Read cell voltages for a 6-cell configuration
 * @param cellVoltages Array to store cell voltages in mV
 * @return ErrorCode_t Status of the operation
 */
ErrorCode_t BQ76907_ReadCellVoltages(uint16_t *cellVoltages) {
    ErrorCode_t errorCode = BQ76907_OK;
    uint8_t data[2];
    uint8_t cellCmds[6] = {CMD_CELL_1_VOLTAGE, CMD_CELL_2_VOLTAGE, CMD_CELL_3_VOLTAGE, CMD_CELL_4_VOLTAGE, CMD_CELL_5_VOLTAGE, CMD_CELL_6_VOLTAGE};

    for(int cell = 0; cell < 6; cell++) {
        errorCode = BQ76907_ReadRegister(cellCmds[cell], data, 2);
        if(errorCode != BQ76907_OK) return errorCode;
        cellVoltages[cell] = ((uint16_t)data[1] << 8) | data[0];
    }
    return errorCode;
}

/**
 * @brief Read all three temperature sensor values from BQ76907
 * @param temp Array to store temperature values in °C
 * @return ErrorCode_t Status of the operation
 */
ErrorCode_t BQ76907_ReadTemperature(float *temp) {
    ErrorCode_t errorCode = BQ76907_OK;
    uint8_t data[2];

    errorCode = BQ76907_ReadRegister(CMD_TS_MEASUREMENT, data, 2);
    if(errorCode != BQ76907_OK) return errorCode;
    uint16_t adc = ((uint16_t)data[1] << 8) | data[0];
    float vTs = ((float)adc / 32768.0f) * (1.8f * 3.0f / 5.0f); // TS pin voltage in volts
    *temp = vTs / 0.0078125f; // 1 LSB = 0.0078125°C

    return errorCode;
}

/**
 * @brief Read the accumulated charge and timer from the BQ76907
 * @param accumulatedCharge Pointer to store the accumulated charge in userA-seconds
 * @param timerValue Pointer to store the timer value in 250ms units
 * @return ErrorCode_t Status of the operation
 */
ErrorCode_t BQ76907_ReadAccumulatedCharge(int64_t *accumulatedCharge, uint32_t *timerValue) {
    ErrorCode_t errorCode = BQ76907_OK;
    uint8_t data[12];

    // Write subcommand 0x0004 to 0x3E/0x3F
    uint8_t subcmd[2] = {0x04, 0x00};
    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // Read 12 bytes from transfer buffer (0x40-0x4B)
    errorCode = BQ76907_ReadRegister(0x40, data, 12);
    if(errorCode != BQ76907_OK) return errorCode;

    // Accumulated charge (signed 48 bits)
    uint64_t lower = ((uint32_t)data[0]) |
                     ((uint32_t)data[1] << 8) |
                     ((uint32_t)data[2] << 16) |
                     ((uint32_t)data[3] << 24);
    uint64_t upper = ((uint16_t)data[4]) |
                     ((uint16_t)data[5] << 8);
    int64_t charge = (int64_t)((upper << 32) | lower);
    // Sign-extend if negative
    if (upper & 0x8000) {
        charge |= (int64_t)0xFFFF000000000000LL;
    }
    *accumulatedCharge = charge;

    // Timer value (bytes 8-11, little-endian, units of 250ms)
    *timerValue = ((uint32_t)data[8]) |
                  ((uint32_t)data[9] << 8) |
                  ((uint32_t)data[10] << 16) |
                  ((uint32_t)data[11] << 24);

    return errorCode;
}

/**
 * @brief Reset the charge integrator and timer
 * @return ErrorCode_t Status of the operation
 */
ErrorCode_t BQ76907_ResetChargeIntegrator(void){
    // Write 0x01 to register 0x41 to reset charge integrator (example, adjust as needed)
    uint8_t reset = 0x01;
    return BQ76907_WriteRegister(0x41, &reset, 1);
}

/**
 * @brief Calculate State of Charge (SOC) based on accumulated charge
 * @param accumulatedCharge Accumulated charge in userA-seconds
 * @param nominalCapacity Nominal battery capacity in mAh
 * @return float SOC percentage (0-100%)
 */
float BQ76907_CalculateSOC(int64_t accumulatedCharge, uint32_t nominalCapacity) {
    // Convert accumulated charge from userA-seconds to mAh
    // 1 userA-second = 1/3600 mAh
    float charge_mAh = (float)accumulatedCharge / 3600.0f;
    
    // Calculate SOC percentage (100% - consumed percentage)
    float soc = 100.0f - ((charge_mAh / (float)nominalCapacity) * 100.0f);
    
    // Clamp SOC between 0 and 100
    if(soc < 0.0f) soc = 0.0f;
    if(soc > 100.0f) soc = 100.0f;
    
    return soc;
}

/**
 * @brief Enable cell balancing for specified cells
 * @param cellMask Bit mask of cells to balance (bit 0 = cell 1, bit 1 = cell 2, etc.)
 * @return ErrorCode_t Status of the operation
 */
ErrorCode_t BQ76907_EnableCellBalancing(uint8_t cellMask) {
    // Write cellMask to register 0x46 (example, adjust as needed)
    return BQ76907_WriteRegister(0x46, &cellMask, 1);
}

/**
 * @brief Disable cell balancing
 * @return ErrorCode_t Status of the operation
 */
ErrorCode_t BQ76907_DisableCellBalancing(void) {
    uint8_t data = 0x00;
    return BQ76907_WriteRegister(0x46, &data, 1);
}

/**
 * @brief Get currently active cell balancing status
 * @param activeCells Pointer to store the bit mask of actively balanced cells
 * @return ErrorCode_t Status of the operation
 */
ErrorCode_t BQ76907_GetCellBalancingStatus(uint8_t *activeCells) {
    // Read from register 0x46 (example, adjust as needed)
    return BQ76907_ReadRegister(0x46, activeCells, 1);
}

/**
 * @brief Check if temperature is within safe limits for cell balancing
 * @param temp Current temperature in °C
 * @return bool True if temperature is within safe limits
 */
static bool BQ76907_IsTemperatureSafe(float temp) {
    return (temp >= MIN_TEMP_THRESHOLD && temp <= MAX_TEMP_THRESHOLD);
}

/**
 * @brief Find cells that need balancing based on voltage differences
 * @param cellVoltages Array of cell voltages in mV
 * @param numCells Number of cells to check
 * @param balanceMask Pointer to store the bit mask of cells to balance
 * @return uint8_t Number of cells selected for balancing
 */
static uint8_t BQ76907_FindCellsToBalance(uint16_t *cellVoltages, uint8_t numCells, uint8_t *balanceMask) {
    uint16_t maxVoltage = cellVoltages[0];
    uint16_t minVoltage = cellVoltages[0];
    uint8_t cellsToBalance = 0;
    *balanceMask = 0;

    // Find highest and lowest cell voltages
    for(uint8_t i = 1; i < numCells; i++) {
        if(cellVoltages[i] > maxVoltage) {
            maxVoltage = cellVoltages[i];
        }
        if(cellVoltages[i] < minVoltage) {
            minVoltage = cellVoltages[i];
        }
    }

    // Check if voltage difference exceeds threshold
    if((maxVoltage - minVoltage) > CELL_VOLTAGE_DIFF_THRESHOLD) {
        // Select cells with highest voltage for balancing
        for(uint8_t i = 0; i < numCells && cellsToBalance < MAX_BALANCING_CELLS; i++) {
            if(cellVoltages[i] > (minVoltage + CELL_VOLTAGE_DIFF_THRESHOLD)) {
                *balanceMask |= (1 << i);
                cellsToBalance++;
            }
        }
    }

    return cellsToBalance;
}

/**
 * @brief Manage cell balancing based on temperature and voltage conditions
 * @param isCharging Whether the battery is currently charging
 * @param activeBalancingCells Pointer to store the current balancing mask
 * @return ErrorCode_t Status of the operation
 */
ErrorCode_t BQ76907_ManageCellBalancing(bool isCharging, uint8_t *activeBalancingCells) {
    ErrorCode_t errorCode = BQ76907_OK;
    float temp;
    uint16_t cellVoltages[6];

    // Read temperature first
    errorCode = BQ76907_ReadTemperature(&temp);
    if(errorCode != BQ76907_OK) return errorCode;

    // Check if temperature is safe for balancing
    bool tempOk = BQ76907_IsTemperatureSafe(temp);

    if(isCharging && tempOk) {
        // Read cell voltages
        errorCode = BQ76907_ReadCellVoltages(cellVoltages);
        if(errorCode == BQ76907_OK) {
            uint8_t newBalanceMask;
            uint8_t cellsToBalance = BQ76907_FindCellsToBalance(cellVoltages, 6, &newBalanceMask);

            if(cellsToBalance > 0) {
                // Enable balancing for selected cells if mask has changed
                if(newBalanceMask != *activeBalancingCells) {
                    errorCode = BQ76907_EnableCellBalancing(newBalanceMask);
                    if(errorCode == BQ76907_OK) {
                        *activeBalancingCells = newBalanceMask;
                    }
                }
            } else {
                // Disable balancing if no cells need balancing
                if(*activeBalancingCells != 0) {
                    errorCode = BQ76907_DisableCellBalancing();
                    if(errorCode == BQ76907_OK) {
                        *activeBalancingCells = 0;
                    }
                }
            }
        }
    } else {
        // Disable balancing if temperature is unsafe or not charging
        if(*activeBalancingCells != 0) {
            errorCode = BQ76907_DisableCellBalancing();
            if(errorCode == BQ76907_OK) {
                *activeBalancingCells = 0;
            }
        }
    }

    return errorCode;
}