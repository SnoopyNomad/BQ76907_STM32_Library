/**
 * @file bq76907.c
 * @brief BQ76907 battery management system driver
 * @details This file contains the implementation of the
 *          BQ76907 battery management system driver, supporting various
 *          operations.
 * @author Cengiz Sinan Kostakoglu
 * @version 1.1
 * @date 2025-06-19
 */

#include "bq76907.h"

/* BQ76907 I2C Address */
#define BQ76907_ADDR		    0x08

/* Register Addresses */
#define SAFETY_ALERT_A	        0x02
#define SAFETY_STATUS_A         0x03
#define SAFETY_ALERT_B	        0x04
#define SAFETY_STATUS_B         0x05
#define BATTERY_STATUS	        0x12
#define CELL_1_VOLTAGE	        0x14
#define CELL_2_VOLTAGE	        0x16
#define CELL_3_VOLTAGE	        0x18
#define CELL_4_VOLTAGE	        0x1A
#define CELL_5_VOLTAGE	        0x1C
#define CELL_6_VOLTAGE	        0x1E
#define TS_MEASUREMENT	        0x2A
#define ALARM_STATUS            0x62
#define ALARM_RAW_STATUS        0x64
#define ALARM_ENABLE	        0x66
#define FET_CONTROL	            0x68
#define REGOUT_CONTROL	        0x69

/* Number of Cells */
#define NUM_CELLS               6

/* Battery Capacity Constants */
#define NOMINAL_CAPACITY_MAH    2800.0f  // 3000mAh per cell (capacity is same in series)
#define CELL_VOLTAGE_FULL       4200     // Full charge voltage per cell (mV)
#define CELL_VOLTAGE_EMPTY      2500     // Empty voltage per cell (mV)
#define CELL_VOLTAGE_RANGE      (CELL_VOLTAGE_FULL - CELL_VOLTAGE_EMPTY)

/* I2C Handle */
extern I2C_HandleTypeDef hi2c1;

/* SoC Variables */
float SoC = 0.0f;
float initialSoC = 0.0f;
bool socInitialized = false;
int64_t lastCharge = 0;
uint32_t lastTime = 0;

/* SoC Configuration */
static float socCapacityMah = NOMINAL_CAPACITY_MAH;
static uint16_t socVoltageFull = CELL_VOLTAGE_FULL;
static uint16_t socVoltageEmpty = CELL_VOLTAGE_EMPTY;

/**
 * @brief Map HAL status to BQ76907 error code
 * @param status HAL status
 * @return BQ76907 error code (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t I2CMapError(HAL_StatusTypeDef status){
    switch (status){
        case HAL_OK: return BQ76907_OK;
        case HAL_ERROR: return BQ76907_ERROR_I2C;
        case HAL_BUSY: return BQ76907_ERROR_BUSY;
        case HAL_TIMEOUT: return BQ76907_ERROR_TIMEOUT;
        default: return BQ76907_ERROR_UNKNOWN;
    }
}

/**
 * @brief Read a single byte from the BQ76907 register
 * @param reg Register address
 * @param data Pointer to store the read data
 * @param length Number of bytes to read
 * @return BQ76907 error code (BQ76907_OK on success)
 */ 
static inline BQ76907_ErrorCode_t BQ76907_ReadRegister(uint8_t reg, uint8_t *data, uint8_t length){
    return I2CMapError(HAL_I2C_Mem_Read(&hi2c1, (BQ76907_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY));
}


/**
 * @brief Write a single byte to the BQ76907 register
 * @param reg Register address
 * @param data Data to write
 * @param length Number of bytes to write
 * @return BQ76907 error code (BQ76907_OK on success)
 */ 
static inline BQ76907_ErrorCode_t BQ76907_WriteRegister(uint8_t reg, uint8_t *data, uint8_t length){ 
    return I2CMapError(HAL_I2C_Mem_Write(&hi2c1, (BQ76907_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY));
}

/**
 * @brief Read the battery status
 * @param status Pointer to store the battery status
 * @return BQ76907 error code (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_ReadBatteryStatus(uint16_t *status){
    BQ76907_ErrorCode_t errorCode;

    errorCode = BQ76907_ReadRegister(BATTERY_STATUS, (uint8_t *)status, sizeof(uint16_t));
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Read the cell voltages    
 * @param voltages Pointer to store the cell voltages
 * @return BQ76907 error code (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_ReadCellVoltages(uint16_t *cellVoltages){
    BQ76907_ErrorCode_t errorCode;

    errorCode = BQ76907_ReadRegister(CELL_1_VOLTAGE, (uint8_t *)cellVoltages, NUM_CELLS * sizeof(uint16_t));
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Convert the ADC code to temperature
 * @param adcCode ADC code
 * @return Temperature in Celsius
 */
static inline int8_t TSADCtoTemperature(uint16_t adcCode){
    float vTs = adcCode * 1.8f * 5.0f / 3.0f / 32768.0f; // ≈ 92uV per LSB
    float vRef = 1.8f;
    float rPullup = 20000.0f; // 20kΩ
    float rNTC = rPullup * vTs / (vRef - vTs);

    // Define thermistor parameters (adjust as needed for your NTC)
    float B = 4300.0f;      // Beta value of thermistor (typical: 3435 or 3950)
    float R0 = 10000.0f;    // Resistance at 25°C (10kΩ)
    float T0 = 298.15f;     // 25°C in Kelvin

    float tempK = 1.0f / ((1.0f/B) * logf(rNTC/R0) + (1.0f/T0));
    float tempC = tempK - 273.15f;
    return (int8_t)roundf(tempC);
}   

/**
 * @brief Read the battery temperature
 * @param temperature Pointer to store the temperature
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_ReadBatteryTemperature(int8_t *temperature){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint16_t adcCode;

    errorCode = BQ76907_ReadRegister(TS_MEASUREMENT, (uint8_t*)&adcCode, 2);
    if(errorCode != BQ76907_OK) return errorCode;

    *temperature = TSADCtoTemperature(adcCode);

    return BQ76907_OK;
}

/**
 * @brief Read the security keys from the BQ76907
 * @param key1 Pointer to store the first 16-bit key
 * @param key2 Pointer to store the second 16-bit key
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_ReadSecurityKeys(uint16_t *key1, uint16_t *key2){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x35, 0x00}; // SECURITY_KEYS subcommand
    uint8_t keys[4];

    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ReadRegister(0x40, keys, 4);
    if(errorCode != BQ76907_OK) return errorCode;

    *key1 = (uint16_t)keys[0] | ((uint16_t)keys[1] << 8);
    *key2 = (uint16_t)keys[2] | ((uint16_t)keys[3] << 8);

    return BQ76907_OK;
}

/**
 * @brief Seal the BQ76907 (return to protected mode)
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_Seal(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x30, 0x00}; // SEAL() subcommand

    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Unseal the BQ76907 with keys (exit protected mode)
 * @param key1 16-bit key1
 * @param key2 16-bit key2
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_Unseal(uint16_t key1, uint16_t key2) {
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t keyBytes[2];

    keyBytes[0] = key1 & 0xFF;        // LSB
    keyBytes[1] = (key1 >> 8) & 0xFF; // MSB
    errorCode = BQ76907_WriteRegister(0x3E, &keyBytes[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &keyBytes[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    keyBytes[0] = key2 & 0xFF;        // LSB
    keyBytes[1] = (key2 >> 8) & 0xFF; // MSB
    errorCode = BQ76907_WriteRegister(0x3E, &keyBytes[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &keyBytes[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Reset the BQ76907
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_Reset(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x12, 0x00}; // RESET() subcommand

    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    HAL_Delay(100);

    return BQ76907_OK;
}

/**
 * @brief Set the BQ76907 to allow configuration updates
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_SetCFGUpdate(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x90, 0x00}; // SET_CFGUPDATE() subcommand

    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    /* Wait for the BQ76907 to enter configuration update mode */
    while(1){
        uint8_t status[2]; 
        BQ76907_ReadRegister(BATTERY_STATUS, status, 2); 
        if(status[0] & (1 << 5)) break;
    }

    return BQ76907_OK;
}

/**
 * @brief Exit configuration update mode
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ExitCFGUpdate(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x92, 0x00}; // EXIT_CFGUPDATE() subcommand

    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    /* Wait for the BQ76907 to exit configuration update mode */
    while(1){
        uint8_t status[2]; 
        BQ76907_ReadRegister(BATTERY_STATUS, status, 2); 
        if(!(status[0] & (1 << 5))) break;
    }

    return BQ76907_OK;
}

/**
 * @brief Configure the power
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ConfigurePower(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr0 = 0x14;
    uint8_t addr1 = 0x90;
    uint8_t data[1] = {0x30};
    uint8_t dataLen = 1;
    uint8_t length = 2 + dataLen + 1 + 1; // 2 address, 1 data, 1 checksum, 1 length = 5
    uint8_t sum = addr0 + addr1 + data[0];
    uint8_t checksum = 0xFF - sum;

    errorCode = BQ76907_WriteRegister(0x3E, &addr0, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr1, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Configure the FET options
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ConfigureFETOptions(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr0 = 0x1E;
    uint8_t addr1 = 0x90;
    uint8_t data[1] = {0x0C};
    uint8_t dataLen = 1;
    uint8_t length = 2 + dataLen + 1 + 1; // 2 address, 1 data, 1 checksum, 1 length = 5
    uint8_t sum = addr0 + addr1 + data[0];
    uint8_t checksum = 0xFF - sum;

    errorCode = BQ76907_WriteRegister(0x3E, &addr0, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr1, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Configure the Vcell mode
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ConfigureVCellMode(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr0 = 0x1B;
    uint8_t addr1 = 0x90;
    uint8_t data[1] = {0x06};
    uint8_t dataLen = 1;
    uint8_t length = 2 + dataLen + 1 + 1; // 2 address, 1 data, 1 checksum, 1 length = 5
    uint8_t sum = addr0 + addr1 + data[0];
    uint8_t checksum = 0xFF - sum;

    errorCode = BQ76907_WriteRegister(0x3E, &addr0, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr1, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}


/**
 * @brief Set the OCC threshold
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_SetOCCThreshold(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr0 = 0x36;
    uint8_t addr1 = 0x90;
    uint8_t data[1] = {0x03};
    uint8_t dataLen = 1;
    uint8_t length = 2 + dataLen + 1 + 1; // 2 address, 1 data, 1 checksum, 1 length = 5
    uint8_t sum = addr0 + addr1 + data[0];
    uint8_t checksum = 0xFF - sum;

    errorCode = BQ76907_WriteRegister(0x3E, &addr0, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr1, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Set the OCD1 threshold
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_SetOCD1Threshold(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr0 = 0x38;
    uint8_t addr1 = 0x90;
    uint8_t data[1] = {0x0C};
    uint8_t dataLen = 1;
    uint8_t length = 2 + dataLen + 1 + 1; // 2 address, 1 data, 1 checksum, 1 length = 5
    uint8_t sum = addr0 + addr1 + data[0];
    uint8_t checksum = 0xFF - sum;

    errorCode = BQ76907_WriteRegister(0x3E, &addr0, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr1, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Set the OCD2 threshold
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_SetOCD2Threshold(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr0 = 0x3A;
    uint8_t addr1 = 0x90;
    uint8_t data[1] = {0x0C};
    uint8_t dataLen = 1;
    uint8_t length = 2 + dataLen + 1 + 1; // 2 address, 1 data, 1 checksum, 1 length = 5
    uint8_t sum = addr0 + addr1 + data[0];
    uint8_t checksum = 0xFF - sum;

    errorCode = BQ76907_WriteRegister(0x3E, &addr0, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr1, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Set the CUV threshold
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_SetCUVThreshold(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr0 = 0x2E;
    uint8_t addr1 = 0x90;
    uint16_t value = 2500;
    uint8_t data[2];
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
    uint8_t dataLen = 2;
    uint8_t length = 2 + dataLen + 1 + 1; // 2 address, 2 data, 1 checksum, 1 length = 6
    uint8_t sum = addr0 + addr1 + data[0] + data[1];
    uint8_t checksum = 0xFF - sum;

    errorCode = BQ76907_WriteRegister(0x3E, &addr0, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr1, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data[0], 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x41, &data[1], 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if (errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Set the CUV protection hysteresis
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_SetCUVProtRecHysterisis(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr0 = 0x31;
    uint8_t addr1 = 0x90;
    uint8_t data[1] = {0x01};
    uint8_t dataLen = 1;
    uint8_t length = 2 + dataLen + 1 + 1; // 2 address, 1 data, 1 checksum, 1 length = 5
    uint8_t sum = addr0 + addr1 + data[0];
    uint8_t checksum = 0xFF - sum;

    errorCode = BQ76907_WriteRegister(0x3E, &addr0, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr1, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data[0], 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if (errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}


/**
 * @brief Set the COV threshold
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_SetCOVThreshold(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr0 = 0x32;
    uint8_t addr1 = 0x90;
    uint16_t value = 4200;
    uint8_t data[2];
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
    uint8_t dataLen = 2;
    uint8_t length = 2 + dataLen + 1 + 1; // 2 address, 2 data, 1 checksum, 1 length = 6
    uint8_t sum = addr0 + addr1 + data[0] + data[1];
    uint8_t checksum = 0xFF - sum;

    errorCode = BQ76907_WriteRegister(0x3E, &addr0, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr1, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data[0], 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x41, &data[1], 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if (errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Set the COV protection hysteresis
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_SetCOVProtRecHysterisis(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr0 = 0x35;
    uint8_t addr1 = 0x90;
    uint8_t data[1] = {0x01};
    uint8_t dataLen = 1;
    uint8_t length = 2 + dataLen + 1 + 1; // 2 address, 1 data, 1 checksum, 1 length = 5
    uint8_t sum = addr0 + addr1 + data[0];
    uint8_t checksum = 0xFF - sum;

    errorCode = BQ76907_WriteRegister(0x3E, &addr0, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr1, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data[0], 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if (errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Enable the thermistor pullup and configure REGOUT_CONTROL
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_EnableThermistorPullup(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t value = 0x1E;

    errorCode = BQ76907_WriteRegister(REGOUT_CONTROL, &value, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Enable protections A
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_EnableProtectionsA(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr0 = 0x24;
    uint8_t addr1 = 0x90;
    uint8_t data[1] = {0xFC};
    uint8_t dataLen = 1;
    uint8_t length = 2 + dataLen + 1 + 1; // 2 address, 1 data, 1 checksum, 1 length = 5
    uint8_t sum = addr0 + addr1 + data[0];
    uint8_t checksum = 0xFF - sum;

    errorCode = BQ76907_WriteRegister(0x3E, &addr0, 1);  
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr1, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data[0], 1);    
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1); 
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);    
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Enable protections B
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_EnableProtectionsB(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr0 = 0x25;
    uint8_t addr1 = 0x90;
    uint8_t data[1] = {0x3E};
    uint8_t dataLen = 1;
    uint8_t length = 2 + dataLen + 1 + 1; // 2 address, 1 data, 1 checksum, 1 length = 5
    uint8_t sum = addr0 + addr1 + data[0];
    uint8_t checksum = 0xFF - sum;

    errorCode = BQ76907_WriteRegister(0x3E, &addr0, 1);   
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr1, 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data[0], 1);    
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);  
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);    
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Enable alarms
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_EnableAlarms(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint16_t data = 0xF000;

    errorCode = BQ76907_WriteRegister(ALARM_ENABLE, (uint8_t *)&data, 2);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Reset the passed charge
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ResetPassedCharge(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x05, 0x00}; // RESET_PASSQ() subcommand

    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Initialize the BQ76907
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_Init(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;

    errorCode = BQ76907_Reset();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_Unseal(0x0414, 0x3672);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_SetCFGUpdate();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ConfigurePower();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ConfigureFETOptions();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ConfigureVCellMode();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_SetOCCThreshold();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_SetOCD1Threshold();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_SetOCD2Threshold();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_SetCUVThreshold();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_SetCUVProtRecHysterisis();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_SetCOVThreshold();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_SetCOVProtRecHysterisis();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_EnableThermistorPullup();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_EnableProtectionsA();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_EnableProtectionsB();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ExitCFGUpdate();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_EnableAlarms();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ResetPassedCharge();
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Read the alarm status
 * @param status Pointer to array to store the status
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ReadAlarmStatus(uint16_t *status){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;

    errorCode = BQ76907_ReadRegister(ALARM_STATUS, (uint8_t*)status, 2);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Clear the alarm status
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ClearAlarmStatus(uint16_t status){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint16_t data = status;

    errorCode = BQ76907_WriteRegister(ALARM_STATUS, (uint8_t*)&data, 2);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Read the safety alert A register
 * @param status Pointer to array to store the status
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ReadSafetyAlertA(uint8_t *status){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;

    errorCode = BQ76907_ReadRegister(SAFETY_ALERT_A, status, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Read the safety alert B register
 * @param status Pointer to array to store the status
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ReadSafetyAlertB(uint8_t *status){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;

    errorCode = BQ76907_ReadRegister(SAFETY_ALERT_B, status, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Read the safety status A register
 * @param status Pointer to array to store the status
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ReadSafetyStatusA(uint8_t *status){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;

    errorCode = BQ76907_ReadRegister(SAFETY_STATUS_A, status, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Read the safety status B register
 * @param status Pointer to array to store the status
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ReadSafetyStatusB(uint8_t *status){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;

    errorCode = BQ76907_ReadRegister(SAFETY_STATUS_B, status, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Enable the protection recovery
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ProtRecovery(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x9B, 0x00};
    uint8_t data[1] = {0x82};
    uint8_t dataLen = 1;
    uint8_t length = 2 + dataLen + 1 + 1; // 2 address, 1 data, 1 checksum, 1 length = 5
    uint8_t sum = subcmd[0] + subcmd[1] + data[0];
    uint8_t checksum = 0xFF - sum;

    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if (errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Handle the alarms
 * @return BQ76907_ErrorCode_t
 */
BQ76907_ErrorCode_t BQ76907_HandleAlarms(void){
	uint8_t data;
    uint16_t alarms;
	BQ76907_ErrorCode_t errorCode = BQ76907_ReadAlarmStatus(&alarms);
	if(errorCode != BQ76907_OK){
    return errorCode;
}

	if(alarms & 0x1000){
		errorCode = BQ76907_ReadSafetyAlertB(&data);
		if(errorCode != BQ76907_OK){
			return errorCode;
		}
//		errorCode = BQ76907_ClearAlarmStatus(0x1000);
//			if(errorCode != BQ76907_OK){
//				return errorCode;
//		}
		switch(data){
			case 0x08: return BQ76907_ERROR_INT_OVERTEMPERATURE;
			case 0x10: return BQ76907_ERROR_UNDERTEMPERATURE_CHARGE;
			case 0x20: return BQ76907_ERROR_UNDERTEMPERATURE_DISCHARGE;
			case 0x40: return BQ76907_ERROR_OVERTEMPERATURE_CHARGE;
			case 0x80: return BQ76907_ERROR_OVERTEMPERATURE_DISCHARGE;
		}
	}
	if(alarms & 0x2000){
		errorCode = BQ76907_ReadSafetyAlertA(&data);
		if(errorCode != BQ76907_OK) {
			return errorCode;
		}
//		errorCode = BQ76907_ClearAlarmStatus(0x2000);
//			if(errorCode != BQ76907_OK){
//				return errorCode;
//		}
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
		errorCode = BQ76907_ReadSafetyStatusB(&data);
		if(errorCode != BQ76907_OK){
			return errorCode;
		}
//		errorCode = BQ76907_ClearAlarmStatus(0x4000);
//			if(errorCode != BQ76907_OK){
//				return errorCode;
//		}
		switch(data){
			case 0x08: return BQ76907_ERROR_INT_OVERTEMPERATURE;
			case 0x10: return BQ76907_ERROR_UNDERTEMPERATURE_CHARGE;
			case 0x20: return BQ76907_ERROR_UNDERTEMPERATURE_DISCHARGE;
			case 0x40: return BQ76907_ERROR_OVERTEMPERATURE_CHARGE;
			case 0x80: return BQ76907_ERROR_OVERTEMPERATURE_DISCHARGE;
		}
	}
	if(alarms & 0x8000){
		errorCode = BQ76907_ReadSafetyStatusA(&data);
		if(errorCode != BQ76907_OK) {
			return errorCode;
		}
//		errorCode = BQ76907_ClearAlarmStatus(0x8000);
//			if(errorCode != BQ76907_OK){
//				return errorCode;
//		}
		switch(data){
			case 0x04: return BQ76907_ERROR_OVERCURRENT_CHARGE;
			case 0x08: return BQ76907_ERROR_OVERCURRENT_DISCHARGE_2;
			case 0x10: return BQ76907_ERROR_OVERCURRENT_DISCHARGE_1;
			case 0x20: return BQ76907_ERROR_SHORT_CIRCUIT_DISCHARGE;
			case 0x40: return BQ76907_ERROR_CELL_UNDERVOLTAGE;
			case 0x80: return BQ76907_ERROR_CELL_OVERVOLTAGE;
		}
	}

    return errorCode;
}

/**
 * @brief Read the passed charge and time
 * @param charge Pointer to store the accumulated charge (48-bit signed value in userA-seconds)
 * @param time Pointer to store the accumulated time (32-bit unsigned value in 250ms units)
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_ReadPassedCharge(int64_t *charge, uint32_t *time){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x04, 0x00}; // PASSQ() subcommand
    uint8_t data[12];

    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ReadRegister(0x40, data, 12);
    if(errorCode != BQ76907_OK) return errorCode;

    // Extract accumulated charge (48-bit signed value)
    // First 32 bits are the lower bits
    uint32_t chargeLower = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | 
                          ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    // Upper 16 bits are sign-extended to 32 bits
    int32_t chargeUpper = (int32_t)((int16_t)(data[4] | (data[5] << 8)));
    
    // Combine into 64-bit signed value
    *charge = ((int64_t)chargeUpper << 32) | chargeLower;

    // Extract accumulated time (32-bit unsigned, in 250ms units)
    *time = (uint32_t)data[8] | ((uint32_t)data[9] << 8) | 
            ((uint32_t)data[10] << 16) | ((uint32_t)data[11] << 24);

    return BQ76907_OK;
}

/**
 * @brief Check if the battery is charging
 * @param isCharging Pointer to store the charging status (true if charging, false if discharging)
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_IsCharging(bool *isCharging){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    int64_t charge;
    uint32_t time;
    
    // Read the passed charge and time
    errorCode = BQ76907_ReadPassedCharge(&charge, &time);
    if(errorCode != BQ76907_OK) return errorCode;
    
    // Calculate current from charge and time
    // charge is in userA-seconds, time is in 250ms units
    // Convert time to seconds: time * 0.25
    float timeSeconds = (float)time * 0.25f;
    
    if(timeSeconds > 0.0f) {
        // Current = charge / time (in amperes)
        float current = (float)charge / timeSeconds;
        
        // Positive current means charging, negative means discharging
        *isCharging = (current > 0.0f);
    } else {
        // If no time has passed, assume not charging
        *isCharging = false;
    }
    
    return BQ76907_OK;
}

/**
 * @brief Enable cell balancing for specified cells
 * @param cellMask Bit mask of cells to balance
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_EnableCellBalancing(uint8_t cellMask){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x83, 0x00}; // CB_ACTIVE_CELLS() subcommand
    uint8_t data[1] = {cellMask};
    uint8_t dataLen = 1;
    uint8_t length = 2 + dataLen + 1 + 1; // 2 address, 1 data, 1 checksum, 1 length = 5
    uint8_t sum = subcmd[0] + subcmd[1] + data[0];
    uint8_t checksum = 0xFF - sum;

    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if (errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Disable cell balancing
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_DisableCellBalancing(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x83, 0x00}; // CB_ACTIVE_CELLS() subcommand
    uint8_t data[1] = {0x00};
    uint8_t dataLen = 1;
    uint8_t length = 2 + dataLen + 1 + 1; // 2 address, 1 data, 1 checksum, 1 length = 5
    uint8_t sum = subcmd[0] + subcmd[1] + data[0];
    uint8_t checksum = 0xFF - sum;

    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if (errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}


/**
 * @brief Get the current cell balancing status
 * @param activeCells Pointer to store the bit mask of actively balanced cells
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_GetCellBalancingStatus(uint8_t *activeCells){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x83, 0x00}; // CB_ACTIVE_CELLS() subcommand for reading

    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ReadRegister(0x40, activeCells, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Manages the cell balancing process based on battery state.
 * @details This function should be called periodically (e.g., every second). It manages 
 *          the balancing of the cells to equalize the pack. BQ76907 only allows 2 cells 
 *          to be balanced at once. Balances the 2 highest voltage cells until there's 
 *          a 20mV difference between min and max.
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_ManageCellBalancing(void){
    BQ76907_ErrorCode_t errorCode;
    uint16_t cellVoltages[NUM_CELLS];
    uint8_t balanceMask = 0x00;
    uint16_t minVoltage = 0xFFFF;
    uint16_t maxVoltage = 0x0000;
    int8_t topIndices[2] = {-1, -1};

    errorCode = BQ76907_ReadCellVoltages(cellVoltages);
    if(errorCode != BQ76907_OK) return errorCode;

    // Find min and max voltages
    for(uint8_t i = 0; i < NUM_CELLS; i++){
        if(cellVoltages[i] < minVoltage) minVoltage = cellVoltages[i];
        if(cellVoltages[i] > maxVoltage) maxVoltage = cellVoltages[i];
    }

    // Check if balancing is needed (difference > 20mV)
    if((maxVoltage - minVoltage) <= 20) {
        // Voltage difference is small enough, disable balancing
        return BQ76907_DisableCellBalancing();
    }

    // Find indices of the two highest voltage cells
    // First, find the highest
    for(uint8_t i = 0; i < NUM_CELLS; i++){
        if(topIndices[0] == -1 || cellVoltages[i] > cellVoltages[topIndices[0]]){
            topIndices[1] = topIndices[0];
            topIndices[0] = i;
        } else if(topIndices[1] == -1 || cellVoltages[i] > cellVoltages[topIndices[1]]){
            if(i != topIndices[0]) // avoid duplicate index
                topIndices[1] = i;
        }
    }

    // If only one cell found (all voltages equal), just balance that one
    if(topIndices[1] == -1) topIndices[1] = topIndices[0];

    // Create balance mask for the 2 highest voltage cells
    // NOTE: The BQ76907 cell balancing mask is 1-indexed (Bit 1 = Cell 1),
    // and Bit 0 is reserved.
    for(uint8_t i = 0; i < 2; i++){
        uint8_t bitToSet = topIndices[i] + 1; // Convert to 1-indexed
        // Skip setting bit 0 as it's reserved
        if(bitToSet != 0) {
            balanceMask |= (1 << bitToSet);
        }
    }

    return BQ76907_EnableCellBalancing(balanceMask);
}

/**
 * @brief Calculate SoC from voltage using linear voltage-to-SoC mapping
 * @param cellVoltage Cell voltage in millivolts
 * @return SoC percentage (0-100)
 */
static float BQ76907_VoltageToSoC(uint16_t cellVoltage){
    // Linear voltage-to-SoC mapping for Li-ion batteries
    
    if(cellVoltage >= socVoltageFull) return 100.0f;
    if(cellVoltage <= socVoltageEmpty) return 0.0f;
    
    // Linear mapping: SoC = (V_cell - V_empty) / (V_full - V_empty) * 100
    float voltageRange = (float)(socVoltageFull - socVoltageEmpty);
    float voltageRatio = (float)(cellVoltage - socVoltageEmpty) / voltageRange;
    
    return voltageRatio * 100.0f;
}

/**
 * @brief Calculate initial SoC based on average cell voltage
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_SoCInit(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint16_t cellVoltages[NUM_CELLS];
    float totalVoltage = 0.0f;
    float avgCellVoltage = 0.0f;
    
    // Read all cell voltages
    errorCode = BQ76907_ReadCellVoltages(cellVoltages);
    if(errorCode != BQ76907_OK) return errorCode;
    
    // Calculate total pack voltage
    for(uint8_t i = 0; i < NUM_CELLS; i++){
        totalVoltage += (float)cellVoltages[i];
    }
    
    // Calculate average cell voltage
    avgCellVoltage = totalVoltage / NUM_CELLS;
    
    // Calculate SoC based on voltage using linear mapping
    float voltageSoC = BQ76907_VoltageToSoC((uint16_t)avgCellVoltage);
    
    // Set initial SoC
    initialSoC = voltageSoC;
    SoC = initialSoC;
    socInitialized = true;
    
    // Reset charge integration for coulomb counting
    lastCharge = 0;
    lastTime = 0;
    
    return BQ76907_OK;
}

/**
 * @brief Update SoC using coulomb counting from passed charge
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_SoCUpdate(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    int64_t currentCharge;
    uint32_t currentTime;
    float chargeDelta = 0.0f;
    float timeDelta = 0.0f;
    
    // Only update if SoC has been initialized
    if(!socInitialized){
        return BQ76907_SoCInit();
    }
    
    // Read current passed charge and time
    errorCode = BQ76907_ReadPassedCharge(&currentCharge, &currentTime);
    if(errorCode != BQ76907_OK) return errorCode;
    
    // Calculate deltas (charge difference since last reading)
    chargeDelta = (float)(currentCharge - lastCharge);
    timeDelta = (float)(currentTime - lastTime) * 0.25f; // Convert 250ms units to seconds
    
    // Only update if we have meaningful time difference
    if(timeDelta > 0.0f) {
        // Convert charge from userA-seconds to mAh
        // userA-seconds = micro-ampere-seconds = 1e-6 A * s
        // mAh = (charge in A*s) / 3600 * 1000
        float chargeMah = (chargeDelta * 1e-6f) / 3600.0f * 1000.0f;
        
        // Update SoC: SoC = SoC_initial + (charge_mAh / capacity_mAh) * 100
        // Note: In series configuration, pack capacity = single cell capacity
        //       Positive charge means charging (SoC increases)
        //       Negative charge means discharging (SoC decreases)
        float newSoC = initialSoC + (chargeMah / socCapacityMah) * 100.0f;
        
        // Clamp SoC between 0% and 100%
        if(newSoC < 0.0f) newSoC = 0.0f;
        if(newSoC > 100.0f) newSoC = 100.0f;
        
        // Update SoC directly
        SoC = newSoC;
    }
    
    // Update last values for next calculation
    lastCharge = currentCharge;
    lastTime = currentTime;
    
    return BQ76907_OK;
}

/**
 * @brief Get the State of Charge (SoC) of the battery pack
 * @return The SoC of the battery pack as a percentage
 */
uint8_t BQ76907_GetSoC(void){
    return (uint8_t)roundf(SoC);

}

