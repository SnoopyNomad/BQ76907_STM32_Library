/**
 * @file bq76907.c
 * @brief BQ76907 battery management system driver
 * @details This file contains the implementation of the
 *          BQ76907 battery management system driver, supporting various
 *          operations.
 * @author Cengiz Sinan Kostakoglu
 * @version 1.0
 * @date 2025-06-13
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

/* Battery Constants */
#define NUM_CELLS               6
#define BATTERY_CAPACITY_MAH    2800

/* State of Charge Variables */
static float SoC = 100.0f;
static float SoCInit = 100.0f;

/* I2C Handle */
extern I2C_HandleTypeDef hi2c1;

/**
 * @brief Map HAL error codes to BQ76907 error codes
 * @param status HAL error code
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_MapError(HAL_StatusTypeDef status){
    if(status == HAL_OK) return BQ76907_OK;
    else if(status == HAL_ERROR) return BQ76907_ERROR_I2C;
    else if(status == HAL_BUSY) return BQ76907_ERROR_BUSY;
    else return BQ76907_ERROR_I2C;
}

/**
 * @brief Write a single byte to a BQ76907 register via I2C
 * @param reg Register address to write to
 * @param data Pointer to data buffer to write from
 * @param length Number of bytes to write
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_WriteRegister(uint8_t reg, uint8_t *data, uint8_t length){
    return BQ76907_MapError(HAL_I2C_Mem_Write(&hi2c1, (BQ76907_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY));
}

/**
 * @brief Read a single byte from a BQ76907 register via I2C
 * @param reg Register address to read from
 * @param data Pointer to data buffer to read into
 * @param length Number of bytes to read
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ReadRegister(uint8_t reg, uint8_t *data, uint8_t length){
    return BQ76907_MapError(HAL_I2C_Mem_Read(&hi2c1, (BQ76907_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY));
}

/**
 * @brief Read the battery status from the BQ76907
 * @param status Pointer to 2-byte array to store the status (status[0] = status, status[1] = flags)
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_ReadBatteryStatus(uint16_t *status){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    errorCode = BQ76907_ReadRegister(BATTERY_STATUS, (uint8_t*)status, 2);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}   

/**
 * @brief Read the security keys from the BQ76907
 * @param keys Pointer to 4-byte array to store the keys (keys[0,1] = key1 LSB,MSB, keys[2,3] = key2 LSB,MSB)
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_ReadKeys(uint8_t *keys) {
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x35, 0x00}; // SECURITY_KEYS subcommand

    // 1. Write 0x35 to 0x3E
    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // 2. Write 0x00 to 0x3F
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // 3. Read 4 bytes from 0x40–0x43
    errorCode = BQ76907_ReadRegister(0x40, keys, 4);
    if(errorCode != BQ76907_OK) return errorCode;

    // keys[0,1] = key1 (LSB, MSB), keys[2,3] = key2 (LSB, MSB)
    return BQ76907_OK;
}

/**
 * @brief Write the security keys to the BQ76907
 * @param keys Pointer to 4-byte array containing the keys (keys[0,1] = key1 LSB,MSB, keys[2,3] = key2 LSB,MSB)
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_WriteKeys(uint8_t *keys){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x35, 0x00}; // SECURITY_KEYS subcommand

    // 1. Write 0x35 to 0x3E
    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // 2. Write 0x00 to 0x3F
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // 3. Write 4 bytes to 0x40–0x43
    errorCode = BQ76907_WriteRegister(0x40, keys, 4);   
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Seal the BQ76907 (return to protected mode)
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_Seal(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x30, 0x00}; // SEAL() subcommand

    // 1. Write 0x30 to 0x3E
    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // 2. Write 0x00 to 0x3F
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // No further action needed for SEAL()
    return BQ76907_OK;
}

/**
 * @brief Unseal the BQ76907 with keys (exit protected mode) using the legacy method
 * @param key1 16-bit key1
 * @param key2 16-bit key2
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_Unseal(uint16_t key1, uint16_t key2) {
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t keyBytes[2];

    // Write key1 (LSB to 0x3E, MSB to 0x3F)
    keyBytes[0] = key1 & 0xFF;         // LSB
    keyBytes[1] = (key1 >> 8) & 0xFF;  // MSB
    errorCode = BQ76907_WriteRegister(0x3E, &keyBytes[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &keyBytes[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // Write key2 (LSB to 0x3E, MSB to 0x3F)
    keyBytes[0] = key2 & 0xFF;         // LSB
    keyBytes[1] = (key2 >> 8) & 0xFF;  // MSB
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
BQ76907_ErrorCode_t BQ76907_Reset(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x12, 0x00}; // RESET() subcommand

    // 1. Write 0x12 to 0x3E    
    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // 2. Write 0x00 to 0x3F
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    
    // 3. Wait for the BQ76907 to reset 
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

    // 1. Write 0x90 to 0x3E
    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // 2. Write 0x00 to 0x3F
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // Wait for the BQ76907 to enter configuration update mode
    while(1){
        uint8_t status[2]; 
        BQ76907_ReadRegister(0x12, status, 2); 
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
    
    // 1. Write 0x91 to 0x3E    
    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // 2. Write 0x00 to 0x3F
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // Wait for the BQ76907 to exit configuration update mode
    while(1){
        uint8_t status[2]; 
        BQ76907_ReadRegister(0x12, status, 2); 
        if(!(status[0] & (1 << 5))) break;
    }
    return BQ76907_OK;
}

/**
 * @brief Disable sleep mode
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_SleepDisable(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subCmd[2] = {0x9A, 0x00}; // SLEEP_DISABLE() subcommand
    
    // 1. Write 0x9A to 0x3E
    errorCode = BQ76907_WriteRegister(0x3E, &subCmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    // 2. Write 0x00 to 0x3F
    errorCode = BQ76907_WriteRegister(0x3F, &subCmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Disable sleep mode
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ConfigureSleep(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subCmd[2] = {0x14, 0x90};
    uint8_t data = 0x00;

    errorCode = BQ76907_WriteRegister(0x3E, &subCmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subCmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Enter deep sleep mode
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_EnterDeepSleep(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subCmd[2] = {0x0F, 0x00}; // DEEPSLEEP() subcommand

    errorCode = BQ76907_WriteRegister(0x3E, &subCmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subCmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    HAL_Delay(100);

    errorCode = BQ76907_WriteRegister(0x3E, &subCmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subCmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Exit deep sleep mode
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_ExitDeepSleep(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subCmd[2] = {0x0E, 0x00}; // EXIT_DEEPSLEEP() subcommand

    errorCode = BQ76907_WriteRegister(0x3E, &subCmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subCmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Configure the FET options
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ConfigureFETOptions(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr[2] = {0x1E, 0x90};
    uint8_t value = 0x6C;
    uint8_t checksum = ~(addr[0] + addr[1] + value);
    uint8_t length = 5;

    errorCode = BQ76907_WriteRegister(0x3E, &addr[0], 1);   
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &value, 1);     
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
    uint8_t addr[2] = {0x36, 0x90};
    uint8_t value = 0x02; // 4 A
    uint8_t checksum = ~(addr[0] + addr[1] + value);
    uint8_t length = 5;

    errorCode = BQ76907_WriteRegister(0x3E, &addr[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &value, 1);
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
    uint8_t addr[2] = {0x38, 0x90};
    uint8_t value = 0x0C; // 24 A 0x0C
    uint8_t checksum = ~(addr[0] + addr[1] + value);
    uint8_t length = 5;

    errorCode = BQ76907_WriteRegister(0x3E, &addr[0], 1);   
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &value, 1);    
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
    uint8_t addr[2] = {0x3A, 0x90};
    uint8_t value = 0x0C; // 24 A
    uint8_t checksum = ~(addr[0] + addr[1] + value);
    uint8_t length = 5;

    errorCode = BQ76907_WriteRegister(0x3E, &addr[0], 1);   
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &value, 1);    
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);  
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);    
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Enable the thermistor pullup and configure REGOUT_CONTROL
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_EnableThermistorPullup(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t value = 0x1E;

    errorCode = BQ76907_WriteRegister(REGOUT_CONTROL, &value, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Set the CUV threshold
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_SetCUVThreshold(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr[2] = {0x2E, 0x90};
    uint16_t value = 3000;   
    uint8_t data[2];
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
    uint8_t checksum = ~(data[0] + data[1]);
    uint8_t length = 2;

    errorCode = BQ76907_WriteRegister(0x3E, &addr[0], 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr[1], 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, data, 2);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if (errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Read the CUV threshold
 * @param value Pointer to array to store the value
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_ReadCUVThreshold(uint16_t *value){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr[2] = {0x2E, 0x90};
    uint8_t data[2];
    uint8_t checksum = ~(data[0] + data[1]);
    uint8_t length = 2;

    errorCode = BQ76907_WriteRegister(0x3E, &addr[0], 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr[1], 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ReadRegister(0x40, data, 2);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ReadRegister(0x60, &checksum, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ReadRegister(0x61, &length, 1);
    if (errorCode != BQ76907_OK) return errorCode;

    *value = (data[1] << 8) | data[0];

    return BQ76907_OK;
}

/**
 * @brief Set the CUV recovery hysterisis
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_SetCUVRecoveryHysterisis(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr[2] = {0x31, 0x90};
    uint8_t data = 0x01;
    uint8_t checksum = ~(data);
    uint8_t length = 1;

    errorCode = BQ76907_WriteRegister(0x3E, &addr[0], 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr[1], 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &data, 2);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);
    if (errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Read the CUV recovery hysterisis
 * @param value Pointer to array to store the value
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_ReadCUVRecoveryHysterisis(uint8_t *value){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr[2] = {0x31, 0x90};

    errorCode = BQ76907_WriteRegister(0x3E, &addr[0], 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr[1], 1);
    if (errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ReadRegister(0x40, value, 1);
    if (errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Enable protections A
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_EnableProtectionsA(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t addr[2] = {0x24, 0x90};
    uint8_t value = 0xFC;
    uint8_t checksum = ~(addr[0] + addr[1] + value);
    uint8_t length = 5;

    errorCode = BQ76907_WriteRegister(0x3E, &addr[0], 1);  
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &value, 1);    
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
    uint8_t addr[2] = {0x25, 0x90};
    uint8_t value = 0x3E;
    uint8_t checksum = ~(addr[0] + addr[1] + value);
    uint8_t length = 5;

    errorCode = BQ76907_WriteRegister(0x3E, &addr[0], 1);   
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &addr[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &value, 1);    
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x60, &checksum, 1);  
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x61, &length, 1);    
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Initialize the BQ76907
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_Init(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    errorCode = BQ76907_ExitDeepSleep();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_Reset();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_Unseal(0x0414, 0x3672);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ExitDeepSleep();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_SleepDisable();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_SetCFGUpdate();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ConfigureSleep();
	if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ConfigureFETOptions();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_SetCUVThreshold();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_SetCUVRecoveryHysterisis();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_SetOCCThreshold();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_SetOCD1Threshold();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_SetOCD2Threshold();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_EnableThermistorPullup();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_EnableProtectionsA();   
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_EnableProtectionsB();
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ExitCFGUpdate();
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK; 
}

/**
 * @brief Turn off the DSG FET
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_DSGFETOff(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK; 
    uint8_t data = 0x04; // DSG_OFF

    errorCode = BQ76907_WriteRegister(FET_CONTROL, &data, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Turn off the CHG FET
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_CHGFETOff(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t data = 0x08; // CHG_OFF

    errorCode = BQ76907_WriteRegister(FET_CONTROL, &data, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}


/**
 * @brief Turn on the DSG FET
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_DSGFETOn(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t data = 0x01; // DSG_ON

    errorCode = BQ76907_WriteRegister(FET_CONTROL, &data, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Turn on the CHG FET
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_CHGFETOn(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t data = 0x02; // CHG_ON

    errorCode = BQ76907_WriteRegister(FET_CONTROL, &data, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Read the cell voltages
 * @param voltages Pointer to array to store the voltages
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_ReadCellVoltages(uint16_t *voltages){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;

    errorCode = BQ76907_ReadRegister(CELL_1_VOLTAGE, (uint8_t*)voltages, NUM_CELLS * 2);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;  
}

/**
 * @brief Convert the ADC code to temperature
 * @param adcCode ADC code
 * @return Temperature in Celsius
 */
static inline float TSADCtoTemperature(uint16_t adcCode){
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
    return tempC;
}

/**
 * @brief Read the battery temperature
 * @param temp Pointer to array to store the temperature
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_ReadBatteryTemperature(float *temp){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint16_t adcCode;

    errorCode = BQ76907_ReadRegister(TS_MEASUREMENT, (uint8_t*)&adcCode, 2);
    if(errorCode != BQ76907_OK) return errorCode;

    *temp = TSADCtoTemperature(adcCode);

    return BQ76907_OK;
}

/**
 * @brief Enable alarms
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_EnableAlarms(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint16_t data = 0xF000;

    errorCode = BQ76907_WriteRegister(ALARM_ENABLE, (uint8_t*)&data, 2);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Read the alarm status
 * @param status Pointer to array to store the status
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */ 
BQ76907_ErrorCode_t BQ76907_ReadAlarmStatus(uint16_t *status){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;

    errorCode = BQ76907_ReadRegister(ALARM_STATUS, (uint8_t*)status, 2);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Clear the alarm status
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_ClearAlarmStatus(uint16_t status){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK; 
    uint16_t data = status;

    errorCode = BQ76907_WriteRegister(ALARM_STATUS, (uint8_t*)&data, 2);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Read the safety status A register
 * @param status Pointer to array to store the status
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */ 
BQ76907_ErrorCode_t BQ76907_ReadSafetyStatusA(uint8_t *status){
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
BQ76907_ErrorCode_t BQ76907_ReadSafetyStatusB(uint8_t *status){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;

    errorCode = BQ76907_ReadRegister(SAFETY_STATUS_B, status, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}   

/**
 * @brief Read the safety alert A register
 * @param status Pointer to array to store the status
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */     
BQ76907_ErrorCode_t BQ76907_ReadSafetyAlertA(uint8_t *status){
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
BQ76907_ErrorCode_t BQ76907_ReadSafetyAlertB(uint8_t *status){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;

    errorCode = BQ76907_ReadRegister(SAFETY_ALERT_B, status, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Enable cell balance
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_CellBalance(uint8_t activeCells){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subCmd[2] = {0x83, 0x00}; // CB_ACTIVE_CELLS() subcommand

    errorCode = BQ76907_WriteRegister(0x3E, &subCmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subCmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x40, &activeCells, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Balance the minimum and maximum cells
 * @param cellVoltages Pointer to array to store the cell voltages
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_BalanceMinMaxCells(uint16_t cellVoltages[6]){
    uint8_t minIndex = 0, maxIndex = 0;
    for (uint8_t i = 1; i < NUM_CELLS; i++){
        if(cellVoltages[i] < cellVoltages[minIndex]) minIndex = i;
        if(cellVoltages[i] > cellVoltages[maxIndex]) maxIndex = i;
    }

    uint8_t mask = 0;
    mask |= (1 << (minIndex + 1));
    mask |= (1 << (maxIndex + 1));

    return BQ76907_CellBalance(mask);
}

/**
 * @brief Read the active cells
 * @param activeCells Pointer to array to store the active cells
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_ReadCBActiveCells(uint8_t *activeCells){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subCmd[2] = {0x83, 0x00}; // CB_ACTIVE_CELLS() subcommand

    errorCode = BQ76907_WriteRegister(0x3E, &subCmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subCmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ReadRegister(0x40, activeCells, 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}

/**
 * @brief Reads all cell voltages and returns the pack voltage.
 * @param packVoltage Pointer to store the total pack voltage (in volts)
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ReadPackVoltage(float *packVoltage){
    uint16_t cellVoltages[NUM_CELLS];
    BQ76907_ErrorCode_t errorCode = BQ76907_ReadCellVoltages(cellVoltages);
    if(errorCode != BQ76907_OK) return errorCode;
  
    float sum = 0.0f;
    for(uint8_t i = 0; i < NUM_CELLS; i++){
        sum += cellVoltages[i];
    }

    *packVoltage = sum;

    return BQ76907_OK;
}

/**
 * @brief Read the pass Q and elapsed time
 * @param passQ Pointer to store the pass Q
 * @param elapsedTime Pointer to store the elapsed time
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ReadPassQ(int64_t *passQ, float *elapsedTime){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x04, 0x00}; // PASSQ() subcommand
    uint8_t data[12];

    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_ReadRegister(0x40, data, 12);
    if(errorCode != BQ76907_OK) return errorCode;

    // Lower 32 bits (LSB)
    int32_t passqLsb = (int32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    // Upper 16 bits, sign-extended to 32 bits
    int32_t passqMsb = (int32_t)(data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));
    // Combine to 48-bit signed value
    int64_t passq = ((int64_t)passqMsb << 32) | ((uint32_t)passqLsb);

    // Timer (in 250 ms units)
    uint32_t timer = (uint32_t)(data[8] | (data[9] << 8) | (data[10] << 16) | (data[11] << 24));
    float elapsedSeconds = timer * 0.25f;

    *passQ = passq;
    *elapsedTime = elapsedSeconds;

    return BQ76907_OK;
}

/**
 * @brief Resets the charge integrator and timer.
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_ResetPassQ(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_OK;
    uint8_t subcmd[2] = {0x05, 0x00}; // RESET_PASSQ() subcommand

    errorCode = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(errorCode != BQ76907_OK) return errorCode;
    errorCode = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(errorCode != BQ76907_OK) return errorCode;

    return BQ76907_OK;
}   

/** 
 * @brief Lookup the state of charge (SOC) from the pack voltage
 * @param voltage Pack voltage (in volts)
 * @return State of charge (0.0 to 100.0)
 */
static inline float lookupSOCFromOCV(float voltage){
    return ((voltage - 18.0f) / (25.2f - 18.0f)) * 100.0f;
}

/**
 * @brief Calculate the initial state of charge (SOC)
 * @param soc Pointer to store the state of charge
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_CalculateInitialSoC(float *soc){
    float packVoltage;
    BQ76907_ErrorCode_t errorCode = BQ76907_ReadPackVoltage(&packVoltage);
    if(errorCode != BQ76907_OK) return errorCode;

    *soc = lookupSOCFromOCV(packVoltage);

    // Check if the calculated SOC is out of range
    if(*soc < 0.0f) *soc = 0.0f;
    if(*soc > 100.0f) *soc = 100.0f;

    // Reset the pass Q
    errorCode = BQ76907_ResetPassQ();
    if(errorCode != BQ76907_OK) return errorCode;
    
    return BQ76907_OK;
}   

/**
 * @brief Update SOC using coulomb counting (PASSQ)
 * @param socInit Initial SOC (in percent, from voltage)
 * @param socPercent Pointer to store the updated SOC in percent
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
static inline BQ76907_ErrorCode_t BQ76907_UpdateSoCWithPassQ(float socInit, float *socPercent){
    int64_t passQ;
    float elapsedTime;
    BQ76907_ErrorCode_t errorCode = BQ76907_ReadPassQ(&passQ, &elapsedTime);
    if(errorCode != BQ76907_OK) return errorCode;

    // Convert passQ mA-seconds to mAh
    float QmAh = (float)passQ / 3600.0f;

    // Update SOC (in percent)
    *socPercent = socInit - (QmAh / BATTERY_CAPACITY_MAH) * 100.0f;

    // Clamp to 0–100%
    if(*socPercent > 100.0f) *socPercent = 100.0f;
    if(*socPercent < 0.0f) *socPercent = 0.0f;

    return BQ76907_OK;
}

/**
 * @brief Initialize the state of charge (SOC)
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_SoCInit(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_CalculateInitialSoC(&SoCInit);
    if(errorCode != BQ76907_OK) return errorCode;
    return BQ76907_OK;
}

/**
 * @brief Update the state of charge (SOC)
 * @return BQ76907_ErrorCode_t (BQ76907_OK on success)
 */
BQ76907_ErrorCode_t BQ76907_SoCUpdate(void){
    BQ76907_ErrorCode_t errorCode = BQ76907_UpdateSoCWithPassQ(SoCInit, &SoC);
    if(errorCode != BQ76907_OK) return errorCode;
    return BQ76907_OK;
}

/**
 * @brief Get the state of charge (SOC)
 * @return float (state of charge)
 */
float BQ76907_SoCGet(void){
    return SoC;
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
		errorCode = BQ76907_ClearAlarmStatus(0x1000);
		if(errorCode != BQ76907_OK){
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
		errorCode = BQ76907_ReadSafetyAlertA(&data);
		if(errorCode != BQ76907_OK) {
			return errorCode;
		}
		errorCode = BQ76907_ClearAlarmStatus(0x2000);
		if(errorCode != BQ76907_OK){
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
		errorCode = BQ76907_ReadSafetyStatusB(&data);
		if(errorCode != BQ76907_OK) {
			return errorCode;
		}
		errorCode = BQ76907_ClearAlarmStatus(0x4000);
		if(errorCode != BQ76907_OK){
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
		errorCode = BQ76907_ReadSafetyStatusA(&data);
		if(errorCode != BQ76907_OK) {
			return errorCode;
		}
		errorCode = BQ76907_ClearAlarmStatus(0x8000);
		if(errorCode != BQ76907_OK){
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

	return errorCode;
}

