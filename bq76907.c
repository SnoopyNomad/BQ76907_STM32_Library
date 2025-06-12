/**
 * @file bq76907.c
 * @brief BQ76907 battery management system IC driver implementation
 * @details This file contains the implementation of the BQ76907 battery management system IC driver,
 *          supporting various battery protection features and configuration options.
 * @author Cengiz Sinan Kostakoglu
 * @version 1.0
 * @date 2025-06-12
 */

#include "bq76907.h"

/* BQ76907 I2C Address */
#define BQ76907_ADDR		0x08

/* Register addresses */
#define BATTERY_STATUS	    0x12
#define CELL_1_VOLTAGE	    0x14
#define CELL_2_VOLTAGE	    0x16
#define CELL_3_VOLTAGE	    0x18
#define CELL_4_VOLTAGE	    0x1A
#define CELL_5_VOLTAGE	    0x1C
#define CELL_6_VOLTAGE	    0x1E
#define CELL_7_VOLTAGE	    0x20
#define TS_MEASUREMENT	    0x2A
#define ALARM_STATUS        0x62
#define ALARM_RAW_STATUS    0x64
#define ALARM_ENABLE	    0x66
#define FET_CONTROL	        0x68
#define REGOUT_CONTROL	    0x69

#define NUM_CELLS           6

extern I2C_HandleTypeDef hi2c1;

/**
 * @brief Write a single byte to a BQ76907 register via I2C
 * @param reg Register address to write to
 * @param data Pointer to data buffer to write from
 * @param length Number of bytes to write
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
static inline HAL_StatusTypeDef BQ76907_WriteRegister(uint8_t reg, uint8_t *data, uint8_t length){
    return HAL_I2C_Mem_Write(&hi2c1, (BQ76907_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

/**
 * @brief Read a single byte from a BQ76907 register via I2C
 * @param reg Register address to read from
 * @param data Pointer to data buffer to read into
 * @param length Number of bytes to read
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
static inline HAL_StatusTypeDef BQ76907_ReadRegister(uint8_t reg, uint8_t *data, uint8_t length){
    return HAL_I2C_Mem_Read(&hi2c1, (BQ76907_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

/**
 * @brief Read the battery status from the BQ76907
 * @param status Pointer to 2-byte array to store the status (status[0] = status, status[1] = flags)
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_ReadBatteryStatus(uint16_t *status){
    HAL_StatusTypeDef status = HAL_OK;
    status = BQ76907_ReadRegister(BATTERY_STATUS, (uint8_t*)status, 2);
    if(status != HAL_OK) return status;

    return status;
}   

/**
 * @brief Read the security keys from the BQ76907
 * @param keys Pointer to 4-byte array to store the keys (keys[0,1] = key1 LSB,MSB, keys[2,3] = key2 LSB,MSB)
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
static inline HAL_StatusTypeDef BQ76907_ReadKeys(uint8_t *keys) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t subcmd[2] = {0x35, 0x00}; // SECURITY_KEYS subcommand

    // 1. Write 0x35 to 0x3E
    status = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(status != HAL_OK) return status;

    // 2. Write 0x00 to 0x3F
    status = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(status != HAL_OK) return status;

    // 3. Read 4 bytes from 0x40–0x43
    status = BQ76907_ReadRegister(0x40, keys, 4);
    if(status != HAL_OK) return status;

    // keys[0,1] = key1 (LSB, MSB), keys[2,3] = key2 (LSB, MSB)
    return status;
}

/**
 * @brief Write the security keys to the BQ76907
 * @param keys Pointer to 4-byte array containing the keys (keys[0,1] = key1 LSB,MSB, keys[2,3] = key2 LSB,MSB)
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
static inline HAL_StatusTypeDef BQ76907_WriteKeys(uint8_t *keys){
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t subcmd[2] = {0x35, 0x00}; // SECURITY_KEYS subcommand

    // 1. Write 0x35 to 0x3E
    status = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(status != HAL_OK) return status;

    // 2. Write 0x00 to 0x3F
    status = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(status != HAL_OK) return status;

    // 3. Write 4 bytes to 0x40–0x43
    status = BQ76907_WriteRegister(0x40, keys, 4);   
    if(status != HAL_OK) return status;

    return status;
}

/**
 * @brief Seal the BQ76907 (return to protected mode)
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
static inline HAL_StatusTypeDef BQ76907_Seal(void){
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t subcmd[2] = {0x30, 0x00}; // SEAL() subcommand

    // 1. Write 0x30 to 0x3E
    status = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(status != HAL_OK) return status;

    // 2. Write 0x00 to 0x3F
    status = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(status != HAL_OK) return status;

    // No further action needed for SEAL()
    return status;
}

/**
 * @brief Unseal the BQ76907 with keys (exit protected mode) using the legacy method
 * @param key1 16-bit key1
 * @param key2 16-bit key2
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
static inline HAL_StatusTypeDef BQ76907_Unseal(uint16_t key1, uint16_t key2) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t keyBytes[2];

    // Write key1 (LSB to 0x3E, MSB to 0x3F)
    keyBytes[0] = key1 & 0xFF;         // LSB
    keyBytes[1] = (key1 >> 8) & 0xFF;  // MSB
    status = BQ76907_WriteRegister(0x3E, &keyBytes[0], 1);
    if(status != HAL_OK) return status;
    status = BQ76907_WriteRegister(0x3F, &keyBytes[1], 1);
    if(status != HAL_OK) return status;

    // Write key2 (LSB to 0x3E, MSB to 0x3F)
    keyBytes[0] = key2 & 0xFF;         // LSB
    keyBytes[1] = (key2 >> 8) & 0xFF;  // MSB
    status = BQ76907_WriteRegister(0x3E, &keyBytes[0], 1);
    if(status != HAL_OK) return status;
    status = BQ76907_WriteRegister(0x3F, &keyBytes[1], 1);
    if(status != HAL_OK) return status;

    return status;
}

/**
 * @brief Reset the BQ76907
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_Reset(void){
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t subcmd[2] = {0x12, 0x00}; // RESET() subcommand

    // 1. Write 0x12 to 0x3E    
    status = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(status != HAL_OK) return status;

    // 2. Write 0x00 to 0x3F
    status = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(status != HAL_OK) return status;
    
    // 3. Wait for the BQ76907 to reset 
    HAL_Delay(100);

    return status;
}

/**
 * @brief Set the BQ76907 to allow configuration updates
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
static inline HAL_StatusTypeDef BQ76907_SetCFGUpdate(void){
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t subcmd[2] = {0x90, 0x00}; // SET_CFGUPDATE() subcommand

    // 1. Write 0x90 to 0x3E
    status = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(status != HAL_OK) return status;

    // 2. Write 0x00 to 0x3F
    status = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(status != HAL_OK) return status;

    // Wait for the BQ76907 to enter configuration update mode
    while(1){
        uint8_t status[2]; 
        BQ76907_ReadRegister(0x12, status, 2); 
        if(status[0] & (1 << 5)) break;
    }

    return status;
}

/**
 * @brief Exit configuration update mode
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
static inline HAL_StatusTypeDef BQ76907_ExitCFGUpdate(void){
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t subcmd[2] = {0x92, 0x00}; // EXIT_CFGUPDATE() subcommand
    
    // 1. Write 0x91 to 0x3E
    status = BQ76907_WriteRegister(0x3E, &subcmd[0], 1);
    if(status != HAL_OK) return status;

    // 2. Write 0x00 to 0x3F
    status = BQ76907_WriteRegister(0x3F, &subcmd[1], 1);
    if(status != HAL_OK) return status;

    // Wait for the BQ76907 to exit configuration update mode
    while(1){
        uint8_t status[2]; 
        BQ76907_ReadRegister(0x12, status, 2); 
        if(!(status[0] & (1 << 5))) break;
    }
    return status;
}

/**
 * @brief Disable sleep mode
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
static inline HAL_StatusTypeDef BQ76907_SleepDisable(void){
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t subCmd[2] = {0x9A, 0x00}; // SLEEP_DISABLE() subcommand
    
    // 1. Write 0x9A to 0x3E
    status = BQ76907_WriteRegister(0x3E, &subCmd[0], 1);
    if(status != HAL_OK) return status;

    // 2. Write 0x00 to 0x3F
    status = BQ76907_WriteRegister(0x3F, &subCmd[1], 1);
    if(status != HAL_OK) return status;

    return status;
}

/**
 * @brief Configure the FET options
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
static inline HAL_StatusTypeDef BQ76907_ConfigureFETOptions(void){
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t addr[2] = {0x1E, 0x90};
    uint8_t value = 0x68; 
    uint8_t checksum = ~(addr[0] + addr[1] + value);
    uint8_t length = 5;

    status = BQ76907_WriteRegister(0x3E, &addr[0], 1);   // Set address pointer
    if(status != HAL_OK) return status;
    status = BQ76907_WriteRegister(0x3F, &addr[1], 1);
    if(status != HAL_OK) return status;
    status = BQ76907_WriteRegister(0x40, &value, 1);     // Write value
    if(status != HAL_OK) return status;
    status = BQ76907_WriteRegister(0x60, &checksum, 1);  // Write checksum
    if(status != HAL_OK) return status;
    status = BQ76907_WriteRegister(0x61, &length, 1);    // Write length
    if(status != HAL_OK) return status;

    return status;
}

/**
 * @brief Set the OTD threshold
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
static inline HAL_StatusTypeDef BQ76907_SetOTDThreshold(void){
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t addr[2] = {0x46, 0x90};
    uint8_t value = 0x48; 
    uint8_t checksum = ~(addr[0] + addr[1] + value);
    uint8_t length = 5;

    status = BQ76907_WriteRegister(0x3E, &addr[0], 1);   // Set address pointer
    if(status != HAL_OK) return status;
    status = BQ76907_WriteRegister(0x3F, &addr[1], 1);
    if(status != HAL_OK) return status;
    status = BQ76907_WriteRegister(0x40, &value, 1);     // Write value
    if(status != HAL_OK) return status;
    status = BQ76907_WriteRegister(0x60, &checksum, 1);  // Write checksum
    if(status != HAL_OK) return status;
    status = BQ76907_WriteRegister(0x61, &length, 1);    // Write length
    if(status != HAL_OK) return status;

    return status;
}

/**
 * @brief Enable the thermistor pullup and configure REGOUT_CONTROL
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
static inline HAL_StatusTypeDef BQ76907_EnableThermistorPullup(void){
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t value = 0x1E;

    status = BQ76907_WriteRegister(REGOUT_CONTROL, &value, 1);
    if(status != HAL_OK) return status;

    return status;
}

/**
 * @brief Set the OTD threshold
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
static inline HAL_StatusTypeDef BQ76907_EnableProtectionsB(void){
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t addr[2] = {0x25, 0x90};
    uint8_t value = 0x10; 
    uint8_t checksum = ~(addr[0] + addr[1] + value);
    uint8_t length = 5;

    status = BQ76907_WriteRegister(0x3E, &addr[0], 1);   // Set address pointer
    if(status != HAL_OK) return status;
    status = BQ76907_WriteRegister(0x3F, &addr[1], 1);
    if(status != HAL_OK) return status;
    status = BQ76907_WriteRegister(0x40, &value, 1);     // Write value
    if(status != HAL_OK) return status;
    status = BQ76907_WriteRegister(0x60, &checksum, 1);  // Write checksum
    if(status != HAL_OK) return status;
    status = BQ76907_WriteRegister(0x61, &length, 1);    // Write length
    if(status != HAL_OK) return status;

    return status;
}

/**
 * @brief Initialize the BQ76907
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_Init(void){
    HAL_StatusTypeDef status = HAL_OK;
    status = BQ76907_Reset();
    if(status != HAL_OK) return status;
    status = BQ76907_Unseal(0x0414, 0x3672);
    if(status != HAL_OK) return status;
    status = BQ76907_SetCFGUpdate();
    if(status != HAL_OK) return status;
    status = BQ76907_ConfigureFETOptions();
    if(status != HAL_OK) return status;
    status = BQ76907_SetOTDThreshold();
    if(status != HAL_OK) return status;
    status = BQ76907_SleepDisable();
    if(status != HAL_OK) return status;
    status = BQ76907_EnableThermistorPullup();
    if(status != HAL_OK) return status;
    status = BQ76907_EnableProtectionsB();
    if(status != HAL_OK) return status;
    status = BQ76907_ExitCFGUpdate();
    if(status != HAL_OK) return status;

    return BQ76907_OK; 
}

/**
 * @brief Turn off the DSG FET
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_DSGFETOff(void){
    HAL_StatusTypeDef status = HAL_OK; 
    uint8_t data = 0x04; //DSG_OFF

    status = BQ76907_WriteRegister(FET_CONTROL, &data, 1);
    if(status != HAL_OK) return status;

    return status;
}

/**
 * @brief Turn on the DSG FET
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_DSGFETOn(void){
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t data = 0x01; // DSG_ON

    status = BQ76907_WriteRegister(FET_CONTROL, &data, 1);
    if(status != HAL_OK) return status;

    return status;
}

/**
 * @brief Read the cell voltages
 * @param voltages Pointer to array to store the voltages
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_ReadCellVoltages(uint16_t *voltages){
    HAL_StatusTypeDef status = HAL_OK;

    status = BQ76907_ReadRegister(CELL_1_VOLTAGE, (uint8_t*)voltages, NUM_CELLS * 2);
    if(status != HAL_OK) return status;

    return status;  
}

/**
 * @brief Convert the ADC code to temperature
 * @param adcCode ADC code
 * @return Temperature in Celsius
 */
static inline float BQ76907_TSADCtoTemperature(uint16_t adcCode){
    float vTs = adcCode * 1.8f * 5.0f / 3.0f / 32768.0f; // ≈ 92uV per LSB
    float vRef = 1.8f;
    float rPullup = 20000.0f; // 20kΩ
    float rNTC = rPullup * vTs / (vRef - vTs); 

    // Define thermistor parameters (adjust as needed for your NTC)
    float B = 3435.0f;      // Beta value of thermistor (typical: 3435 or 3950)
    float R0 = 10000.0f;    // Resistance at 25°C (10kΩ)
    float T0 = 298.15f;     // 25°C in Kelvin

    float tempK = 1.0f / ((1.0f/B) * logf(rNTC/R0) + (1.0f/T0));
    float tempC = tempK - 273.15f;
    return tempC;
}

/**
 * @brief Read the battery temperature
 * @param temp Pointer to array to store the temperature
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_ReadBatteryTemperature(float *temp){
    HAL_StatusTypeDef status = HAL_OK;
    uint16_t adcCode;

    status = BQ76907_ReadRegister(TS_MEASUREMENT, (uint8_t*)&adcCode, 2);
    if(status != HAL_OK) return status;

    *temp = BQ76907_TSADCtoTemperature(adcCode);

    return status;
}

/**
 * @brief Enable alarms
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_EnableAlarms(void){
    HAL_StatusTypeDef status = HAL_OK;
    uint16_t data = 0xF000;

    status = BQ76907_WriteRegister(ALARM_ENABLE, (uint8_t*)&data, 2);
    if(status != HAL_OK) return status;

    return status;
}

/**
 * @brief Read the alarm status
 * @param status Pointer to array to store the status
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */ 
HAL_StatusTypeDef BQ76907_ReadAlarmStatus(uint16_t *status){
    HAL_StatusTypeDef status = HAL_OK;

    status = BQ76907_ReadRegister(ALARM_STATUS, (uint8_t*)status, 2);
    if(status != HAL_OK) return status;

    return status;
}