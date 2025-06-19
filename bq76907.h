/**
 * @file bq76907.h
 * @brief BQ76907 battery management system driver header
 * @details This header file contains the declarations and definitions for the
 *          BQ76907 battery management system driver, supporting various
 *          operations.
 * @author Cengiz Sinan Kostakoglu
 * @version 1.1
 * @date 2025-06-19
 */

#ifndef __BQ76907_H__
#define __BQ76907_H__

/* For HAL functions and error codes */
#include "main.h"

/**
 * @brief Read the battery status
 * @param status Pointer to store the battery status
 * @return BQ76907 error code
 */
BQ76907_ErrorCode_t BQ76907_ReadBatteryStatus(uint16_t *status);

/**
 * @brief Read the cell voltages
 * @param cellVoltages Pointer to store the cell voltages
 * @return BQ76907 error code
 */
BQ76907_ErrorCode_t BQ76907_ReadCellVoltages(uint16_t *cellVoltages);

/**
 * @brief Read the temperature
 * @param temperature Pointer to store the temperature
 * @return BQ76907 error code
 */
BQ76907_ErrorCode_t BQ76907_ReadBatteryTemperature(int8_t *temperature);

/**
 * @brief Read the security keys
 * @param key1 Pointer to store the first 16-bit key
 * @param key2 Pointer to store the second 16-bit key
 * @return BQ76907 error code
 */
BQ76907_ErrorCode_t BQ76907_ReadSecurityKeys(uint16_t *key1, uint16_t *key2);

/**
 * @brief Initialize the BQ76907
 * @return BQ76907 error code
 */
BQ76907_ErrorCode_t BQ76907_Init(void);

/**
 * @brief Handle alarms
 * @return BQ76907 error code
 */
BQ76907_ErrorCode_t BQ76907_HandleAlarms(void);

#endif

