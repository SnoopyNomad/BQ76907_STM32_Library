/**
 * @file bq76907.h
 * @brief BQ76907 battery management system IC driver implementation
 * @details This file contains the implementation of the BQ76907 battery management system IC driver,
 *          supporting various battery protection features and configuration options.
 * @author Cengiz Sinan Kostakoglu
 * @version 1.0
 * @date 2025-06-12
 */

#ifndef __BQ76907_H__
#define __BQ76907_H__

#ifdef __cplusplus
extern "C" {
#endif

/* For uint8_t, uint16_t, uint32_t */
#include <stdint.h> 
 /* For GPIO definitions */
#include "main.h"

/**
 * @brief Read the battery status from the BQ76907
 * @param status Pointer to 2-byte array to store the status (status[0] = status, status[1] = flags)
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_ReadBatteryStatus(uint16_t *status);

/**
 * @brief Reset the BQ76907
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_Reset(void);

/**
 * @brief Initialize the BQ76907
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_Init(void);

/**
 * @brief Turn on the DSG FET
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_DSGFETOn(void);

/**
 * @brief Turn off the DSG FET
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_DSGFETOff(void);

/**
 * @brief Read the cell voltages
 * @param voltages Pointer to array to store the voltages
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_ReadCellVoltages(uint16_t *voltages);

/**
 * @brief Read the battery temperature
 * @param temp Pointer to array to store the temperature
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_ReadBatteryTemperature(float *temp);

/**
 * @brief Enable alarms
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_EnableAlarms(void);

/**
 * @brief Read the enabled alarms
 * @param status Pointer to array to store the status
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_ReadEnabledAlarms(uint16_t *status);

/**
 * @brief Read the alarm status
 * @param status Pointer to array to store the status
 * @return HAL_StatusTypeDef (HAL_OK on success)
 */
HAL_StatusTypeDef BQ76907_ReadAlarmStatus(uint16_t *status);

#ifdef __cplusplus
}
#endif

#endif
