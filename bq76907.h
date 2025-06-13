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
/* For GPIO definitons */
#include "main.h"

/**
 * @brief Read the battery status from the BQ76907
 * @param status Pointer to 2-byte array to store the status (status[0] = status, status[1] = flags)
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_ReadBatteryStatus(uint16_t *status);

/** 
 * @brief Read the security keys from the BQ76907
 * @param keys Pointer to 4-byte array to store the keys (keys[0,1] = key1 LSB,MSB, keys[2,3] = key2 LSB,MSB)
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_ReadKeys(uint8_t *keys);

/**
 * @brief Write the security keys to the BQ76907
 * @param keys Pointer to 4-byte array containing the keys (keys[0,1] = key1 LSB,MSB, keys[2,3] = key2 LSB,MSB)
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_WriteKeys(uint8_t *keys);   

/**
 * @brief Reset the BQ76907
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_Reset(void);

/**
 * @brief Enter deep sleep mode
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_EnterDeepSleep(void);

/**
 * @brief Exit deep sleep mode
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_ExitDeepSleep(void);

/**
 * @brief Read the CUV threshold
 * @param value Pointer to 2-byte array to store the value
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_ReadCUVThreshold(uint16_t *value);

/**
 * @brief Read the CUV recovery hysterisis
 * @param value Pointer to 1-byte array to store the value
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_ReadCUVRecoveryHysterisis(uint8_t *value);

/**
 * @brief Initialize the BQ76907
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_Init(void);

/**
 * @brief Turn on the DSG FET
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_DSGFETOn(void);

/**
 * @brief Turn on the CHG FET
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_CHGFETOn(void);

/**
 * @brief Turn off the DSG FET
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_DSGFETOff(void);

/**
 * @brief Turn off the CHG FET
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_CHGFETOff(void);

/**
 * @brief Read the cell voltages
 * @param voltages Pointer to array to store the voltages
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_ReadCellVoltages(uint16_t *voltages);
/**
 * @brief Read the battery temperature
 * @param temp Pointer to array to store the temperature
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_ReadBatteryTemperature(float *temp);

/**
 * @brief Enable alarms
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_EnableAlarms(void);

/**
 * @brief Read the alarm status
 * @param status Pointer to array to store the status
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_ReadAlarmStatus(uint16_t *status);

/**
 * @brief Clear the alarm status
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_ClearAlarmStatus(uint16_t status);

/**
 * @brief Read the safety status A register
 * @param status Pointer to array to store the status
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_ReadSafetyStatusA(uint8_t *status);

/**
 * @brief Read the safety status B register
 * @param status Pointer to array to store the status
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_ReadSafetyStatusB(uint8_t *status);

/**
 * @brief Read the safety alert A register
 * @param status Pointer to array to store the status
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_ReadSafetyAlertA(uint8_t *status);

/**
 * @brief Read the safety alert B register
 * @param status Pointer to array to store the status
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_ReadSafetyAlertB(uint8_t *status);

/**
 * @brief Enable cell balance
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_CellBalance(uint8_t activeCells);

/**
 * @brief Balance the minimum and maximum cells
 * @param cellVoltages Pointer to array to store the cell voltages
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_BalanceMinMaxCells(uint16_t cellVoltages[6]);

/**
 * @brief Read the active cells
 * @param activeCells Pointer to array to store the active cells
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_ReadCBActiveCells(uint8_t *activeCells);

/**
 * @brief Initialize the state of charge (SOC)
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_SoCInit(void);

/**
 * @brief Update the state of charge (SOC)
 * @return ErrorCode_t (BQ76907_OK on success)
 */
ErrorCode_t BQ76907_SoCUpdate(void);

/**
 * @brief Get the state of charge (SOC)
 * @return float (state of charge)
 */
float BQ76907_SoCGet(void);

#ifdef __cplusplus
}
#endif

#endif
