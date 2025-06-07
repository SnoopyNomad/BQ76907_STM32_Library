#ifndef BQ76907_H_
#define BQ76907_H_

#include "main.h"

ErrorCode_t BQ76907_Init(void);
ErrorCode_t BQ76907_ReadCellVoltages(uint16_t *cellVoltages);
ErrorCode_t BQ76907_ReadTemperature(float *temp);
ErrorCode_t BQ76907_ReadAccumulatedCharge(int64_t *accumulatedCharge, uint32_t *timerValue);
ErrorCode_t BQ76907_ResetChargeIntegrator(void);
float BQ76907_CalculateSOC(int64_t accumulatedCharge, uint32_t nominalCapacity);
ErrorCode_t BQ76907_ManageCellBalancing(bool isCharging, uint8_t *activeBalancingCells);

#endif