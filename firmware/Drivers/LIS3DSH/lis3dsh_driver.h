#ifndef LIS3DSH_DRIVER_H
#define LIS3DSH_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef enum
{
    LIS3DSH_STATUS_OK = 0,
    LIS3DSH_STATUS_ERROR,
    LIS3DSH_STATUS_INVALID_ID
} LIS3DSH_Status_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} LIS3DSH_RawData_t;

LIS3DSH_Status_t LIS3DSH_Init(SPI_HandleTypeDef *hspi);

LIS3DSH_Status_t LIS3DSH_ReadWhoAmI(uint8_t *device_id);

LIS3DSH_Status_t LIS3DSH_ReadRaw(LIS3DSH_RawData_t *data);

#endif
