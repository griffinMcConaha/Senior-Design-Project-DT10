#ifndef I2C_UTILS_H
#define I2C_UTILS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

// Scan I2C bus (addresses 0x08 to 0x77) and print found devices; returns count
uint8_t I2C_ScanBus(I2C_HandleTypeDef *hi2c);

#endif
