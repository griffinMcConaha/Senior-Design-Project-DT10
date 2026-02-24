#include "i2c_utils.h"
#include <stdio.h>

// Scan I2C bus from 0x08 to 0x77 (7-bit addresses); prints found devices to console
uint8_t I2C_ScanBus(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL) return 0;

    printf("\r\n--- I2C Scan Start ---\r\n");

    uint8_t found = 0;

    // Scan standard I2C range (0x08 to 0x77 in 7-bit notation)
    for (uint16_t addr = 0x08; addr <= 0x77; addr++)
    {
        // HAL_I2C uses 8-bit address (7-bit address shifted left by 1)
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 2, 10) == HAL_OK)
        {
            printf("I2C device found at 0x%02X\r\n", (unsigned)addr);
            found++;
        }
    }

    if (found == 0)
        printf("No I2C devices found.\r\n");
    else
        printf("Scan complete: %u device(s) found.\r\n", found);

    printf("--- I2C Scan End ---\r\n\r\n");

    return found;
}
