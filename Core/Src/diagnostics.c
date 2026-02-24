#include "diagnostics.h"
#include "sabertooth.h"
#include "dispersion.h"
#include "proximity.h"
#include "imu_icm20948.h"
#include "gps.h"
#include "robot_sm.h"
#include "heading_fusion.h"
#include "system_health.h"
#include "console_io.h"  // For ANSI color codes
#include "stm32f4xx_hal.h"
#include <stdio.h>

// ============================================================================
// HARDWARE DIAGNOSTICS AND COMPONENT TESTING
// Interactive tests for individual components without full system operation.
// Most tests are blocking; watchdog kicks are required to avoid resets.
// ============================================================================

static inline void Diag_WatchdogKick(void)
{
    IWDG->KR = 0xAAAA;
}

// Test individual motor speed
void Diag_TestMotor(uint8_t motor, int speed)
{
    extern UART_HandleTypeDef huart2;
    
    // Clamp speed to valid range (percent)
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    printf(ANSI_CYAN "[DIAG] Testing Motor %d at speed %d\r\n" ANSI_RESET, motor, speed);

    // Set motor
    if (motor == 1)
        Sabertooth_SetM1(speed);
    else if (motor == 2)
        Sabertooth_SetM2(speed);
    else {
        printf(ANSI_RED "[DIAG] ERROR: Invalid motor %d (1-2)\r\n" ANSI_RESET, motor);
        Console_ShowTestMenu();
        return;
    }

    printf(ANSI_GREEN "[DIAG] Motor %d speed applied\r\n" ANSI_RESET, motor);
    printf(ANSI_CYAN "[DIAG] Press ESC to stop motor and return to menu\r\n\r\n" ANSI_RESET);

    // Wait for ESC
    while (1)
    {
        Diag_WatchdogKick();
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
            if (ch == 0x1B) {
                break;
            }
        }
        if (Console_CheckEscPressed()) {
            break;
        }
        HAL_Delay(100);
    }

    // Stop motor
    Sabertooth_StopAll();
    printf("\r\n" ANSI_GREEN "[DIAG] Motor %d stopped\r\n\r\n" ANSI_RESET, motor);
    Console_ShowTestMenu();
}

// Test individual motor raw command value (-2047 to +2047)
void Diag_TestMotorRaw(uint8_t motor, int value)
{
    extern UART_HandleTypeDef huart2;

    // Clamp value to valid range
    if (value > 2047) value = 2047;
    if (value < -2047) value = -2047;

    printf(ANSI_CYAN "[DIAG] Testing Motor %d raw command %d\r\n" ANSI_RESET, motor, value);

    // Send raw command
    if (motor == 1 || motor == 2) {
        Sabertooth_SetMotorRaw(motor, value);
    } else {
        printf(ANSI_RED "[DIAG] ERROR: Invalid motor %d (1-2)\r\n" ANSI_RESET, motor);
        Console_ShowTestMenu();
        return;
    }

    printf(ANSI_GREEN "[DIAG] Motor %d raw command applied\r\n" ANSI_RESET, motor);
    printf(ANSI_CYAN "[DIAG] Press ESC to stop motor and return to menu\r\n\r\n" ANSI_RESET);

    // Wait for ESC
    while (1)
    {
        Diag_WatchdogKick();
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
            if (ch == 0x1B) {
                break;
            }
        }
        if (Console_CheckEscPressed()) {
            break;
        }
        HAL_Delay(100);
    }

    // Stop motor
    Sabertooth_StopAll();
    printf("\r\n" ANSI_GREEN "[DIAG] Motor %d stopped\r\n\r\n" ANSI_RESET, motor);
    Console_ShowTestMenu();
}

// Sweep all Sabertooth motor speeds (-100 to +100) with delay between each
void Diag_TestMotorSweep(uint16_t delay_ms)
{
    extern UART_HandleTypeDef huart2;
    
    printf(ANSI_YELLOW "[DIAG] ========== MOTOR SWEEP TEST ==========\r\n" ANSI_RESET);
    printf(ANSI_CYAN "[DIAG] Testing Motor 1: -100 to +100 with %d ms delay\r\n" ANSI_RESET, delay_ms);
    printf(ANSI_RED "[DIAG] WARNING: Robot will move! Ensure clear space!\r\n" ANSI_RESET);
    printf(ANSI_CYAN "[DIAG] Press ESC to stop sweep\r\n\r\n" ANSI_RESET);

    // Motor 1: Reverse speeds
    for (int speed = -100; speed <= 100; speed += 10)
    {
        Diag_WatchdogKick();
        Sabertooth_SetM1(speed);
        printf("[DIAG] M1 = %3d\r\n", speed);
        
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
            if (ch == 0x1B) {
                printf(ANSI_YELLOW "\r\n[DIAG] ESC pressed. Stopping sweep.\r\n" ANSI_RESET);
                Sabertooth_StopAll();
                Console_ShowTestMenu();
                return;
            }
        }
        if (Console_CheckEscPressed()) {
            printf(ANSI_YELLOW "\r\n[DIAG] ESC pressed. Stopping sweep.\r\n" ANSI_RESET);
            Sabertooth_StopAll();
            Console_ShowTestMenu();
            return;
        }
        
        HAL_Delay(delay_ms);
    }

    // Stop and continue to Motor 2
    Sabertooth_StopAll();
    printf("\r\n" ANSI_GREEN "[DIAG] Motor 1 sweep complete.\r\n" ANSI_RESET);
    printf(ANSI_CYAN "[DIAG] Testing Motor 2: -100 to +100 with %d ms delay\r\n" ANSI_RESET, delay_ms);
    printf(ANSI_RED "[DIAG] WARNING: Robot will move! Ensure clear space!\r\n" ANSI_RESET);
    printf(ANSI_CYAN "[DIAG] Press ESC to stop sweep\r\n\r\n" ANSI_RESET);

    // Motor 2: Reverse speeds
    for (int speed = -100; speed <= 100; speed += 10)
    {
        Diag_WatchdogKick();
        Sabertooth_SetM2(speed);
        printf("[DIAG] M2 = %3d\r\n", speed);
        
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
            if (ch == 0x1B) {
                printf(ANSI_YELLOW "\r\n[DIAG] ESC pressed. Stopping sweep.\r\n" ANSI_RESET);
                Sabertooth_StopAll();
                Console_ShowTestMenu();
                return;
            }
        }
        if (Console_CheckEscPressed()) {
            printf(ANSI_YELLOW "\r\n[DIAG] ESC pressed. Stopping sweep.\r\n" ANSI_RESET);
            Sabertooth_StopAll();
            Console_ShowTestMenu();
            return;
        }
        
        HAL_Delay(delay_ms);
    }

    // Stop
    Sabertooth_StopAll();
    printf("\r\n" ANSI_GREEN "[DIAG] Motor 2 sweep complete.\r\n" ANSI_RESET);
    printf(ANSI_YELLOW "[DIAG] ========== MOTOR SWEEP TEST COMPLETE ==========\r\n\r\n" ANSI_RESET);
    Console_ShowTestMenu();
}

// Test single salt dispersion rate (0-100%)
void Diag_TestSaltRate(uint8_t rate_percent)
{
    extern UART_HandleTypeDef huart2;
    
    if (rate_percent > 100) rate_percent = 100;

    printf(ANSI_CYAN "[DIAG] Testing Salt Rate: %d%%\r\n" ANSI_RESET, rate_percent);

    // Set salt rate (brine auto-adjusts to maintain 1:9 ratio)
    Dispersion_SetRate(rate_percent, 0); // Brine will be auto-set

    printf(ANSI_GREEN "[DIAG] Salt Rate %d%% applied\r\n" ANSI_RESET, rate_percent);
    printf(ANSI_CYAN "[DIAG] Press ESC to stop and reset to 0%%\r\n\r\n" ANSI_RESET);

    // Wait for ESC
    while (1)
    {
        Diag_WatchdogKick();
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
            if (ch == 0x1B) {
                break;
            }
        }
        if (Console_CheckEscPressed()) {
            break;
        }
        HAL_Delay(100);
    }

    // Reset to 0
    Dispersion_SetRate(0, 0);
    printf("\r\n" ANSI_GREEN "[DIAG] Salt rate reset to 0%%\r\n\r\n" ANSI_RESET);
    Console_ShowTestMenu();
}

// Sweep all salt dispersion rates (0-100%) with delay between each
void Diag_TestSaltSweep(uint16_t delay_ms)
{
    extern UART_HandleTypeDef huart2;

    printf(ANSI_YELLOW "[DIAG] ========== SALT RATE SWEEP TEST ==========\r\n" ANSI_RESET);
    printf(ANSI_CYAN "[DIAG] Testing salt rates 0-100%% with %d ms delay\r\n" ANSI_RESET, delay_ms);
    printf(ANSI_CYAN "[DIAG] Press ESC to stop sweep\r\n" ANSI_RESET);
    printf(ANSI_RED "[DIAG] WARNING: Salt auger will operate! Check for clogs!\r\n\r\n" ANSI_RESET);

    for (uint8_t rate = 0; rate <= 100; rate += 5)
    {
        Diag_WatchdogKick();
        Dispersion_SetRate(rate, 0);
        printf("[DIAG] Salt = %3d%%\r\n", rate);

        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
            if (ch == 0x1B) {
                printf(ANSI_YELLOW "\r\n[DIAG] ESC pressed. Stopping sweep.\r\n" ANSI_RESET);
                Dispersion_SetRate(0, 0);
                Console_ShowTestMenu();
                return;
            }
        }
        if (Console_CheckEscPressed()) {
            printf(ANSI_YELLOW "\r\n[DIAG] ESC pressed. Stopping sweep.\r\n" ANSI_RESET);
            Dispersion_SetRate(0, 0);
            Console_ShowTestMenu();
            return;
        }

        HAL_Delay(delay_ms);
    }

    // Stop dispensing
    Dispersion_SetRate(0, 0);
    printf("\r\n" ANSI_GREEN "[DIAG] Salt sweep complete. Dispensing stopped.\r\n" ANSI_RESET);
    printf(ANSI_YELLOW "[DIAG] ===========================================\r\n\r\n" ANSI_RESET);
    Console_ShowTestMenu();
}

// Test single brine dispersion rate (0-100%)
void Diag_TestBrineRate(uint8_t rate_percent)
{
    extern UART_HandleTypeDef huart2;

    if (rate_percent > 100) rate_percent = 100;

    printf(ANSI_CYAN "[DIAG] Testing Brine Rate: %d%%\r\n" ANSI_RESET, rate_percent);

    // For brine testing, we need to set salt proportionally (1:9 ratio)
    // If brine = 90%, then salt should be = 10% (90/9 = 10)
    uint8_t salt_rate = rate_percent / 9;
    if (salt_rate == 0 && rate_percent > 0) salt_rate = 1; // Minimum 1% salt if brine > 0

    Dispersion_SetRate(salt_rate, rate_percent);

    printf(ANSI_GREEN "[DIAG] Brine Rate %d%% applied (Salt: %d%%)\r\n" ANSI_RESET, 
           rate_percent, salt_rate);
    printf(ANSI_CYAN "[DIAG] Press ESC to stop and reset to 0%%\r\n\r\n" ANSI_RESET);

    // Wait for ESC
    while (1)
    {
        Diag_WatchdogKick();
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
            if (ch == 0x1B) {
                break;
            }
        }
        if (Console_CheckEscPressed()) {
            break;
        }
        HAL_Delay(100);
    }

    // Reset to 0
    Dispersion_SetRate(0, 0);
    printf("\r\n" ANSI_GREEN "[DIAG] Brine rate reset to 0%%\r\n\r\n" ANSI_RESET);
    Console_ShowTestMenu();
}

// Sweep all brine dispersion rates (0-100%) with delay between each
void Diag_TestBrineSweep(uint16_t delay_ms)
{
    extern UART_HandleTypeDef huart2;
    
    printf(ANSI_YELLOW "[DIAG] ========== BRINE RATE SWEEP TEST ==========\r\n" ANSI_RESET);
    printf(ANSI_CYAN "[DIAG] Testing brine rates 0-100%% with %d ms delay\r\n" ANSI_RESET, delay_ms);
    printf(ANSI_RED "[DIAG] WARNING: Brine pump will operate! Check for leaks!\r\n" ANSI_RESET);
    printf(ANSI_CYAN "[DIAG] Press ESC to stop sweep\r\n\r\n" ANSI_RESET);

    for (uint8_t brine_rate = 0; brine_rate <= 100; brine_rate += 5)
    {
        Diag_WatchdogKick();
        // Calculate salt for 1:9 ratio
        uint8_t salt_rate = brine_rate / 9;
        if (salt_rate == 0 && brine_rate > 0) salt_rate = 1;

        Dispersion_SetRate(salt_rate, brine_rate);
        printf("[DIAG] Brine = %3d%% (Salt = %3d%%)\r\n", brine_rate, salt_rate);
        
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
            if (ch == 0x1B) {
                printf(ANSI_YELLOW "\r\n[DIAG] ESC pressed. Stopping sweep.\r\n" ANSI_RESET);
                Dispersion_SetRate(0, 0);
                Console_ShowTestMenu();
                return;
            }
        }
        if (Console_CheckEscPressed()) {
            printf(ANSI_YELLOW "\r\n[DIAG] ESC pressed. Stopping sweep.\r\n" ANSI_RESET);
            Dispersion_SetRate(0, 0);
            Console_ShowTestMenu();
            return;
        }
        
        HAL_Delay(delay_ms);
    }

    // Stop dispensing
    Dispersion_SetRate(0, 0);
    printf("\r\n" ANSI_GREEN "[DIAG] Brine sweep complete. Dispensing stopped.\r\n" ANSI_RESET);
    printf(ANSI_YELLOW "[DIAG] ============================================\r\n\r\n" ANSI_RESET);
    Console_ShowTestMenu();
}

// Test GPS sensor - read current position
void Diag_TestGPS(void)
{
    extern UART_HandleTypeDef huart2;
    printf(ANSI_YELLOW "[DIAG] ========== GPS TEST ==========\r\n" ANSI_RESET);
    printf(ANSI_CYAN "[DIAG] Continuous display (press ESC to exit)\r\n\r\n" ANSI_RESET);

    while (1)
    {
        Diag_WatchdogKick();
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
            if (ch == 0x1B) {
                break;
            }
        }
        if (Console_CheckEscPressed()) {
            break;
        }

        const GPS_Data_t *gps = GPS_Get();

        if (!gps->has_fix)
        {
            printf("\r" ANSI_YELLOW "[DIAG] GPS: No fix" ANSI_RESET " | Sats: %d | HDOP: %.1f | VDOP: %.1f    ",
                   gps->num_satellites, gps->hdop, gps->vdop);
        }
        else
        {
            printf("\r" ANSI_GREEN "[DIAG] GPS: FIX" ANSI_RESET " | Lat: %.6f° | Lon: %.6f° | Sats: %d | HDOP: %.1f | Alt: %.1fm | Speed: %.2fkn    ",
                   gps->latitude_deg, gps->longitude_deg, gps->num_satellites,
                   gps->hdop, gps->altitude_m, gps->speed_knots);
        }
        fflush(stdout);

        HAL_Delay(1000);  // Update 1x per second
    }

    printf("\r\n" ANSI_YELLOW "[DIAG] ==================================\r\n\r\n" ANSI_RESET);
    Console_ShowTestMenu();
}

// Test IMU sensor - read accel/gyro/mag
void Diag_TestIMU(void)
{
    extern UART_HandleTypeDef huart2;
    printf("[DIAG] ========== IMU TEST ==========\r\n");
    printf("[DIAG] Continuous display (press ESC to exit)\r\n\r\n");

    while (1)
    {
        Diag_WatchdogKick();
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
            if (ch == 0x1B) {
                break;
            }
        }
        if (Console_CheckEscPressed()) {
            break;
        }

        IMU_Status_t imu = IMU_Read();

        if (!imu.ok)
        {
            printf("\r[DIAG] IMU: ERROR - Not responding    ");
        }
        else
        {
            printf("\r[DIAG] Accel: X=%6.3fg Y=%6.3fg Z=%6.3fg | Gyro: X=%6.3f Y=%6.3f Z=%6.3f rad/s",
                   imu.ax_g, imu.ay_g, imu.az_g,
                   imu.gx_rad_s, imu.gy_rad_s, imu.gz_rad_s);
#if IMU_USE_MAG
            printf(" | Mag: X=%5.0f Y=%5.0f Z=%5.0fµT    ",
                   imu.mx_uT, imu.my_uT, imu.mz_uT);
#else
            printf("    ");
#endif
        }
        fflush(stdout);

        HAL_Delay(1000);  // Update 1x per second
    }

    printf("\r\n[DIAG] ==================================\r\n\r\n");
    Console_ShowTestMenu();
}

// Detailed IMU communication check
void Diag_TestIMUDetailed(void)
{
    extern I2C_HandleTypeDef hi2c1;
    HAL_StatusTypeDef status;
    uint8_t data;

    printf(ANSI_YELLOW "[DIAG] ========== IMU DETAILED CHECK ==========\r\n" ANSI_RESET);

    // Test 1: I2C bus ready check
    printf(ANSI_CYAN "[DIAG] 1. Checking if I2C1 bus is ready...\r\n" ANSI_RESET);
    if (hi2c1.Instance == NULL) {
        printf(ANSI_RED "[DIAG]    ERROR: I2C1 handle not initialized!\r\n" ANSI_RESET);
        printf(ANSI_YELLOW "[DIAG] ============================================\r\n\r\n" ANSI_RESET);
        return;
    }
    printf(ANSI_GREEN "[DIAG]    I2C1 handle OK\r\n" ANSI_RESET);

    // Test 2: Device presence - try both addresses
    printf(ANSI_CYAN "[DIAG] 2. Checking if device responds at address 0x69 or 0x68...\r\n" ANSI_RESET);
    uint8_t imu_addr = 0x69;
    status = HAL_I2C_IsDeviceReady(&hi2c1, 0x69 << 1, 3, 100);
    if (status != HAL_OK) {
        // Try 0x68
        status = HAL_I2C_IsDeviceReady(&hi2c1, 0x68 << 1, 3, 100);
        imu_addr = 0x68;
        if (status != HAL_OK) {
            printf(ANSI_RED "[DIAG]    ERROR: No ACK from 0x69 or 0x68 (error code: %d)\r\n" ANSI_RESET, status);
            printf(ANSI_YELLOW "[DIAG]    Possible causes:\r\n");
            printf("[DIAG]    - Wrong I2C address (check AD0 pin level)\r\n");
            printf("[DIAG]    - SDA/SCL wiring incorrect (should be SDA=PB9, SCL=PB6)\r\n");
            printf("[DIAG]    - Missing pull-up resistors (built into breakout board)\r\n");
            printf("[DIAG]    - IMU not powered (VDD should be 3.3V)\r\n");
            printf("[DIAG]    - I2C bus stuck (power cycle may help)\r\n" ANSI_RESET);
            printf(ANSI_YELLOW "[DIAG] ============================================\r\n\r\n" ANSI_RESET);
            return;
        }
    }
    printf(ANSI_GREEN "[DIAG]    SUCCESS: Device ACKed at 0x%02X\r\n" ANSI_RESET, imu_addr);
    
    // Set the correct address for remaining tests
    IMU_SetAddress(imu_addr);

    // Test 3: Read WHO_AM_I register
    printf(ANSI_CYAN "[DIAG] 3. Reading WHO_AM_I register (0x00)...\r\n" ANSI_RESET);
    status = HAL_I2C_Mem_Read(&hi2c1, imu_addr << 1, 0x00, 1, &data, 1, 100);
    if (status != HAL_OK) {
        printf(ANSI_RED "[DIAG]    ERROR: Failed to read WHO_AM_I (error: %d)\r\n" ANSI_RESET, status);
        printf(ANSI_YELLOW "[DIAG] ============================================\r\n\r\n" ANSI_RESET);
        return;
    }
    printf("[DIAG]    WHO_AM_I = 0x%02X\r\n", data);
    if (data == 0xEA) {
        printf(ANSI_GREEN "[DIAG]    SUCCESS: Correct WHO_AM_I for ICM-20948!\r\n" ANSI_RESET);
    } else {
        printf(ANSI_YELLOW "[DIAG]    WARNING: Expected 0xEA, got 0x%02X\r\n", data);
        printf("[DIAG]    This may not be an ICM-20948!\r\n" ANSI_RESET);
    }

    // Test 4: Read PWR_MGMT_1 register
    printf(ANSI_CYAN "[DIAG] 4. Reading PWR_MGMT_1 register (0x06)...\r\n" ANSI_RESET);
    status = HAL_I2C_Mem_Read(&hi2c1, imu_addr << 1, 0x06, 1, &data, 1, 100);
    if (status != HAL_OK) {
        printf(ANSI_RED "[DIAG]    ERROR: Failed to read PWR_MGMT_1 (error: %d)\r\n" ANSI_RESET, status);
    } else {
        printf("[DIAG]    PWR_MGMT_1 = 0x%02X\r\n", data);
        if (data & 0x40) {
            printf(ANSI_YELLOW "[DIAG]    WARNING: Device is in SLEEP mode!\r\n" ANSI_RESET);
        } else {
            printf(ANSI_GREEN "[DIAG]    Device is AWAKE\r\n" ANSI_RESET);
        }
    }

    // Test 5: Try reading accelerometer data
    printf(ANSI_CYAN "[DIAG] 5. Reading accelerometer data (registers 0x2D-0x32)...\r\n" ANSI_RESET);
    uint8_t accel_data[6];
    status = HAL_I2C_Mem_Read(&hi2c1, imu_addr << 1, 0x2D, 1, accel_data, 6, 100);
    if (status != HAL_OK) {
        printf(ANSI_RED "[DIAG]    ERROR: Failed to read accelerometer (error: %d)\r\n" ANSI_RESET, status);
    } else {
        int16_t ax = (int16_t)((accel_data[0] << 8) | accel_data[1]);
        int16_t ay = (int16_t)((accel_data[2] << 8) | accel_data[3]);
        int16_t az = (int16_t)((accel_data[4] << 8) | accel_data[5]);
        printf("[DIAG]    Raw values: X=%d Y=%d Z=%d\r\n", ax, ay, az);
        printf(ANSI_GREEN "[DIAG]    SUCCESS: Accelerometer data readable\r\n" ANSI_RESET);
    }

    printf(ANSI_YELLOW "[DIAG] ============================================\r\n\r\n" ANSI_RESET);
    Console_ShowTestMenu();
}

// Scan entire I2C bus for devices
void Diag_ScanI2C(void)
{
    extern I2C_HandleTypeDef hi2c1;
    uint8_t found_count = 0;

    printf(ANSI_YELLOW "[DIAG] ========== I2C BUS SCAN ==========\r\n" ANSI_RESET);
    printf("[DIAG] Scanning I2C1 bus (addresses 0x01-0x7F)...\r\n");

    for (uint8_t addr = 1; addr < 128; addr++)
    {
        Diag_WatchdogKick();
        HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10);
        if (status == HAL_OK)
        {
            printf(ANSI_GREEN "[DIAG] Found device at address 0x%02X\r\n" ANSI_RESET, addr);
            found_count++;

            // Identify known devices
            if (addr == 0x69 || addr == 0x68) {
                printf(ANSI_CYAN "[DIAG]   -> Likely ICM-20948 IMU\r\n" ANSI_RESET);
            } else if (addr == 0x0C) {
                printf(ANSI_CYAN "[DIAG]   -> Likely AK09916 magnetometer\r\n" ANSI_RESET);
            }
        }
    }

    if (found_count == 0) {
        printf(ANSI_RED "[DIAG] No I2C devices found!\r\n" ANSI_RESET);
        printf(ANSI_YELLOW "[DIAG] Troubleshooting:\r\n");
        printf("[DIAG]   - Check SDA/SCL wiring (SDA=PB9, SCL=PB6)\r\n");
        printf("[DIAG]   - Verify 3.3V power to I2C devices\r\n");
        printf("[DIAG]   - Verify CubeMX I2C1 config: AF, open-drain, pull-up\r\n");
        printf("[DIAG]   - Try power cycling the board\r\n" ANSI_RESET);
    } else {
        printf(ANSI_GREEN "[DIAG] Total devices found: %d\r\n" ANSI_RESET, found_count);
    }

    printf(ANSI_YELLOW "[DIAG] =====================================\r\n\r\n" ANSI_RESET);
    Console_ShowTestMenu();
}

// Test proximity sensors - read left and right distances
void Diag_TestProximity(void)
{
    extern UART_HandleTypeDef huart2;
    
    printf(ANSI_YELLOW "[DIAG] ========== PROXIMITY TEST ==========\r\n" ANSI_RESET);
    printf(ANSI_CYAN "[DIAG] Continuous display (press ESC to exit)\r\n\r\n" ANSI_RESET);

    const char *status_str[] = {"CLEAR", "WARNING", "CRITICAL", "ERROR"};
    const char *status_color[] = {ANSI_GREEN, ANSI_YELLOW, ANSI_RED, ANSI_RED};

    while (1)
    {
        Diag_WatchdogKick();
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
            if (ch == 0x1B) {
                break;
            }
        }
        if (Console_CheckEscPressed()) {
            break;
        }

        uint16_t dist_left = Proximity_ReadLeft();
        uint16_t dist_right = Proximity_ReadRight();

        uint8_t status_left = Proximity_GetStatus(dist_left);
        uint8_t status_right = Proximity_GetStatus(dist_right);

        // Format proximity readings: show "NO_DETECT" in cyan, otherwise show distance with status color
        char left_str[16], right_str[16];
        char left_color[16] = "", right_color[16] = "";
        
        if (dist_left == PROX_NO_DETECTION) {
            snprintf(left_str, sizeof(left_str), "NO_DETECT");
            snprintf(left_color, sizeof(left_color), ANSI_CYAN);
        } else {
            snprintf(left_str, sizeof(left_str), "%3u cm", dist_left);
            snprintf(left_color, sizeof(left_color), "%s", status_color[status_left]);
        }
        
        if (dist_right == PROX_NO_DETECTION) {
            snprintf(right_str, sizeof(right_str), "NO_DETECT");
            snprintf(right_color, sizeof(right_color), ANSI_CYAN);
        } else {
            snprintf(right_str, sizeof(right_str), "%3u cm", dist_right);
            snprintf(right_color, sizeof(right_color), "%s", status_color[status_right]);
        }

        printf("\r" ANSI_YELLOW "[DIAG] Proximity: " ANSI_RESET 
               "%sLeft %s [%s]" ANSI_RESET " | "
               "%sRight %s [%s]" ANSI_RESET,
               left_color, left_str, status_str[status_left],
               right_color, right_str, status_str[status_right]);
        fflush(stdout);

        HAL_Delay(200);  // Update 5x per second (200ms)
    }

    printf("\r\n\r\n" ANSI_GREEN "[DIAG] Proximity test complete.\r\n" ANSI_RESET);
    printf(ANSI_YELLOW "[DIAG] ======================================\r\n\r\n" ANSI_RESET);
    Console_ShowTestMenu();
}

// Display current state machine state
void Diag_PrintState(void)
{
    extern RobotSM_t g_sm;
    extern HeadingFusion_t g_hf;

    printf(ANSI_YELLOW "[DIAG] ========== STATE INFO ==========\r\n" ANSI_RESET);
    printf(ANSI_CYAN "[DIAG] Current State: " ANSI_RESET "%d\r\n", (int)RobotSM_Current(&g_sm));
    printf(ANSI_CYAN "[DIAG] Requested State: " ANSI_RESET "%d\r\n", (int)RobotSM_Requested(&g_sm));
    printf(ANSI_CYAN "[DIAG] Heading (Fused): " ANSI_RESET "%.1f°\r\n", g_hf.yaw_deg);
    printf(ANSI_CYAN "[DIAG] Heading Confidence: " ANSI_RESET "%.2f\r\n", g_hf.heading_confidence);
    printf(ANSI_CYAN "[DIAG] Mission Active: " ANSI_RESET "%d\r\n", g_sm.mission.mission_active);
    printf(ANSI_CYAN "[DIAG] Waypoint Index: " ANSI_RESET "%d/%d\r\n",
           g_sm.mission.current_index, g_sm.mission.total_waypoints);
    printf(ANSI_YELLOW "[DIAG] ===================================\r\n\r\n" ANSI_RESET);
    Console_ShowTestMenu();
}

// Run all tests sequentially
void Diag_TestAll(void)
{
    printf("\r\n");
    printf(ANSI_CYAN "╔════════════════════════════════════════╗\r\n");
    printf("║     COMPREHENSIVE SYSTEM DIAGNOSTICS   ║\r\n");
    printf("╚════════════════════════════════════════╝\r\n\r\n" ANSI_RESET);

    printf(ANSI_YELLOW "[DIAG] 1/5 - GPS Test\r\n" ANSI_RESET);
    Diag_TestGPS();
    HAL_Delay(1000);

    printf(ANSI_YELLOW "[DIAG] 2/5 - IMU Test\r\n" ANSI_RESET);
    Diag_TestIMU();
    HAL_Delay(1000);

    printf(ANSI_YELLOW "[DIAG] 3/5 - Proximity Test\r\n" ANSI_RESET);
    Diag_TestProximity();
    HAL_Delay(1000);

    printf(ANSI_YELLOW "[DIAG] 4/5 - State Info\r\n" ANSI_RESET);
    Diag_PrintState();
    HAL_Delay(1000);

    printf(ANSI_GREEN "[DIAG] All basic tests complete!\r\n" ANSI_RESET);
    printf(ANSI_CYAN "[DIAG] Use TEST MOTOR, TEST SALT, TEST BRINE for hardware tests\r\n\r\n" ANSI_RESET);
}

// Display Sabertooth motor feedback (battery voltage, current, temperature)
// Live Sabertooth feedback monitor (continuous polling with ESC to exit)
void Diag_MonitorSabertoothFeedback(void)
{
    extern UART_HandleTypeDef huart2;
    
    printf(ANSI_YELLOW "[DIAG] ===== SABERTOOTH FEEDBACK MONITOR =====\r\n" ANSI_RESET);
    printf("[DIAG] Requesting feedback from motor controller...\r\n");
    printf("[DIAG] Press ESC to exit monitor\r\n\r\n");
    
    uint32_t last_display_ms = HAL_GetTick();
    
    while (1)
    {
        Diag_WatchdogKick();
        // Check for ESC key
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
            if (ch == 0x1B) {
                printf(ANSI_YELLOW "\r\n[DIAG] ESC pressed. Exiting feedback monitor.\r\n" ANSI_RESET);
                break;
            }
        }
        if (Console_CheckEscPressed()) {
            printf(ANSI_YELLOW "\r\n[DIAG] ESC pressed. Exiting feedback monitor.\r\n" ANSI_RESET);
            break;
        }
        
        // Poll feedback from Sabertooth periodically
        Sabertooth_PollFeedback();
        
        // Display feedback every 500ms
        uint32_t now_ms = HAL_GetTick();
        if ((now_ms - last_display_ms) >= 500)
        {
            last_display_ms = now_ms;
            
            // Display with newline to keep feedback messages separated
            printf("[DIAG] Battery: %5.1f V | M1: %5.1f A %3d C | M2: %5.1f A %3d C\r\n",
                   Sabertooth_GetBatteryVoltage(),
                   Sabertooth_GetMotorCurrent(1), Sabertooth_GetTemperature(1),
                   Sabertooth_GetMotorCurrent(2), Sabertooth_GetTemperature(2));
            fflush(stdout);
        }
        
        HAL_Delay(10);
    }
    
    Console_ShowTestMenu();
}

// ============================================================================
// SYSTEM HEALTH DIAGNOSTIC
// ============================================================================

void Diag_SystemHealth(void)
{
    printf("\033[2J\033[H");  // Clear screen
    printf(ANSI_GREEN "========== SYSTEM HEALTH STATUS ==========\r\n" ANSI_RESET);
    
    // Get current health state (pointer)
    SystemHealthState_t *state_ptr = SystemHealth_GetState();
    if (!state_ptr) {
        printf(ANSI_RED "ERROR: Cannot retrieve health state!\r\n" ANSI_RESET);
        return;
    }
    SystemHealthState_t state = *state_ptr;  // Dereference pointer
    
    // Display emergency stop status
    printf("\r\n");
    if (state.emergency_stop_active) {
        printf(ANSI_RED "⚠️  EMERGENCY STOP: ACTIVE\r\n" ANSI_RESET);
    } else {
        printf(ANSI_GREEN "✓ EMERGENCY STOP: Inactive\r\n" ANSI_RESET);
    }
    
    // Display individual sensor status
    printf("\r\n" ANSI_YELLOW "Sensor Status:\r\n" ANSI_RESET);
    
    const char *sensor_names[] = {
        "IMU",
        "GPS",
        "Proximity-Left",
        "Proximity-Right",
        "LoRA",
        "Sabertooth"
    };
    
    const char *status_strings[] = {
        "OK",
        "TIMEOUT",
        "INVALID",
        "DEGRADED"
    };
    
    const char *status_colors[] = {
        ANSI_GREEN,
        ANSI_RED,
        ANSI_RED,
        ANSI_YELLOW
    };
    
    for (int i = 0; i < SENSOR_COUNT; i++) {
        SensorHealth_t status = state.sensor_status[i];
        printf("  %s\t%s%-10s%s\n",
               sensor_names[i],
               status_colors[status],
               status_strings[status],
               ANSI_RESET);
    }
    
    // Display overall health summary
    printf("\r\n" ANSI_CYAN "System Summary:\r\n" ANSI_RESET);
    printf("  Last Healthy: %lu ms ago\r\n", HAL_GetTick() - state.last_healthy_tick_ms);
    printf("  Error Count: %lu\r\n", state.error_count);
    printf("  Degraded Sensors: %u\r\n", state.degraded_sensor_flags);
    
    // Check if any sensors are unhealthy
    uint8_t any_unhealthy = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (state.sensor_status[i] != SENSOR_OK) {
            any_unhealthy = 1;
            break;
        }
    }
    
    printf("\r\n" ANSI_YELLOW "Overall Status: " ANSI_RESET);
    if (any_unhealthy) {
        printf(ANSI_RED "⚠️  SOME SENSORS UNHEALTHY\r\n" ANSI_RESET);
    } else {
        printf(ANSI_GREEN "✓ ALL SYSTEMS NOMINAL\r\n" ANSI_RESET);
    }
    
    printf(ANSI_YELLOW "=========================================\r\n" ANSI_RESET);
    printf(ANSI_CYAN "Press ESC to return to menu\r\n" ANSI_RESET);
    
    // Wait for ESC
    extern UART_HandleTypeDef huart2;
    while (1) {
        Diag_WatchdogKick();
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
            if (ch == 0x1B) {
                break;
            }
        }
        if (Console_CheckEscPressed()) {
            break;
        }
        HAL_Delay(100);
    }
    
    printf("\r\n");
}
