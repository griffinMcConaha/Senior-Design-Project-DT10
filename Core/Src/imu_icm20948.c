/*
 * imu_icm20948.c
 * ICM-20948 IMU driver (I2C): accel, gyro, optional AK09916 magnetometer.
 *
 * Notes:
 *  - Uses I2C1 via HAL with short timeouts to avoid long stalls on bus errors.
 *  - Calibration averages samples while the board is stationary.
 *  - Magnetometer support requires IMU_USE_MAG and a board with AK09916.
 */


#include "imu_icm20948.h"
#include "console_io.h"  // For ANSI color codes
#include "system_health.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// ======================================================
// ICM-20948 register defs (moved from your main.c)
// ======================================================

// ICM-20948 Register Banks
#define REG_BANK_SEL         0x7F

// Bank 0 registers
#define WHO_AM_I             0x00
#define USER_CTRL            0x03
#define LP_CONFIG            0x05
#define PWR_MGMT_1           0x06
#define PWR_MGMT_2           0x07

// Additional Bank 0 registers
#define INT_PIN_CFG           0x0F
#define EXT_SLV_SENS_DATA_00  0x3B

// Accel + Gyro data registers (Bank 0)
#define ACCEL_XOUT_H         0x2D

// I2C address (8-bit address for HAL) - will be auto-detected
static uint8_t s_imu_addr = 0x69 << 1;  // Default to 0x69, can change to 0x68
#define ICM20948_ADDR (s_imu_addr)

// ---------------------- MAG (optional) ----------------------
#define AK09916_I2C_ADDR      0x0C
#define AK09916_WIA           0x01      // WHO_AM_I = 0x09
#define AK09916_ST1           0x10
#define AK09916_HXL           0x11
#define AK09916_CNTL2         0x31
#define AK09916_CNTL3         0x32

// Bank 3 registers for auxiliary I2C / magnetometer
#define I2C_MST_CTRL          0x01
#define I2C_SLV0_ADDR         0x03
#define I2C_SLV0_REG          0x04
#define I2C_SLV0_CTRL         0x05
#define I2C_SLV0_DO           0x06
#define I2C_SLV4_ADDR         0x13
#define I2C_SLV4_REG          0x14
#define I2C_SLV4_CTRL         0x15
#define I2C_SLV4_DO           0x16

// ======================================================
// Scale factors (your exact ones)
// ======================================================
#define ACC_SCALE   (1.0f / 16384.0f)      // ±2g => 16384 LSB/g
#define GYRO_SCALE  (1.0f / 131.0f)        // ±250 dps => 131 LSB/(°/s)
#define MAG_SCALE   (0.15f)                // µT per LSB (approx; if used)

// Optional temperature trim (disabled by default)
#define IMU_TEMP_OFFSET_ENABLE 0
#define IMU_TEMP_OFFSET_C      0.0f

// I2C timeout (ms) to avoid long stalls on bus errors
#define I2C_TIMEOUT_MS 50

// ======================================================
// Module private state
// ======================================================
static I2C_HandleTypeDef *s_hi2c = NULL;

static uint8_t s_last_ok = 0;
static uint8_t s_calibrated = 0;
static uint8_t s_init_ok = 0;  // Tracks whether IMU init succeeded
static uint32_t s_last_i2c_error_log_ms = 0;

// Recovery state machine: tracks recovery strategy progression
typedef enum {
    RECOVERY_IDLE = 0,          // No recovery needed
    RECOVERY_SOFT_RESET = 1,    // Try soft reset + re-init
    RECOVERY_FULL_REINIT = 2,   // Full I2C reinit + re-probe
    RECOVERY_ADDRESS_FALLBACK = 3,  // Try alternate addresses
    RECOVERY_BUS_UNLOCK = 4     // GPIO-based bus recovery
} RecoveryStage_t;

static RecoveryStage_t s_recovery_stage = RECOVERY_IDLE;
static uint32_t s_recovery_stage_entered_ms = 0;
static uint32_t s_recovery_attempt_count = 0;

// raw last readings (counts)
static float s_ax=0, s_ay=0, s_az=0;
static float s_gx=0, s_gy=0, s_gz=0;
static float s_temperature=0.0f;  // Temperature in Celsius
#if IMU_USE_MAG
static float s_mx=0, s_my=0, s_mz=0;
#endif

// biases in counts
static float s_acc_bias[3]  = {0};
static float s_gyro_bias[3] = {0};
#if IMU_USE_MAG
static float s_mag_bias[3]  = {0};
#endif

// ======================================================
// Private helpers
// ======================================================
static HAL_StatusTypeDef ICM20948_SelectBank(uint8_t bank)
{
    uint8_t value = (uint8_t)(bank << 4);
    return HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, REG_BANK_SEL, 1, &value, 1, I2C_TIMEOUT_MS);
}

static void IMU_LogI2CFailure(const char *op, HAL_StatusTypeDef status)
{
    if (s_hi2c == NULL) {
        return;
    }

    const uint32_t now = HAL_GetTick();
    if ((now - s_last_i2c_error_log_ms) < 200u) {
        return;
    }
    s_last_i2c_error_log_ms = now;

    const uint32_t err = s_hi2c->ErrorCode;
    printf(ANSI_RED "[IMU][I2C] %s failed: status=%d err=0x%08lX" ANSI_RESET,
           op,
           (int)status,
           (unsigned long)err);
    if (err != HAL_I2C_ERROR_NONE) {
        printf(ANSI_RED " flags:" ANSI_RESET);
        if (err & HAL_I2C_ERROR_BERR)   printf(" BERR");
        if (err & HAL_I2C_ERROR_ARLO)   printf(" ARLO");
        if (err & HAL_I2C_ERROR_AF)     printf(" AF");
        if (err & HAL_I2C_ERROR_OVR)    printf(" OVR");
#ifdef HAL_I2C_ERROR_DMA
        if (err & HAL_I2C_ERROR_DMA)    printf(" DMA");
#endif
#ifdef HAL_I2C_ERROR_TIMEOUT
        if (err & HAL_I2C_ERROR_TIMEOUT) printf(" TIMEOUT");
#endif
    }
    printf("\r\n");
}

static HAL_StatusTypeDef IMU_ReinitializeI2C(const char *reason)
{
    if (s_hi2c == NULL) {
        return HAL_ERROR;
    }

    if (reason != NULL && reason[0] != '\0') {
        printf(ANSI_YELLOW "[IMU][I2C] Reinit: %s\r\n" ANSI_RESET, reason);
    }

    // First attempt: standard HAL reinit
    HAL_I2C_DeInit(s_hi2c);
    HAL_Delay(5);
    
    // Clear any lingering error flags
    s_hi2c->ErrorCode = 0;
    
    HAL_StatusTypeDef st = HAL_I2C_Init(s_hi2c);
    HAL_Delay(5);
    
    if (st != HAL_OK) {
        IMU_LogI2CFailure("HAL_I2C_Init", st);
        
        // Last resort: attempt GPIO-based bus unlock if available
        printf(ANSI_RED "[IMU][I2C] ALERT: Standard reinit failed, attempting GPIO bus unlock\r\n" ANSI_RESET);
        
        // Hardware-specific: STM32F4 I2C1 uses PB6 (SCL), PB7 (SDA)
        // More aggressive recovery: check SDA first, then clock SCL
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // Open-drain mode
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        
        __HAL_RCC_GPIOB_CLK_ENABLE();
        
        // Configure both PB6 (SCL) and PB7 (SDA) as GPIO outputs (open-drain)
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        
        // Check if SDA is stuck low - if so, we have a larger problem
        GPIO_PinState sda_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
        GPIO_PinState scl_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
        printf(ANSI_YELLOW "[IMU][I2C] Pre-recovery: SCL=%d, SDA=%d\r\n" ANSI_RESET, (int)scl_state, (int)sda_state);
        
        // If SDA stuck low, try to release it with SCL pulses
        if (sda_state == GPIO_PIN_RESET) {
            printf(ANSI_RED "[IMU][I2C] WARNING: SDA stuck low! Attempting aggressive recovery...\r\n" ANSI_RESET);
            
            // Set SCL low to create START condition
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  
            HAL_Delay(5);
            
            // Release SCL (open-drain will be pulled high by resistor)
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);    
            HAL_Delay(5);
            
            // Now send 16 SCL pulses instead of 9
            printf(ANSI_YELLOW "[IMU][I2C] Sending 16 SCL pulses to recover stuck SDA...\r\n" ANSI_RESET);
            for (int i = 0; i < 16; i++) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SCL low
                HAL_Delay(2);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);    // SCL high
                HAL_Delay(2);
            }
            
            // Send STOP condition (SCL high, then SDA high)
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
            HAL_Delay(2);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            HAL_Delay(2);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
            HAL_Delay(5);
        } else {
            // Normal case: just send 9 SCL pulses
            printf(ANSI_YELLOW "[IMU][I2C] Sending 9 SCL pulses to unlock bus...\r\n" ANSI_RESET);
            for (int i = 0; i < 9; i++) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SCL low
                HAL_Delay(1);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);    // SCL high
                HAL_Delay(1);
            }
        }
        
        // Check recovery result
        sda_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
        scl_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
        printf(ANSI_YELLOW "[IMU][I2C] Post-recovery: SCL=%d, SDA=%d\r\n" ANSI_RESET, (int)scl_state, (int)sda_state);
        
        // Release GPIO back to I2C peripheral
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
        
        // Retry HAL_I2C_Init after GPIO recovery
        st = HAL_I2C_Init(s_hi2c);
        HAL_Delay(5);
        
        if (st == HAL_OK) {
            printf(ANSI_GREEN "[IMU][I2C] GPIO bus recovery succeeded\r\n" ANSI_RESET);
        } else {
            printf(ANSI_RED "[IMU][I2C] GPIO bus recovery failed too (error: %d)\r\n" ANSI_RESET, st);
        }
    }
    
    return st;
}

static HAL_StatusTypeDef IMU_I2C_Read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef st = ICM20948_SelectBank(0);
    if (st != HAL_OK) {
        IMU_LogI2CFailure("SelectBank(0)", st);
        return st;
    }
    st = HAL_I2C_Mem_Read(s_hi2c, ICM20948_ADDR, reg, 1, buf, len, I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        IMU_LogI2CFailure("Mem_Read", st);
    }
    return st;
}

static HAL_StatusTypeDef IMU_I2C_Write(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, reg, 1, &data, 1, I2C_TIMEOUT_MS);
}

// Early diagnostic: scan I2C bus for ANY responding devices
// Call this before probe to understand bus health
static void IMU_DiagnosticBusScan(void)
{
    printf(ANSI_CYAN "[IMU][DIAG] ═══ I2C Bus Diagnostic Scan ═══\r\n" ANSI_RESET);
    printf(ANSI_CYAN "[IMU][DIAG] Scanning 0x00-0x7F on I2C1...\r\n" ANSI_RESET);
    
    uint32_t devices_found = 0;
    uint8_t target_addrs[] = {0x68, 0x69, 0x0C};  // ICM 0x68, ICM alt 0x69, AK09916 mag 0x0C
    
    // First check target addresses with more detail
    for (size_t i = 0; i < sizeof(target_addrs); i++) {
        uint8_t addr_7bit = target_addrs[i];
        uint8_t test_addr = addr_7bit << 1;
        uint8_t dummy = 0;
        
        HAL_StatusTypeDef st = HAL_I2C_Mem_Read(s_hi2c, test_addr, 0x00, 1, &dummy, 1, I2C_TIMEOUT_MS);
        
        if (st == HAL_OK) {
            printf(ANSI_GREEN "[IMU][DIAG] ✓ Address 0x%02X ACK (data=0x%02X) - RESPONSIVE\r\n" ANSI_RESET, 
                   addr_7bit, dummy);
            devices_found++;
        } else {
            printf(ANSI_YELLOW "[IMU][DIAG] ✗ Address 0x%02X: status=%d err=0x%lX\r\n" ANSI_RESET, 
                   addr_7bit, (int)st, (unsigned long)s_hi2c->ErrorCode);
        }
    }
    
    // Scan all addresses quickly (reduced output)
    printf(ANSI_CYAN "[IMU][DIAG] Full I2C scan (0x00-0x7F)...\r\n" ANSI_RESET);
    for (uint8_t addr_7bit = 0x00; addr_7bit < 0x80; addr_7bit++) {
        uint8_t test_addr = (uint8_t)(addr_7bit << 1);
        uint8_t dummy = 0;
        
        HAL_StatusTypeDef st = HAL_I2C_Mem_Read(s_hi2c, test_addr, 0x00, 1, &dummy, 1, I2C_TIMEOUT_MS);
        
        if (st == HAL_OK) {
            printf(ANSI_GREEN "[IMU][DIAG]   Found: 0x%02X\r\n" ANSI_RESET, addr_7bit);
            devices_found++;
        }
    }
    
    if (devices_found == 0) {
        printf(ANSI_RED "[IMU][DIAG] ✗ NO I2C devices found!\r\n" ANSI_RESET);
        printf(ANSI_RED "[IMU][DIAG] CHECK: I2C power, SCL/SDA wiring, pull-up resistors\r\n" ANSI_RESET);
        printf(ANSI_RED "[IMU][DIAG] CHECK: I2C1 enabled in MX_I2C1_Init(), GPIO clocks enabled\r\n" ANSI_RESET);
    } else {
        printf(ANSI_GREEN "[IMU][DIAG] ✓ Found %lu device(s) on I2C bus\r\n" ANSI_RESET, devices_found);
    }
}

static HAL_StatusTypeDef IMU_ProbeAndSelectAddress(void)
{
    // Use a WHO_AM_I register read instead of HAL_I2C_IsDeviceReady.
    // HAL_I2C_IsDeviceReady generates a naked address-only frame which fails
    // on STM32F4 HAL even when the device is present; Mem_Read uses the same
    // I2C path as normal operation and also verifies the device ID.
    static const uint8_t candidate_addrs[] = {0x69u, 0x68u};
    
    printf(ANSI_CYAN "[IMU] Probing for device on I2C bus...\r\n" ANSI_RESET);
    
    for (uint32_t attempt = 0; attempt < 5u; ++attempt) {
        for (size_t i = 0; i < (sizeof(candidate_addrs) / sizeof(candidate_addrs[0])); ++i) {
            uint8_t addr_7bit = candidate_addrs[i];
            uint8_t test_addr = (uint8_t)(addr_7bit << 1);
            uint8_t who = 0;
            
            printf("[IMU] Attempt %lu: probing address 0x%02X...\r\n", attempt + 1, addr_7bit);
            
            HAL_StatusTypeDef st = HAL_I2C_Mem_Read(
                s_hi2c, test_addr, WHO_AM_I, 1, &who, 1, I2C_TIMEOUT_MS);
                
            if (st == HAL_OK && who == 0xEAu) {
                s_imu_addr = test_addr;
                printf(ANSI_GREEN "[IMU] ✓ Device found at 0x%02X (WHO_AM_I=0x%02X)\r\n" ANSI_RESET, 
                       addr_7bit, who);
                return HAL_OK;
            }
            
            if (st != HAL_OK) {
                printf(ANSI_YELLOW "[IMU]   I2C error at 0x%02X: status=%d (err=0x%08lX)\r\n" ANSI_RESET, 
                       addr_7bit, (int)st, (unsigned long)s_hi2c->ErrorCode);
            } else {
                printf(ANSI_YELLOW "[IMU]   WHO_AM_I mismatch: got 0x%02X (expected 0xEA)\r\n" ANSI_RESET, who);
            }
        }

        // Clear error flags and retry
        if (attempt < 4u) {
            s_hi2c->ErrorCode = 0;
            printf(ANSI_YELLOW "[IMU] Retrying probe (attempt %lu/5)...\r\n" ANSI_RESET, attempt + 2);
            
            // Progressively more aggressive reinits
            if (attempt < 2u) {
                HAL_Delay(50);
            } else if (attempt < 4u) {
                (void)IMU_ReinitializeI2C("probe retry with reinit");
            }
        }
    }

    // If we get here, device was NOT found. Run diagnostic scan to help debug
    printf(ANSI_RED "[IMU] ✗ Device not found at 0x68 or 0x69 after 5 attempts\r\n" ANSI_RESET);
    IMU_DiagnosticBusScan();
    return HAL_ERROR;

// Set IMU I2C address (must be called before IMU_Init)
void IMU_SetAddress(uint8_t addr_7bit)
{
    s_imu_addr = addr_7bit << 1;
}

// Get current IMU I2C address (7-bit)
uint8_t IMU_GetAddress(void)
{
    return s_imu_addr >> 1;
}

#if IMU_USE_MAG
static HAL_StatusTypeDef AK09916_Write(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status;
    uint8_t data;

    // Select Bank 3
    status = ICM20948_SelectBank(3);
    if (status != HAL_OK) {
        IMU_LogI2CFailure("SelectBank(3)", status);
        return status;
    }

    // 1) Set AK addr for write
    data = AK09916_I2C_ADDR; // write
    status = HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, I2C_SLV4_ADDR, 1, &data, 1, I2C_TIMEOUT_MS);
    if (status != HAL_OK) goto out;

    // 2) Target reg
    data = reg;
    status = HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, I2C_SLV4_REG, 1, &data, 1, I2C_TIMEOUT_MS);
    if (status != HAL_OK) goto out;

    // 3) Data
    data = value;
    status = HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, I2C_SLV4_DO, 1, &data, 1, I2C_TIMEOUT_MS);
    if (status != HAL_OK) goto out;

    // 4) Start transaction
    data = 0x80;
    status = HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, I2C_SLV4_CTRL, 1, &data, 1, I2C_TIMEOUT_MS);
    if (status != HAL_OK) goto out;

    HAL_Delay(10);

out:
    (void)ICM20948_SelectBank(0);
    return status;
}

// Read temperature sensor
static HAL_StatusTypeDef ICM20948_ReadTemp(float *temp_c)
{
    uint8_t raw[2];
    HAL_StatusTypeDef st;

    st = ICM20948_SelectBank(0);
    if (st != HAL_OK) {
        IMU_LogI2CFailure("SelectBank(0)", st);
        return st;
    }
    // Temperature registers: TEMP_OUT_H (0x39), TEMP_OUT_L (0x3A)
    st = HAL_I2C_Mem_Read(s_hi2c, ICM20948_ADDR, 0x39, 1, raw, 2, I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        IMU_LogI2CFailure("Temp read", st);
        return st;
    }

    int16_t raw_temp = (int16_t)((raw[0] << 8) | raw[1]);
    
    // Temperature formula from datasheet:
    // Temp (°C) = (TEMP_OUT / 333.87) + 21.0
    float temp = (raw_temp / 333.87f) + 21.0f;
#if IMU_TEMP_OFFSET_ENABLE
    temp += IMU_TEMP_OFFSET_C;
#endif
    *temp_c = temp;
    return HAL_OK;
}

static void AK09916_Init(void)
{
    uint8_t data;
    HAL_StatusTypeDef status;

    printf(ANSI_CYAN "[MAG] Initializing AK09916...\r\n" ANSI_RESET);

    // Enable I2C master
    status = ICM20948_SelectBank(0);
    if (status != HAL_OK) {
        IMU_LogI2CFailure("SelectBank(0)", status);
        return;
    }
    data = 0x20; // USER_CTRL I2C_MST_EN
    status = HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, USER_CTRL, 1, &data, 1, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        printf(ANSI_RED "[MAG] Failed to enable I2C master (error: %d)\r\n" ANSI_RESET, status);
        return;
    }
    HAL_Delay(10);

    // I2C master clock
    status = ICM20948_SelectBank(3);
    if (status != HAL_OK) {
        IMU_LogI2CFailure("SelectBank(3)", status);
        return;
    }
    data = 0x07;
    status = HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, I2C_MST_CTRL, 1, &data, 1, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        printf(ANSI_RED "[MAG] Failed to set I2C master clock (error: %d)\r\n" ANSI_RESET, status);
        return;
    }

    // Reset mag
    status = AK09916_Write(AK09916_CNTL3, 0x01);
    if (status != HAL_OK) {
        printf(ANSI_RED "[MAG] Failed to reset magnetometer (error: %d)\r\n" ANSI_RESET, status);
        return;
    }
    HAL_Delay(50);
    printf(ANSI_GREEN "[MAG] Magnetometer reset successful\r\n" ANSI_RESET);

    // Set continuous mode 100 Hz (CNTL2: 0x08 = mode 1, 10Hz; 0x09 = mode 2, 100Hz)
    // Using 0x09 for 100 Hz continuous measurement
    status = AK09916_Write(AK09916_CNTL2, 0x09);
    if (status != HAL_OK) {
        printf(ANSI_RED "[MAG] Failed to set continuous mode (error: %d)\r\n" ANSI_RESET, status);
        return;
    }
    HAL_Delay(50);
    printf(ANSI_GREEN "[MAG] Continuous measurement mode enabled (100 Hz)\r\n" ANSI_RESET);

    // Set up SLV0 auto read of 8 bytes from ST1
    // IMPORTANT: Must be in Bank 3 to write SLV0 registers
    status = ICM20948_SelectBank(3);
    if (status != HAL_OK) {
        IMU_LogI2CFailure("SelectBank(3)", status);
        return;
    }
    
    uint8_t addr = 0x19; // (0x0C<<1 | 1) - AK09916 read address
    status = HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, I2C_SLV0_ADDR, 1, &addr, 1, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        printf(ANSI_RED "[MAG] Failed to set SLV0 address (error: %d)\r\n" ANSI_RESET, status);
        return;
    }

    data = AK09916_ST1;  // Start at status register
    status = HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, I2C_SLV0_REG, 1, &data, 1, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        printf(ANSI_RED "[MAG] Failed to set SLV0 register (error: %d)\r\n" ANSI_RESET, status);
        return;
    }

    data = 0x88; // enable SLV0, read length=8 bytes
    status = HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, I2C_SLV0_CTRL, 1, &data, 1, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        printf(ANSI_RED "[MAG] Failed to enable SLV0 (error: %d)\r\n" ANSI_RESET, status);
        return;
    }

    (void)ICM20948_SelectBank(0);
    printf(ANSI_GREEN "[MAG] AK09916 initialization complete (auto-read via SLV0)\r\n" ANSI_RESET);
}

static HAL_StatusTypeDef ICM20948_ReadMag(float *mx, float *my, float *mz)
{
    uint8_t raw[8];
    HAL_StatusTypeDef st;

    st = ICM20948_SelectBank(0);
    if (st != HAL_OK) {
        IMU_LogI2CFailure("SelectBank(0)", st);
        return st;
    }
    st = HAL_I2C_Mem_Read(s_hi2c, ICM20948_ADDR, EXT_SLV_SENS_DATA_00, 1, raw, 8, I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        IMU_LogI2CFailure("Mag read", st);
        return st;
    }

    // raw[0]=ST1 (data status), raw[1-6]=mag data (little-endian), raw[7]=ST2
    // AK09916 data format: HXL(1), HXH(2), HYL(3), HYH(4), HZL(5), HZH(6)
    int16_t imx = (int16_t)((raw[2] << 8) | raw[1]);
    int16_t imy = (int16_t)((raw[4] << 8) | raw[3]);
    int16_t imz = (int16_t)((raw[6] << 8) | raw[5]);

    *mx = (float)imx;
    *my = (float)imy;
    *mz = (float)imz;
    
    return HAL_OK;
}
#endif

static HAL_StatusTypeDef ICM20948_ReadAccelGyro(float *ax, float *ay, float *az,
                                               float *gx, float *gy, float *gz)
{
    uint8_t raw[14];
    HAL_StatusTypeDef st;

    st = ICM20948_SelectBank(0);
    if (st != HAL_OK) {
        IMU_LogI2CFailure("SelectBank(0)", st);
        return st;
    }
    st = HAL_I2C_Mem_Read(s_hi2c, ICM20948_ADDR, ACCEL_XOUT_H, 1, raw, 14, I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        IMU_LogI2CFailure("AccelGyro read", st);
        return st;
    }

    int16_t iax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t iay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t iaz = (int16_t)((raw[4] << 8) | raw[5]);

    int16_t igx = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t igy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t igz = (int16_t)((raw[12] << 8) | raw[13]);

    *ax = (float)iax; *ay = (float)iay; *az = (float)iaz;
    *gx = (float)igx; *gy = (float)igy; *gz = (float)igz;
    return HAL_OK;
}

// Read accel/gyro with one short retry to tolerate transient I2C glitches.
static HAL_StatusTypeDef ICM20948_ReadAccelGyro_Retry(float *ax, float *ay, float *az,
                                                      float *gx, float *gy, float *gz)
{
    HAL_StatusTypeDef st = ICM20948_ReadAccelGyro(ax, ay, az, gx, gy, gz);
    if (st == HAL_OK) return HAL_OK;

    HAL_Delay(2);
    st = ICM20948_ReadAccelGyro(ax, ay, az, gx, gy, gz);
    if (st == HAL_OK) return HAL_OK;

    (void)IMU_ReinitializeI2C("read retry exhausted");
    HAL_Delay(2);
    st = ICM20948_ReadAccelGyro(ax, ay, az, gx, gy, gz);
    return st;
}

// ======================================================
// Public API
// ======================================================
void IMU_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    s_hi2c = hi2c;
    s_last_ok = 0;
    s_calibrated = 0;
    s_init_ok = 0;

    printf(ANSI_CYAN "[IMU] ═══════ Starting ICM-20948 Initialization ═══════\r\n" ANSI_RESET);

    // Critical: ensure I2C is in clean state before probe
    (void)IMU_ReinitializeI2C("pre-init (probe cleanup)");
    HAL_Delay(10);
    
    // Run early bus diagnostic to understand what's on the I2C bus
    IMU_DiagnosticBusScan();
    HAL_Delay(100);

    // Probe and locate device
    if (IMU_ProbeAndSelectAddress() != HAL_OK) {
        printf(ANSI_RED "[IMU] FATAL: ICM-20948 device probe failed\r\n" ANSI_RESET);
        return;
    }

    // --- Bank 0 initialization ---
    status = ICM20948_SelectBank(0);
    if (status != HAL_OK) {
        IMU_LogI2CFailure("SelectBank(0) in init", status);
        return;
    }

    // Reset device with extended delay to ensure clean state
    printf("[IMU] Step 1/7: Resetting device...\r\n");
    status = IMU_I2C_Write(PWR_MGMT_1, 0x80);
    if (status != HAL_OK) {
        printf(ANSI_YELLOW "[IMU] WARNING: Reset command failed (error: %d) - continuing anyway\r\n" ANSI_RESET, status);
    }
    HAL_Delay(150);  // Increased from 100ms to ensure full reset

    // Wake with PLL clock source
    printf("[IMU] Step 2/7: Waking device with PLL...\r\n");
    status = IMU_I2C_Write(PWR_MGMT_1, 0x01);  // 0x01 = wake + PLL clock source
    if (status != HAL_OK) {
        printf(ANSI_YELLOW "[IMU] WARNING: Wake command failed (error: %d) - continuing anyway\r\n" ANSI_RESET, status);
    }
    HAL_Delay(50);  // Increased from 10ms

    // Enable accel + gyro (clear any sleep bits)
    printf("[IMU] Step 3/7: Enabling accelerometer and gyroscope...\r\n");
    status = IMU_I2C_Write(PWR_MGMT_2, 0x00);  // 0x00 = enable all sensors
    if (status != HAL_OK) {
        printf(ANSI_YELLOW "[IMU] WARNING: Failed to enable accel+gyro (error: %d)\r\n" ANSI_RESET, status);
    }

    // Disable LP duty cycling for continuous operation
    printf("[IMU] Step 4/7: Disabling low-power cycling...\r\n");
    status = IMU_I2C_Write(LP_CONFIG, 0x00);
    if (status != HAL_OK) {
        printf(ANSI_YELLOW "[IMU] WARNING: Failed to disable low-power mode (error: %d)\r\n" ANSI_RESET, status);
    }

    // Verify device ID before proceeding
    printf("[IMU] Step 5/7: Verifying device ID (WHO_AM_I)...\r\n");
    {
        uint8_t who = 0;
        HAL_StatusTypeDef who_status = IMU_I2C_Read(WHO_AM_I, &who, 1);
        if (who_status != HAL_OK || who != 0xEA) {
            printf(ANSI_RED "[IMU] FAILED: WHO_AM_I %s (value=0x%02X, error=%d)\r\n" ANSI_RESET,
                   (who_status == HAL_OK) ? "mismatch" : "read failed",
                   who,
                   who_status);
            s_init_ok = 0;
            return;
        }
        printf(ANSI_GREEN "[IMU]   ✓ WHO_AM_I = 0x%02X (device verified!)\r\n" ANSI_RESET, who);
    }

    // --- Bank 2: Sensor configuration ---
    printf("[IMU] Step 6/7: Configuring accelerometer and gyroscope...\r\n");
    status = ICM20948_SelectBank(2);
    if (status != HAL_OK) {
        IMU_LogI2CFailure("SelectBank(2) in config", status);
        return;
    }

    // Gyro config (reg 0x01): ±250 dps + digital low-pass filter
    {
        uint8_t data = 0x03;  // GYRO_FS_SEL=0 (±250dps), GYRO_DLPF=3 (10Hz LPF)
        status = HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, 0x01, 1, &data, 1, I2C_TIMEOUT_MS);
        if (status != HAL_OK) {
            printf(ANSI_YELLOW "[IMU] WARNING: Gyro config failed (error: %d)\r\n" ANSI_RESET, status);
        } else {
            printf("[IMU]   ✓ Gyro: ±250 dps, DLPF enabled\r\n");
        }
    }

    // Accel config (reg 0x14): ±2g + digital low-pass filter
    {
        uint8_t data = 0x13;  // ACCEL_FS_SEL=0 (±2g), ACCEL_DLPF=3 (10Hz LPF)
        status = HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, 0x14, 1, &data, 1, I2C_TIMEOUT_MS);
        if (status != HAL_OK) {
            printf(ANSI_YELLOW "[IMU] WARNING: Accel config failed (error: %d)\r\n" ANSI_RESET, status);
        } else {
            printf("[IMU]   ✓ Accel: ±2g, DLPF enabled\r\n");
        }
    }

    // Return to Bank 0
    status = ICM20948_SelectBank(0);
    if (status != HAL_OK) {
        IMU_LogI2CFailure("SelectBank(0) return", status);
        return;
    }

#if IMU_USE_MAG
    printf("[IMU] Step 7/7: Initializing magnetometer...\r\n");
    AK09916_Init();
#else
    printf(ANSI_YELLOW "[IMU] Step 7/7: Magnetometer disabled (IMU_USE_MAG=0)\r\n" ANSI_RESET);
#endif

    // Final verification: read live data to ensure everything works
    printf("[IMU] Post-init verification: reading sensor data...\r\n");
    {
        float tax = 0, tay = 0, taz = 0, tgx = 0, tgy = 0, tgz = 0;
        HAL_StatusTypeDef sample_status = ICM20948_ReadAccelGyro_Retry(&tax, &tay, &taz, &tgx, &tgy, &tgz);
        if (sample_status != HAL_OK) {
            printf(ANSI_RED "[IMU] FAILED: post-init sample read unsuccessful (error: %d)\r\n" ANSI_RESET, sample_status);
            s_init_ok = 0;
            s_last_ok = 0;
            return;
        }
        printf(ANSI_GREEN "[IMU]   ✓ Sample read OK (ax=%.0f ay=%.0f az=%.0f)\r\n" ANSI_RESET, tax, tay, taz);
    }

    // Mark initialization as successful
    s_init_ok = 1;
    s_last_ok = 1;
    printf(ANSI_GREEN "[IMU] ═══════ Initialization Complete ═══════\r\n" ANSI_RESET);
    printf(ANSI_GREEN "[IMU] Device operational (accel/gyro%s)\r\n" ANSI_RESET,
           IMU_USE_MAG ? " + mag" : "");
}

IMU_Status_t IMU_Read(void)
{
    IMU_Status_t out;
    memset(&out, 0, sizeof(out));

    if (s_hi2c == NULL || !s_init_ok)
    {
        // Not initialized: do not attempt I2C reads on an unconfigured device.
        // Health status is managed exclusively through main.c's debounced imu_ok.
        out.ok = 0;
        return out;
    }

    HAL_StatusTypeDef st = ICM20948_ReadAccelGyro_Retry(&s_ax, &s_ay, &s_az, &s_gx, &s_gy, &s_gz);

    // Read temperature
    if (st == HAL_OK)
    {
        (void)ICM20948_ReadTemp(&s_temperature);
    }

#if IMU_USE_MAG
    if (st == HAL_OK)
    {
        (void)ICM20948_ReadMag(&s_mx, &s_my, &s_mz);
    }
#endif

    s_last_ok = (st == HAL_OK) ? 1u : 0u;
    out.ok = s_last_ok;

    // Health status is intentionally NOT set here.
    // The debounced imu_ok in main.c's 50 Hz block is the single source of
    // truth fed into SystemHealth_SafetyCheck, which then calls
    // SystemHealth_SetSensorStatus. Calling it here directly would bypass
    // the consecutive-failure debounce and cause false ESTOP triggers.

    // Raw
    out.raw_ax = s_ax; out.raw_ay = s_ay; out.raw_az = s_az;
    out.raw_gx = s_gx; out.raw_gy = s_gy; out.raw_gz = s_gz;

#if IMU_USE_MAG
    out.raw_mx = s_mx; out.raw_my = s_my; out.raw_mz = s_mz;
#endif

    // Bias copy
    out.acc_bias[0] = s_acc_bias[0];
    out.acc_bias[1] = s_acc_bias[1];
    out.acc_bias[2] = s_acc_bias[2];

    out.gyro_bias[0] = s_gyro_bias[0];
    out.gyro_bias[1] = s_gyro_bias[1];
    out.gyro_bias[2] = s_gyro_bias[2];

#if IMU_USE_MAG
    out.mag_bias[0] = s_mag_bias[0];
    out.mag_bias[1] = s_mag_bias[1];
    out.mag_bias[2] = s_mag_bias[2];
#endif

    // Scaled outputs (bias corrected)
    float ax = (s_ax - s_acc_bias[0]) * ACC_SCALE;
    float ay = (s_ay - s_acc_bias[1]) * ACC_SCALE;
    float az = (s_az - s_acc_bias[2]) * ACC_SCALE;

    // gyro -> deg/s -> rad/s
    float gx = (s_gx - s_gyro_bias[0]) * GYRO_SCALE * IMU_DEG2RAD;
    float gy = (s_gy - s_gyro_bias[1]) * GYRO_SCALE * IMU_DEG2RAD;
    float gz = (s_gz - s_gyro_bias[2]) * GYRO_SCALE * IMU_DEG2RAD;

    out.ax_g = ax; out.ay_g = ay; out.az_g = az;
    out.gx_rad_s = gx; out.gy_rad_s = gy; out.gz_rad_s = gz;
    out.temperature_c = s_temperature;

#if IMU_USE_MAG
    out.mx_uT = (s_mx - s_mag_bias[0]) * MAG_SCALE;
    out.my_uT = (s_my - s_mag_bias[1]) * MAG_SCALE;
    out.mz_uT = (s_mz - s_mag_bias[2]) * MAG_SCALE;
#endif

    return out;
}

void IMU_Calibrate(uint16_t samples, uint16_t delay_ms)
{
    if (samples == 0) samples = 500;
    if (delay_ms == 0) delay_ms = 5;

    int32_t sax=0, say=0, saz=0;
    int32_t sgx=0, sgy=0, sgz=0;
#if IMU_USE_MAG
    int32_t smx=0, smy=0, smz=0;
#endif

    printf("IMU calibration: keep board still and flat...\r\n");

    uint32_t start_ms = HAL_GetTick();
    uint16_t collected = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        if ((HAL_GetTick() - start_ms) >= 10000u) {
            printf("IMU calibration timeout after %u samples.\r\n", (unsigned)collected);
            break;
        }

        IMU_Status_t st = IMU_Read();
        if (!st.ok)
        {
            // If reads fail, still continue but you’ll get bad biases
        }

        sax += (int32_t)s_ax;
        say += (int32_t)s_ay;
        saz += (int32_t)s_az;

        sgx += (int32_t)s_gx;
        sgy += (int32_t)s_gy;
        sgz += (int32_t)s_gz;

#if IMU_USE_MAG
        smx += (int32_t)s_mx;
        smy += (int32_t)s_my;
        smz += (int32_t)s_mz;
#endif

        collected++;

        HAL_Delay(delay_ms);

        // Refresh watchdog during long calibration to prevent reset loops
        IWDG->KR = 0xAAAA;
    }

    if (collected == 0) collected = 1; // avoid divide-by-zero

    s_acc_bias[0] = (float)sax / (float)collected;
    s_acc_bias[1] = (float)say / (float)collected;
    s_acc_bias[2] = (float)saz / (float)collected;

    // Assume Z sees +1g during calibration (counts)
    s_acc_bias[2] -= 16384.0f;

    s_gyro_bias[0] = (float)sgx / (float)collected;
    s_gyro_bias[1] = (float)sgy / (float)collected;
    s_gyro_bias[2] = (float)sgz / (float)collected;

#if IMU_USE_MAG
    s_mag_bias[0]  = (float)smx / (float)collected;
    s_mag_bias[1]  = (float)smy / (float)collected;
    s_mag_bias[2]  = (float)smz / (float)collected;
#endif

    s_calibrated = 1;

    printf("Calibration done.\r\n");
    printf("ACC bias: %.1f, %.1f, %.1f\r\n", s_acc_bias[0], s_acc_bias[1], s_acc_bias[2]);
    printf("GYR bias: %.1f, %.1f, %.1f\r\n", s_gyro_bias[0], s_gyro_bias[1], s_gyro_bias[2]);
#if IMU_USE_MAG
    printf("MAG bias: %.1f, %.1f, %.1f\r\n", s_mag_bias[0], s_mag_bias[1], s_mag_bias[2]);
#endif
}

uint8_t IMU_IsCalibrated(void)
{
    return s_calibrated;
}

uint8_t IMU_LastOk(void)
{
    return s_last_ok;
}

void IMU_SoftReset(void)
{
    (void)IMU_I2C_Write(PWR_MGMT_1, 0x80);
    HAL_Delay(100);
    printf("IMU soft-reset issued.\r\n");
}

void IMU_RuntimeCheckWHOAMI(void)
{
    uint8_t who = 0;
    if (IMU_I2C_Read(WHO_AM_I, &who, 1) != HAL_OK)
    {
        s_last_ok = 0;
        printf("IMU WHO_AM_I read failed\r\n");
        return;
    }

    if (who != 0xEA)
    {
        s_last_ok = 0;
        printf("IMU LOST! WHO_AM_I = 0x%02X\r\n", who);
    }
}

uint8_t IMU_GetInitStatus(void)
{
    return s_init_ok;
}

// Periodic recovery check: call from main loop (~1Hz)
// If IMU isn't responding, attempt multi-stage recovery with exponential backoff
uint8_t IMU_CheckAndRecover(void)
{
    if (!s_hi2c) {
        printf(ANSI_RED "[IMU] Recovery: I2C handle is NULL\r\n" ANSI_RESET);
        return 0;
    }
    
    // If already OK and last read succeeded, no recovery needed
    if (s_init_ok && s_last_ok) {
        s_recovery_stage = RECOVERY_IDLE;
        return 1;
    }

    uint32_t now_ms = HAL_GetTick();
    
    // Transition through recovery stages with progressive backoff
    switch (s_recovery_stage) {
        
        case RECOVERY_IDLE:
            // Initial detection of failure
            printf(ANSI_YELLOW "[IMU] ⚠ Recovery: Detected IMU failure (init_ok=%u, last_ok=%u)\r\n" ANSI_RESET, 
                   s_init_ok, s_last_ok);
            s_recovery_stage = RECOVERY_SOFT_RESET;
            s_recovery_stage_entered_ms = now_ms;
            s_recovery_attempt_count = 0;
            // Fall through to attempt soft reset immediately
            
        case RECOVERY_SOFT_RESET: {
            // Stage 1: Try soft reset + re-init (fast, ~500ms)
            if ((now_ms - s_recovery_stage_entered_ms) < 100) {
                break;  // Wait a bit before attempting
            }
            
            s_recovery_attempt_count++;
            printf(ANSI_YELLOW "[IMU] Recovery Stage 1/4: Soft reset + re-init (attempt %lu)\r\n" ANSI_RESET,
                   s_recovery_attempt_count);
            
            uint8_t was_calibrated = s_calibrated;
            s_init_ok = 0;
            s_last_ok = 0;
            s_calibrated = was_calibrated;
            
            // Soft reset via register
            (void)IMU_I2C_Write(PWR_MGMT_1, 0x80);
            HAL_Delay(150);
            
            // Attempt re-initialization
            IMU_Init(s_hi2c);
            
            if (s_init_ok) {
                printf(ANSI_GREEN "[IMU] ✓ Soft reset succeeded! Device recovered.\r\n" ANSI_RESET);
                s_recovery_stage = RECOVERY_IDLE;
                return 1;
            }
            
            if (s_recovery_attempt_count >= 2) {
                // Move to hard reset if soft reset failed twice
                printf(ANSI_YELLOW "[IMU] Soft reset failed %lu times, escalating...\r\n" ANSI_RESET, 
                       s_recovery_attempt_count);
                s_recovery_stage = RECOVERY_FULL_REINIT;
                s_recovery_stage_entered_ms = now_ms;
                s_recovery_attempt_count = 0;
            }
            break;
        }
        
        case RECOVERY_FULL_REINIT: {
            // Stage 2: Full I2C reinit + re-probe (medium, ~1-2 sec)
            if ((now_ms - s_recovery_stage_entered_ms) < 500) {
                break;  // Longer wait before hard reset
            }
            
            s_recovery_attempt_count++;
            printf(ANSI_YELLOW "[IMU] Recovery Stage 2/4: Full I2C reinit (attempt %lu)\r\n" ANSI_RESET,
                   s_recovery_attempt_count);
            
            uint8_t was_calibrated = s_calibrated;
            s_init_ok = 0;
            s_last_ok = 0;
            s_calibrated = was_calibrated;
            
            // Force complete I2C reinitialization
            (void)IMU_ReinitializeI2C("hard recovery");
            HAL_Delay(100);
            
            // Reset address to default and re-probe
            s_imu_addr = 0x69 << 1;
            IMU_Init(s_hi2c);
            
            if (s_init_ok) {
                printf(ANSI_GREEN "[IMU] ✓ Full reinit succeeded! Device online.\r\n" ANSI_RESET);
                s_recovery_stage = RECOVERY_IDLE;
                return 1;
            }
            
            if (s_recovery_attempt_count >= 2) {
                printf(ANSI_YELLOW "[IMU] Full reinit failed %lu times, trying address fallback...\r\n" ANSI_RESET,
                       s_recovery_attempt_count);
                s_recovery_stage = RECOVERY_ADDRESS_FALLBACK;
                s_recovery_stage_entered_ms = now_ms;
                s_recovery_attempt_count = 0;
            }
            break;
        }
        
        case RECOVERY_ADDRESS_FALLBACK: {
            // Stage 3: Try both I2C addresses explicitly (slow, ~2-3 sec)
            if ((now_ms - s_recovery_stage_entered_ms) < 800) {
                break;  // Longer backoff
            }
            
            s_recovery_attempt_count++;
            printf(ANSI_YELLOW "[IMU] Recovery Stage 3/4: Address fallback (attempt %lu)\r\n" ANSI_RESET,
                   s_recovery_attempt_count);
            
            uint8_t was_calibrated = s_calibrated;
            s_init_ok = 0;
            s_last_ok = 0;
            s_calibrated = was_calibrated;
            
            // Try explicit address 0x68
            printf("[IMU]   Trying address 0x68...\r\n");
            IMU_SetAddress(0x68);
            IMU_Init(s_hi2c);
            if (s_init_ok) {
                printf(ANSI_GREEN "[IMU] ✓ SUCCESS at 0x68!\r\n" ANSI_RESET);
                s_recovery_stage = RECOVERY_IDLE;
                return 1;
            }
            
            // Try explicit address 0x69
            printf("[IMU]   Trying address 0x69...\r\n");
            IMU_SetAddress(0x69);
            IMU_Init(s_hi2c);
            if (s_init_ok) {
                printf(ANSI_GREEN "[IMU] ✓ SUCCESS at 0x69!\r\n" ANSI_RESET);
                s_recovery_stage = RECOVERY_IDLE;
                return 1;
            }
            
            if (s_recovery_attempt_count >= 1) {
                printf(ANSI_YELLOW "[IMU] Address fallback exhausted, attempting GPIO bus unlock...\r\n" ANSI_RESET);
                s_recovery_stage = RECOVERY_BUS_UNLOCK;
                s_recovery_stage_entered_ms = now_ms;
                s_recovery_attempt_count = 0;
            }
            break;
        }
        
        case RECOVERY_BUS_UNLOCK: {
            // Stage 4: GPIO-based bus recovery with full reinit (aggressive, ~5+ sec)
            if ((now_ms - s_recovery_stage_entered_ms) < 1000) {
                break;  // Very long backoff
            }
            
            s_recovery_attempt_count++;
            printf(ANSI_RED "[IMU] Recovery Stage 4/4: GPIO bus unlock (attempt %lu)\r\n" ANSI_RESET,
                   s_recovery_attempt_count);
            
            uint8_t was_calibrated = s_calibrated;
            s_init_ok = 0;
            s_last_ok = 0;
            s_calibrated = was_calibrated;
            
            // GPIO unlock included in IMU_ReinitializeI2C
            (void)IMU_ReinitializeI2C("GPIO bus recovery");
            HAL_Delay(200);
            
            // Reset to default address and try again
            s_imu_addr = 0x69 << 1;
            IMU_Init(s_hi2c);
            
            if (s_init_ok) {
                printf(ANSI_GREEN "[IMU] ✓ GPIO recovery succeeded!\r\n" ANSI_RESET);
                s_recovery_stage = RECOVERY_IDLE;
                return 1;
            }
            
            if (s_recovery_attempt_count >= 2) {
                printf(ANSI_RED "[IMU] ✗✗✗ All recovery stages exhausted - giving up\r\n" ANSI_RESET);
                printf(ANSI_RED "[IMU]     Check: 1) Wiring/connectors\r\n" ANSI_RESET);
                printf(ANSI_RED "[IMU]           2) Pull-up resistors on SDA/SCL\r\n" ANSI_RESET);
                printf(ANSI_RED "[IMU]           3) I2C address (0x68 vs 0x69)\r\n" ANSI_RESET);
                printf(ANSI_RED "[IMU]           4) Sensor power supply\r\n" ANSI_RESET);
                printf(ANSI_RED "[IMU]           5) Replace IMU module\r\n" ANSI_RESET);
                s_recovery_stage = RECOVERY_IDLE;  // Stop trying
                return 0;
            }
            break;
        }
        
        default:
            printf(ANSI_RED "[IMU] UNKNOWN RECOVERY STAGE: %u\r\n" ANSI_RESET, s_recovery_stage);
            s_recovery_stage = RECOVERY_IDLE;
            break;
    }

    return 0;  // Still recovering, not yet successful
}
