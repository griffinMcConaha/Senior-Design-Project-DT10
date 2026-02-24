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

// I2C timeout (ms) to avoid long stalls on bus errors
#define I2C_TIMEOUT_MS 20

// ======================================================
// Module private state
// ======================================================
static I2C_HandleTypeDef *s_hi2c = NULL;

static uint8_t s_last_ok = 0;
static uint8_t s_calibrated = 0;

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
static void ICM20948_SelectBank(uint8_t bank)
{
    uint8_t value = (uint8_t)(bank << 4);
    (void)HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, REG_BANK_SEL, 1, &value, 1, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef IMU_I2C_Read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    ICM20948_SelectBank(0);
    return HAL_I2C_Mem_Read(s_hi2c, ICM20948_ADDR, reg, 1, buf, len, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef IMU_I2C_Write(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, reg, 1, &data, 1, I2C_TIMEOUT_MS);
}

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

static void ICM20948_CheckWHOAMI(void)
{
    uint8_t who = 0;
    HAL_StatusTypeDef status;
    
    ICM20948_SelectBank(0);
    
    // Try direct I2C read without bank selection first
    status = HAL_I2C_IsDeviceReady(s_hi2c, ICM20948_ADDR, 3, I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        printf(ANSI_RED "[IMU] Device not responding to I2C address 0x%02X (HAL error: %d)\r\n" ANSI_RESET, 
               ICM20948_ADDR >> 1, status);
        return;
    }
    printf(ANSI_GREEN "[IMU] Device ACKed at address 0x%02X\r\n" ANSI_RESET, ICM20948_ADDR >> 1);
    
    status = IMU_I2C_Read(WHO_AM_I, &who, 1);
    if (status != HAL_OK)
    {
        printf(ANSI_RED "[IMU] WHO_AM_I read failed (HAL error: %d)\r\n" ANSI_RESET, status);
        return;
    }
    
    if (who == 0xEA)
    {
        printf(ANSI_GREEN "[IMU] WHO_AM_I = 0x%02X - CORRECT!\r\n" ANSI_RESET, who);
    }
    else
    {
        printf(ANSI_YELLOW "[IMU] WHO_AM_I = 0x%02X (expected 0xEA) - WRONG!\r\n" ANSI_RESET, who);
    }
}

#if IMU_USE_MAG
static HAL_StatusTypeDef AK09916_Write(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status;
    uint8_t data;

    // Select Bank 3
    ICM20948_SelectBank(3);

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
    ICM20948_SelectBank(0);
    return status;
}

// Read temperature sensor
static HAL_StatusTypeDef ICM20948_ReadTemp(float *temp_c)
{
    uint8_t raw[2];
    HAL_StatusTypeDef st;

    ICM20948_SelectBank(0);
    // Temperature registers: TEMP_OUT_H (0x39), TEMP_OUT_L (0x3A)
    st = HAL_I2C_Mem_Read(s_hi2c, ICM20948_ADDR, 0x39, 1, raw, 2, I2C_TIMEOUT_MS);
    if (st != HAL_OK) return st;

    int16_t raw_temp = (int16_t)((raw[0] << 8) | raw[1]);
    
    // Temperature formula from datasheet:
    // Temp (°C) = (TEMP_OUT / 333.87) + 21.0
    *temp_c = (raw_temp / 333.87f) + 21.0f;
    return HAL_OK;
}

static void AK09916_Init(void)
{
    uint8_t data;
    HAL_StatusTypeDef status;

    printf(ANSI_CYAN "[MAG] Initializing AK09916...\r\n" ANSI_RESET);

    // Enable I2C master
    ICM20948_SelectBank(0);
    data = 0x20; // USER_CTRL I2C_MST_EN
    status = HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, USER_CTRL, 1, &data, 1, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        printf(ANSI_RED "[MAG] Failed to enable I2C master (error: %d)\r\n" ANSI_RESET, status);
        return;
    }
    HAL_Delay(10);

    // I2C master clock
    ICM20948_SelectBank(3);
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
    ICM20948_SelectBank(3);
    
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

    ICM20948_SelectBank(0);
    printf(ANSI_GREEN "[MAG] AK09916 initialization complete (auto-read via SLV0)\r\n" ANSI_RESET);
}

static HAL_StatusTypeDef ICM20948_ReadMag(float *mx, float *my, float *mz)
{
    uint8_t raw[8];
    HAL_StatusTypeDef st;

    ICM20948_SelectBank(0);
    st = HAL_I2C_Mem_Read(s_hi2c, ICM20948_ADDR, EXT_SLV_SENS_DATA_00, 1, raw, 8, I2C_TIMEOUT_MS);
    if (st != HAL_OK) return st;

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

    ICM20948_SelectBank(0);
    st = HAL_I2C_Mem_Read(s_hi2c, ICM20948_ADDR, ACCEL_XOUT_H, 1, raw, 14, I2C_TIMEOUT_MS);
    if (st != HAL_OK) return st;

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

// ======================================================
// Public API
// ======================================================
void IMU_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    s_hi2c = hi2c;
    s_last_ok = 0;
    s_calibrated = 0;

    printf(ANSI_CYAN "[IMU] Starting ICM-20948 initialization...\r\n" ANSI_RESET);

    // --- Bank 0 ---
    ICM20948_SelectBank(0);

    // Reset device
    printf("[IMU] Resetting device...\r\n");
    status = IMU_I2C_Write(PWR_MGMT_1, 0x80);
    if (status != HAL_OK) {
        printf(ANSI_RED "[IMU] CRITICAL: Reset command failed (error: %d)\r\n" ANSI_RESET, status);
        printf(ANSI_YELLOW "[IMU] Check I2C wiring: SDA=PB9, SCL=PB6, VCC=3.3V, GND\r\n" ANSI_RESET);
        return;
    }
    HAL_Delay(100);

    // Wake, select best clock
    printf("[IMU] Waking device...\r\n");
    status = IMU_I2C_Write(PWR_MGMT_1, 0x01);
    if (status != HAL_OK) {
        printf(ANSI_RED "[IMU] CRITICAL: Wake command failed (error: %d)\r\n" ANSI_RESET, status);
        printf(ANSI_YELLOW "[IMU] Check I2C wiring: SDA=PB9, SCL=PB6, VCC=3.3V, GND\r\n" ANSI_RESET);
        return;
    }
    HAL_Delay(10);

    // Enable accel + gyro
    printf("[IMU] Enabling sensors...\r\n");
    status = IMU_I2C_Write(PWR_MGMT_2, 0x00);
    if (status != HAL_OK) {
        printf(ANSI_YELLOW "[IMU] WARNING: Failed to enable accel+gyro (error: %d)\r\n" ANSI_RESET, status);
    }

    // Disable LP duty cycling
    status = IMU_I2C_Write(LP_CONFIG, 0x00);
    if (status != HAL_OK) {
        printf(ANSI_YELLOW "[IMU] WARNING: Failed to disable low-power mode (error: %d)\r\n" ANSI_RESET, status);
    }

    // Check WHO_AM_I
    printf("[IMU] Checking WHO_AM_I...\r\n");
    ICM20948_CheckWHOAMI();

    // --- Bank 2 configs ---
    printf("[IMU] Configuring accelerometer and gyroscope...\r\n");
    ICM20948_SelectBank(2);

    // Gyro config (reg 0x01): ±250 dps + DLPF
    {
        uint8_t data = 0x03;
        status = HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, 0x01, 1, &data, 1, I2C_TIMEOUT_MS);
        if (status != HAL_OK) {
            printf(ANSI_YELLOW "[IMU] WARNING: Gyro config failed (error: %d)\r\n" ANSI_RESET, status);
        }
    }

    // Accel config (reg 0x14): ±2g + DLPF
    {
        uint8_t data = 0x13;
        status = HAL_I2C_Mem_Write(s_hi2c, ICM20948_ADDR, 0x14, 1, &data, 1, I2C_TIMEOUT_MS);
        if (status != HAL_OK) {
            printf(ANSI_YELLOW "[IMU] WARNING: Accel config failed (error: %d)\r\n" ANSI_RESET, status);
        }
    }

    ICM20948_SelectBank(0);

#if IMU_USE_MAG
    AK09916_Init();
#else
    printf(ANSI_YELLOW "[IMU] Magnetometer disabled (IMU_USE_MAG=0)\r\n" ANSI_RESET);
#endif

    printf(ANSI_GREEN "[IMU] Initialization complete (accel/gyro%s)\r\n" ANSI_RESET,
           IMU_USE_MAG ? " + mag" : "");
}

IMU_Status_t IMU_Read(void)
{
    IMU_Status_t out;
    memset(&out, 0, sizeof(out));

    if (s_hi2c == NULL)
    {
        out.ok = 0;
        return out;
    }

    HAL_StatusTypeDef st = ICM20948_ReadAccelGyro(&s_ax, &s_ay, &s_az, &s_gx, &s_gy, &s_gz);

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

    // Report health status to system health monitor
    if (st != HAL_OK) {
        SystemHealth_SetSensorStatus(SENSOR_IMU, SENSOR_TIMEOUT);
    } else if (s_temperature > 70.0f || s_temperature < -20.0f) {
        // Temperature out of safe range
        SystemHealth_SetSensorStatus(SENSOR_IMU, SENSOR_DEGRADED);
    } else {
        SystemHealth_SetSensorStatus(SENSOR_IMU, SENSOR_OK);
    }

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
            // You can break here if you want stricter behavior.
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
