// Proximity sensor driver implementation
#include "proximity.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ============================================================================
// ULTRASONIC PROXIMITY SENSOR DRIVER (HC-SR04)
// Pins:
//  - PE10: Left trigger (GPIO output)
//  - PE11: Left echo (TIM1_CH2, input capture)
//  - PB0: Right trigger (GPIO output)
//  - PB1: Right echo (TIM3_CH4, input capture)
// Range: 2-400 cm typical (software capped to 250 cm for this robot)
// Timing: 10 µs trigger pulse, measure echo high time
// Distance = pulse_width_us / 58 cm (datasheet formula)
// 
// CRITICAL CLOCK FREQUENCIES:
//  - TIM1: 168 MHz / 168 = 1 MHz → 1 µs per tick
//  - TIM3: 84 MHz / 84 = 1 MHz → 1 µs per tick
//  
//  HC-SR04 datasheet formula: distance (cm) = pulse_width (µs) / 58
//  Since both timers now run at 1 MHz (1 tick = 1 µs):
//  distance (cm) = ticks / 58
//  
//  Max range with 16-bit counter: 65535 ticks / 58 = 1130 cm (way more than 400 cm needed)
// ============================================================================

// Configuration constants
#define SAMPLES_PER_READING 3      // Number of measurements to average
#define INTER_SAMPLE_DELAY_MS 10   // Delay between samples
#define TICKS_PER_CM 58            // Both timers @ 1 MHz: 58 ticks (µs) per cm
#define MAX_CHANGE_CM 350          // Max allowed change between readings (350cm jumps allowed for obstacle planning)
#define PROX_MAX_RANGE_CM 250      // Effective max range for control logic (cm)
#define PROX_MAX_ECHO_TICKS (PROX_MAX_RANGE_CM * TICKS_PER_CM + 200)  // Echo timeout + margin

// Baseline false echo rejection - reject measurements in the 77-79cm range (false echo signature)
#define BASELINE_FALSE_ECHO_MIN_CM 76   // Reject this range as it's the false echo baseline
#define BASELINE_FALSE_ECHO_MAX_CM 80

// Sensor configuration
typedef struct {
    uint8_t initialized;        // Sensor initialized flag
    uint16_t distance_left_cm;  // Last distance reading left sensor (cm)
    uint16_t distance_right_cm; // Last distance reading right sensor (cm)
    uint32_t echo_left_ticks;   // Last left echo pulse width (TIM1 ticks)
    uint32_t echo_right_ticks;  // Last right echo pulse width (TIM3 ticks)
    uint32_t last_trigger_ms;   // Last trigger time for debouncing
    
    // Median filtering (store last 5 readings)
    uint16_t left_readings[5];   // Rolling buffer for left sensor
    uint16_t right_readings[5];  // Rolling buffer for right sensor
    uint8_t left_index;          // Current index for left sensor buffer
    uint8_t right_index;         // Current index for right sensor buffer
    uint16_t last_valid_left;    // Last valid left reading for rate-of-change check
    uint16_t last_valid_right;   // Last valid right reading for rate-of-change check
} Proximity_State_t;

static Proximity_State_t prox_state = {0};
extern TIM_HandleTypeDef htim1;  // TIM1 for PE11 capture (CH2)
extern TIM_HandleTypeDef htim3;  // TIM3 for PB1 capture (CH4)

// ============================================================================
// PRIVATE FUNCTIONS
// ============================================================================

// Median filter: sort and return middle value of 5 readings
// Rejects outliers by discarding min and max
static uint16_t Proximity_MedianFilter(uint16_t *readings)
{
    // Bubble sort 5 values
    for (int i = 0; i < 5; i++) {
        for (int j = i + 1; j < 5; j++) {
            if (readings[i] > readings[j]) {
                uint16_t temp = readings[i];
                readings[i] = readings[j];
                readings[j] = temp;
            }
        }
    }
    // Return middle value (index 2) - rejects 2 lowest and 2 highest
    return readings[2];
}

// Heuristic: if recent history is mostly "no detect" (zeros), treat baseline echoes as no detect
static uint8_t Proximity_NoDetectHistory(const uint16_t *readings)
{
    uint8_t zero_count = 0;
    for (int i = 0; i < 5; i++) {
        if (readings[i] == 0) {
            zero_count++;
        }
    }
    return (zero_count >= 3);
}

// Send 10 µs trigger pulse on trigger pin
static void Proximity_SendTrigger(GPIO_TypeDef *gpio, uint16_t pin)
{
    // At 168 MHz: 1 cycle = 5.95 ns, need ~1,680 cycles for 10 µs
    HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_SET);
    for (int i = 0; i < 1700; i++) __NOP();  // ~10 µs at 168 MHz
    HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_RESET);
    // Trigger sent successfully
}

// Calculate distance from echo time in timer ticks
// Both timers now run at 1 MHz (1 tick = 1 µs), so same conversion for both
// HC-SR04 formula: distance (cm) = pulse_width (µs) / 58
static uint16_t Proximity_TicksToDistance(uint32_t ticks)
{
    if (ticks == 0)
        return 0;
    
    // Both timers @ 1 MHz: 58 ticks (µs) per cm
    uint16_t distance_cm = ticks / TICKS_PER_CM;
    
    if (distance_cm < 2)
        return 2;
    if (distance_cm > PROX_MAX_RANGE_CM)
        return PROX_NO_DETECTION;
    
    return distance_cm;
}

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

// Initialize proximity sensors with HC-SR04 GPIO and timer capture
void Proximity_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIO clocks
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    
    // ========== LEFT SENSOR: PE10 trigger, PE11 echo ==========
    // Configure PE10 as GPIO output (trigger)
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
    
    // Configure PE11 as TIM1_CH2 for echo input capture
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
    // ========== RIGHT SENSOR: PB0 trigger, PB1 echo ==========
    // Configure PB0 as GPIO output (trigger)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    
    // Configure PB1 as TIM3_CH4 for echo input capture
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // Start timer base counters and input capture
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_2);
    
    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_4);
    
    memset(&prox_state, 0, sizeof(prox_state));
    prox_state.initialized = 1;
    
    printf("[PROX] Initialized: Left (PE10 trig/PE11 echo TIM1_CH2), Right (PB0 trig/PB1 echo TIM3_CH4)\r\n");
        printf("[PROX] Both timers @ 1 MHz (prescaled), %d ticks/cm, max range %d cm\r\n", 
            TICKS_PER_CM, PROX_MAX_RANGE_CM);
    printf("[PROX] Sampling: %d samples/reading, %dms interval, median filtering enabled\r\n", 
           SAMPLES_PER_READING, INTER_SAMPLE_DELAY_MS);
}

// ============================================================================
// RAW MEASUREMENT HELPERS (single sample per call)
// ============================================================================

// Take single raw measurement from left sensor (returns pulse width in ticks, 0 on error)
static uint32_t Proximity_RawReadLeft(void)
{
    uint32_t start_time = 0, end_time = 0, pulse_width = 0;
    uint32_t timeout = 0;
    
    // Wait for echo pin to return LOW (previous pulse complete)
    timeout = 100000;
    while ((GPIOE->IDR & GPIO_PIN_11) && --timeout) { }
    if (timeout == 0)
        return 0;
    
    // Small delay to ensure echo pin is truly stable and LOW (avoid stale edges)
    for (volatile int i = 0; i < 100; i++) { }  // ~1-2 µs delay
    
    // Verify echo pin is actually LOW before proceeding
    if ((GPIOE->IDR & GPIO_PIN_11)) {
        return 0;  // Echo pin not LOW, abort measurement
    }
    
    // Reset timer counter to 0
    htim1.Instance->CNT = 0;
    
    // Send 10 µs trigger pulse
    Proximity_SendTrigger(GPIOE, GPIO_PIN_10);
    
    // Clear any pending capture flags
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC2);
    
    // Wait for rising edge (timeout based on max range)
    timeout = PROX_MAX_ECHO_TICKS;
    while (!(htim1.Instance->SR & TIM_FLAG_CC2) && --timeout) {
        if (htim1.Instance->CNT > PROX_MAX_ECHO_TICKS) break;
    }
    if (!(htim1.Instance->SR & TIM_FLAG_CC2))
        return 0;
    
    start_time = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC2);
    
    // Wait for falling edge (timeout based on max range)
    timeout = PROX_MAX_ECHO_TICKS;
    while (!(htim1.Instance->SR & TIM_FLAG_CC2) && --timeout) {
        if (htim1.Instance->CNT > (start_time + PROX_MAX_ECHO_TICKS)) break;
    }
    if (!(htim1.Instance->SR & TIM_FLAG_CC2))
        return 0;
    
    end_time = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC2);
    
    // Calculate pulse width with overflow handling
    if (end_time >= start_time) {
        pulse_width = end_time - start_time;
    } else {
        pulse_width = (0xFFFF - start_time) + end_time + 1;
    }
    
    return pulse_width;
}

// Take single raw measurement from right sensor (returns pulse width in ticks, 0 on error)
static uint32_t Proximity_RawReadRight(void)
{
    uint32_t start_time = 0, end_time = 0, pulse_width = 0;
    uint32_t timeout = 0;
    
    // Wait for echo pin to return LOW (previous pulse complete)
    timeout = 100000;
    while ((GPIOB->IDR & GPIO_PIN_1) && --timeout) { }
    if (timeout == 0)
        return 0;
    
    // Small delay to ensure echo pin is truly stable and LOW (avoid stale edges)
    for (volatile int i = 0; i < 100; i++) { }  // ~1-2 µs delay
    
    // Verify echo pin is actually LOW before proceeding
    if ((GPIOB->IDR & GPIO_PIN_1)) {
        return 0;  // Echo pin not LOW, abort measurement
    }
    
    // Reset timer counter to 0
    htim3.Instance->CNT = 0;
    
    // Send 10 µs trigger pulse
    Proximity_SendTrigger(GPIOB, GPIO_PIN_0);
    
    // Clear any pending capture flags
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC4);
    
    // Wait for rising edge (timeout based on max range)
    timeout = PROX_MAX_ECHO_TICKS;
    while (!(htim3.Instance->SR & TIM_FLAG_CC4) && --timeout) {
        if (htim3.Instance->CNT > PROX_MAX_ECHO_TICKS) break;
    }
    if (!(htim3.Instance->SR & TIM_FLAG_CC4))
        return 0;
    
    start_time = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC4);
    
    // Wait for falling edge (timeout based on max range)
    timeout = PROX_MAX_ECHO_TICKS;
    while (!(htim3.Instance->SR & TIM_FLAG_CC4) && --timeout) {
        if (htim3.Instance->CNT > (start_time + PROX_MAX_ECHO_TICKS)) break;
    }
    if (!(htim3.Instance->SR & TIM_FLAG_CC4))
        return 0;
    
    end_time = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC4);
    
    // Calculate pulse width with overflow handling
    if (end_time >= start_time) {
        pulse_width = end_time - start_time;
    } else {
        pulse_width = (0xFFFF - start_time) + end_time + 1;
    }
    
    return pulse_width;
}

// ============================================================================
// PUBLIC FUNCTIONS (multi-sample with filtering)
// ============================================================================

// Read left proximity sensor (centimeters)
// Triggers PE10, reads echo on PE11 via TIM1_CH2 using pulse width measurement
// Both-edges capture: automatically captures rising and falling edges
// Takes SAMPLES_PER_READING measurements and averages them for noise rejection
// Returns: distance in cm (2-250), or PROX_NO_DETECTION (65535) if no object detected
uint16_t Proximity_ReadLeft(void)
{
    if (!prox_state.initialized)
        return PROX_NO_DETECTION;
    
    uint32_t total_ticks = 0;
    uint8_t valid_samples = 0;
    uint32_t samples[SAMPLES_PER_READING];
    
    // Take multiple samples
    for (int i = 0; i < SAMPLES_PER_READING; i++) {
        uint32_t pulse_width = Proximity_RawReadLeft();
        
        // Validate pulse width @ 1 MHz (1 tick = 1 µs)
        // Min: 2 cm × 58 ticks/cm = 116 ticks
        // Max: PROX_MAX_RANGE_CM × 58 ticks/cm
        if (pulse_width >= 116 && pulse_width <= PROX_MAX_ECHO_TICKS) {
            samples[valid_samples] = pulse_width;
            valid_samples++;
        }
        
        // Inter-sample delay (HC-SR04 needs recovery time)
        if (i < SAMPLES_PER_READING - 1) {
            HAL_Delay(INTER_SAMPLE_DELAY_MS);
        }
    }
    
    // Check for false echo baseline signature: all 3 samples tightly clustered in 76-80cm range
    // If all 3 are in baseline range with tight consistency (<30 ticks variation), it's the false echo
    if (valid_samples == 3) {
        uint16_t min_sample = (samples[0] < samples[1]) ? samples[0] : samples[1];
        min_sample = (min_sample < samples[2]) ? min_sample : samples[2];
        uint16_t max_sample = (samples[0] > samples[1]) ? samples[0] : samples[1];
        max_sample = (max_sample > samples[2]) ? max_sample : samples[2];
        uint16_t spread = max_sample - min_sample;
        
        uint16_t min_cm = min_sample / TICKS_PER_CM;
        uint16_t max_cm = max_sample / TICKS_PER_CM;
        
        // If all samples are in baseline range AND VERY tightly clustered (<30 ticks), reject as false echo
        // Real objects will show more jitter/variation when transitioning through this range
        if (min_cm >= BASELINE_FALSE_ECHO_MIN_CM && max_cm <= BASELINE_FALSE_ECHO_MAX_CM && spread < 30) {
            // Clear the rolling buffer since we have no valid reading
            memset(prox_state.left_readings, 0, sizeof(prox_state.left_readings));
            prox_state.left_index = 0;
            prox_state.last_valid_left = 0;
            return PROX_NO_DETECTION;
        }
    }
    
    // If no valid samples, return "no detection" sentinel value
    if (valid_samples == 0) {
        // Clear the rolling buffer since we have no valid reading
        memset(prox_state.left_readings, 0, sizeof(prox_state.left_readings));
        prox_state.left_index = 0;
        prox_state.last_valid_left = 0;
        return PROX_NO_DETECTION;
    }
    
    // Outlier rejection: if we have 3 samples, remove the one furthest from median
    if (valid_samples == 3) {
        // Sort samples
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 3; j++) {
                if (samples[i] > samples[j]) {
                    uint32_t temp = samples[i];
                    samples[i] = samples[j];
                    samples[j] = temp;
                }
            }
        }
        // Use middle value only (reject min and max)
        total_ticks = samples[1];
        valid_samples = 1;
    } else {
        // Average whatever valid samples we got
        for (int i = 0; i < valid_samples; i++) {
            total_ticks += samples[i];
        }
    }
    
    // Average the valid samples
    uint32_t avg_pulse_width = total_ticks / valid_samples;
    uint16_t distance = Proximity_TicksToDistance(avg_pulse_width);

    // Treat out-of-range as no detection
    if (distance == PROX_NO_DETECTION) {
        memset(prox_state.left_readings, 0, sizeof(prox_state.left_readings));
        prox_state.left_index = 0;
        prox_state.last_valid_left = 0;
        return PROX_NO_DETECTION;
    }

    // Baseline false-echo handling: if we're mostly seeing no-detects, suppress 76-80cm echoes
    if (distance >= BASELINE_FALSE_ECHO_MIN_CM && distance <= BASELINE_FALSE_ECHO_MAX_CM) {
        if (prox_state.last_valid_left == 0 || Proximity_NoDetectHistory(prox_state.left_readings)) {
            return PROX_NO_DETECTION;
        }
    }
    
    // Rate-of-change validation: reject impossible jumps
    if (prox_state.last_valid_left > 0) {
        int change = abs((int)distance - (int)prox_state.last_valid_left);
        if (change > MAX_CHANGE_CM) {
            // Reject this reading - return last valid value instead
            return prox_state.last_valid_left;
        }
    }
    
    // Update last valid reading
    prox_state.last_valid_left = distance;
    
    // Add to rolling buffer for median filtering
    prox_state.left_readings[prox_state.left_index] = distance;
    prox_state.left_index = (prox_state.left_index + 1) % 5;
    
    // Once buffer is full (after 5 readings), use median filter
    if (prox_state.left_index == 0) {
        uint16_t temp_buf[5];
        memcpy(temp_buf, prox_state.left_readings, sizeof(temp_buf));
        uint16_t filtered = Proximity_MedianFilter(temp_buf);
        prox_state.distance_left_cm = filtered;
        prox_state.echo_left_ticks = avg_pulse_width;
        return filtered;
    } else {
        // Return averaged reading while buffer fills
        prox_state.distance_left_cm = distance;
        prox_state.echo_left_ticks = avg_pulse_width;
        return distance;
    }
}

// Read right proximity sensor (centimeters)
// Triggers PB0, reads echo on PB1 via TIM3_CH4 using pulse width measurement  
// Both-edges capture: automatically captures rising and falling edges
// Takes SAMPLES_PER_READING measurements and averages them for noise rejection
// Returns: distance in cm (2-250), or PROX_NO_DETECTION (65535) if no object detected
uint16_t Proximity_ReadRight(void)
{
    if (!prox_state.initialized)
        return PROX_NO_DETECTION;
    
    uint32_t total_ticks = 0;
    uint8_t valid_samples = 0;
    uint32_t samples[SAMPLES_PER_READING];
    
    // Take multiple samples
    for (int i = 0; i < SAMPLES_PER_READING; i++) {
        uint32_t pulse_width = Proximity_RawReadRight();
        
        // Validate pulse width @ 1 MHz (1 tick = 1 µs)
        // Min: 2 cm × 58 ticks/cm = 116 ticks
        // Max: PROX_MAX_RANGE_CM × 58 ticks/cm
        if (pulse_width >= 116 && pulse_width <= PROX_MAX_ECHO_TICKS) {
            samples[valid_samples] = pulse_width;
            valid_samples++;
        }
        
        // Inter-sample delay (HC-SR04 needs recovery time)
        if (i < SAMPLES_PER_READING - 1) {
            HAL_Delay(INTER_SAMPLE_DELAY_MS);
        }
    }
    
    // Check for false echo baseline signature: all 3 samples tightly clustered in 76-80cm range
    // If all 3 are in baseline range with tight consistency (<30 ticks variation), it's the false echo
    if (valid_samples == 3) {
        uint16_t min_sample = (samples[0] < samples[1]) ? samples[0] : samples[1];
        min_sample = (min_sample < samples[2]) ? min_sample : samples[2];
        uint16_t max_sample = (samples[0] > samples[1]) ? samples[0] : samples[1];
        max_sample = (max_sample > samples[2]) ? max_sample : samples[2];
        uint16_t spread = max_sample - min_sample;
        
        uint16_t min_cm = min_sample / TICKS_PER_CM;
        uint16_t max_cm = max_sample / TICKS_PER_CM;
        
        // If all samples are in baseline range AND VERY tightly clustered (<30 ticks), reject as false echo
        // Real objects will show more jitter/variation when transitioning through this range
        if (min_cm >= BASELINE_FALSE_ECHO_MIN_CM && max_cm <= BASELINE_FALSE_ECHO_MAX_CM && spread < 30) {
            // Clear the rolling buffer since we have no valid reading
            memset(prox_state.right_readings, 0, sizeof(prox_state.right_readings));
            prox_state.right_index = 0;
            prox_state.last_valid_right = 0;
            return PROX_NO_DETECTION;
        }
    }
    
    // If no valid samples, return "no detection" sentinel value
    if (valid_samples == 0) {
        // Clear the rolling buffer since we have no valid reading
        memset(prox_state.right_readings, 0, sizeof(prox_state.right_readings));
        prox_state.right_index = 0;
        prox_state.last_valid_right = 0;
        return PROX_NO_DETECTION;
    }
    
    // Outlier rejection: if we have 3 samples, remove the one furthest from median
    if (valid_samples == 3) {
        // Sort samples
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 3; j++) {
                if (samples[i] > samples[j]) {
                    uint32_t temp = samples[i];
                    samples[i] = samples[j];
                    samples[j] = temp;
                }
            }
        }
        // Use middle value only (reject min and max)
        total_ticks = samples[1];
        valid_samples = 1;
    } else {
        // Average whatever valid samples we got
        for (int i = 0; i < valid_samples; i++) {
            total_ticks += samples[i];
        }
    }
    
    // Average the valid samples
    uint32_t avg_pulse_width = total_ticks / valid_samples;
    uint16_t distance = Proximity_TicksToDistance(avg_pulse_width);

    // Treat out-of-range as no detection
    if (distance == PROX_NO_DETECTION) {
        memset(prox_state.right_readings, 0, sizeof(prox_state.right_readings));
        prox_state.right_index = 0;
        prox_state.last_valid_right = 0;
        return PROX_NO_DETECTION;
    }

    // Baseline false-echo handling: if we're mostly seeing no-detects, suppress 76-80cm echoes
    if (distance >= BASELINE_FALSE_ECHO_MIN_CM && distance <= BASELINE_FALSE_ECHO_MAX_CM) {
        if (prox_state.last_valid_right == 0 || Proximity_NoDetectHistory(prox_state.right_readings)) {
            return PROX_NO_DETECTION;
        }
    }
    
    // Rate-of-change validation: reject impossible jumps
    if (prox_state.last_valid_right > 0) {
        int change = abs((int)distance - (int)prox_state.last_valid_right);
        if (change > MAX_CHANGE_CM) {
            // Reject this reading - return last valid value instead
            return prox_state.last_valid_right;
        }
    }
    
    // Update last valid reading
    prox_state.last_valid_right = distance;
    
    // Add to rolling buffer for median filtering
    prox_state.right_readings[prox_state.right_index] = distance;
    prox_state.right_index = (prox_state.right_index + 1) % 5;
    
    // Once buffer is full (after 5 readings), use median filter
    if (prox_state.right_index == 0) {
        uint16_t temp_buf[5];
        memcpy(temp_buf, prox_state.right_readings, sizeof(temp_buf));
        uint16_t filtered = Proximity_MedianFilter(temp_buf);
        prox_state.distance_right_cm = filtered;
        prox_state.echo_right_ticks = avg_pulse_width;
        return filtered;
    } else {
        // Return averaged reading while buffer fills
        prox_state.distance_right_cm = distance;
        prox_state.echo_right_ticks = avg_pulse_width;
        return distance;
    }
}

// Read both proximity sensors at once (convenience function)
void Proximity_ReadBoth(uint16_t *left_cm, uint16_t *right_cm)
{
    if (!left_cm || !right_cm)
        return;
    
    *left_cm = Proximity_ReadLeft();
    *right_cm = Proximity_ReadRight();
}

// Get last cached reading without triggering new measurement (fast access for other MCUs)
uint16_t Proximity_GetLastLeft(void)
{
    return prox_state.distance_left_cm;
}

uint16_t Proximity_GetLastRight(void)
{
    return prox_state.distance_right_cm;
}

// Check if distance is in warning range (10-50 cm)
bool Proximity_WarningRange(uint16_t distance_cm)
{
    // Return true if distance in warning range, false otherwise
    return (distance_cm >= 10 && distance_cm <= 50);
}

// Check if distance is in critical range (< 10 cm)
bool Proximity_CriticalRange(uint16_t distance_cm)
{
    // Return true if distance in critical range, false otherwise
    return (distance_cm < 10 && distance_cm > 0);
}

// Get proximity status for display/control
// Returns: 0 = clear (>50cm), 1 = warning (10-50cm), 2 = critical (<10cm), 3 = error
uint8_t Proximity_GetStatus(uint16_t distance_cm)
{
    // Handle error case
    if (distance_cm == 0)
        return 3; // Error reading

    // Check ranges
    if (Proximity_CriticalRange(distance_cm))
        return 2; // Critical

    if (Proximity_WarningRange(distance_cm))
        return 1; // Warning

    // Clear (distance > 50cm)
    return 0;
}
