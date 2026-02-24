// Proximity sensor driver for obstacle detection (HC-SR04 or LV-Maxsonar)
#ifndef PROXIMITY_H
#define PROXIMITY_H

#include <stdint.h>
#include <stdbool.h>

// Special return value indicating no object detected (sensor timed out waiting for echo)
#define PROX_NO_DETECTION 65535  // Special sentinel value (uint16_t max)

// Proximity sensor reading (distance in cm)
typedef struct {
    uint16_t distance_cm;    // Distance in centimeters
    bool valid;              // True if valid reading
} ProximityReading_t;

// Initialize proximity sensors (GPIO/ADC/UART depending on sensor type)
void Proximity_Init(void);

// Read proximity sensors (left and right, typically front-left/front-right)
// Returns distance in centimeters, or PROX_NO_DETECTION (65535) if no object detected
uint16_t Proximity_ReadLeft(void);
uint16_t Proximity_ReadRight(void);

// Read both sensors at once (convenience function)
void Proximity_ReadBoth(uint16_t *left_cm, uint16_t *right_cm);

// Get last cached reading without triggering new measurement (fast, for inter-MCU comms)
uint16_t Proximity_GetLastLeft(void);
uint16_t Proximity_GetLastRight(void);

// Check if obstacle detected at warning distance (30-50cm)
bool Proximity_WarningRange(uint16_t distance_cm);

// Check if obstacle detected at critical distance (<10cm)
bool Proximity_CriticalRange(uint16_t distance_cm);

// Get status: 0=clear, 1=warning, 2=critical
uint8_t Proximity_GetStatus(uint16_t distance_cm);

#endif // PROXIMITY_H
