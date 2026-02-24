#ifndef MISSION_H
#define MISSION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Mission file format version
#define MISSION_FORMAT_VERSION 1

// Maximum waypoints per mission
#define MAX_WAYPOINTS 50

// Mission metadata and statistics
typedef struct {
    char name[32];              // Mission name
    uint32_t created_timestamp; // Creation time (Unix seconds)
    float total_distance_m;     // Total distance to travel (meters)
    float total_coverage_m2;    // Total coverage area (m^2)
    float salt_needed_kg;       // Estimated salt needed
    float brine_needed_l;       // Estimated brine needed (1:9 ratio)
    uint32_t estimated_time_s;  // Estimated mission time (seconds)
    uint16_t waypoint_count;    // Number of waypoints
} MissionMetadata_t;

// Mission file structure (stored in flash/EEPROM)
typedef struct {
    uint16_t format_version;
    MissionMetadata_t metadata;
    // Waypoints stored separately via Mission_GetWaypoint()
} MissionFile_t;

// Initialize mission module
void Mission_Init(void);

// Create new mission with metadata
// Returns: 1 on success, 0 if storage full
uint8_t Mission_Create(const char *name, const char *description);

// Load mission from storage (index 0-9 for 10 mission slots)
// Returns: 1 on success, 0 if mission not found
uint8_t Mission_Load(uint8_t slot);

// Save current mission to storage
// Returns: 1 on success, 0 if storage error
uint8_t Mission_Save(uint8_t slot);

// Add waypoint to current mission
// Returns: 1 on success, 0 if mission full (>50 waypoints)
uint8_t Mission_AddWaypoint(float latitude, float longitude, 
                             uint8_t salt_rate, uint8_t brine_rate);

// Remove last waypoint from current mission
void Mission_RemoveLastWaypoint(void);

// Get waypoint count in current mission
uint16_t Mission_GetWaypointCount(void);

// Calculate mission statistics (distance, coverage, material usage)
// Call after adding all waypoints to update metadata
void Mission_CalculateStats(float robot_width_m, float coverage_overlap_m);

// Get current mission metadata
const MissionMetadata_t* Mission_GetMetadata(void);

// Print mission summary to console
void Mission_PrintSummary(void);

// Delete mission from storage slot
void Mission_Delete(uint8_t slot);

#ifdef __cplusplus
}
#endif

#endif // MISSION_H
