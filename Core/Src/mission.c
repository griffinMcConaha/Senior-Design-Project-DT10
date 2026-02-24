#include "mission.h"
#include "robot_sm.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// ============================================================================
// MISSION MANAGEMENT MODULE
// Handles mission creation, storage, loading, and statistics calculation
// ============================================================================

// Mission storage (in RAM - TODO: add flash/EEPROM persistence)
typedef struct {
    MissionMetadata_t metadata;
    Waypoint_t waypoints[MAX_WAYPOINTS];
    uint16_t waypoint_count;
} Mission_t;

// Current mission in memory
static Mission_t g_current_mission = {0};

// Initialize mission module
void Mission_Init(void)
{
    memset(&g_current_mission, 0, sizeof(Mission_t));
    printf("[MISSION] Module initialized\r\n");
}

// Create new mission with metadata
// Returns: 1 on success, 0 if storage full
uint8_t Mission_Create(const char *name, const char *description)
{
    // Clear previous mission
    memset(&g_current_mission, 0, sizeof(Mission_t));
    
    // Copy mission name (max 31 chars + null)
    strncpy(g_current_mission.metadata.name, name, sizeof(g_current_mission.metadata.name) - 1);
    g_current_mission.metadata.name[sizeof(g_current_mission.metadata.name) - 1] = '\0';
    
    // Set creation timestamp (TODO: get real time from RTC)
    g_current_mission.metadata.created_timestamp = 0; // Placeholder
    
    printf("[MISSION] Created mission: %s\r\n", name);
    if (description)
        printf("[MISSION] Description: %s\r\n", description);
    
    return 1;
}

// Add waypoint to current mission
// Returns: 1 on success, 0 if mission full (>50 waypoints)
uint8_t Mission_AddWaypoint(float latitude, float longitude, 
                             uint8_t salt_rate, uint8_t brine_rate)
{
    // Check if mission is full
    if (g_current_mission.waypoint_count >= MAX_WAYPOINTS)
    {
        printf("[MISSION] ERROR: Mission full (%d/%d waypoints)\r\n",
               g_current_mission.waypoint_count, MAX_WAYPOINTS);
        return 0;
    }

    // Add waypoint
    Waypoint_t *wp = &g_current_mission.waypoints[g_current_mission.waypoint_count];
    wp->latitude = latitude;
    wp->longitude = longitude;
    wp->salt_rate = salt_rate;
    wp->brine_rate = brine_rate;

    g_current_mission.waypoint_count++;
    g_current_mission.metadata.waypoint_count = g_current_mission.waypoint_count;

    printf("[MISSION] Waypoint %d added: %.6f, %.6f (salt=%d%%, brine=%d%%)\r\n",
           g_current_mission.waypoint_count, latitude, longitude, salt_rate, brine_rate);

    return 1;
}

// Remove last waypoint from current mission
void Mission_RemoveLastWaypoint(void)
{
    if (g_current_mission.waypoint_count > 0)
    {
        g_current_mission.waypoint_count--;
        g_current_mission.metadata.waypoint_count = g_current_mission.waypoint_count;
        printf("[MISSION] Last waypoint removed (now %d waypoints)\r\n",
               g_current_mission.waypoint_count);
    }
    else
    {
        printf("[MISSION] WARNING: No waypoints to remove\r\n");
    }
}

// Helper: Haversine distance formula (meters)
// Calculates great-circle distance between two lat/lon points
static float Haversine_Distance(float lat1, float lon1, float lat2, float lon2)
{
    const float R = 6371000.0f; // Earth radius in meters
    const float PI = 3.14159265f;
    
    float lat1_rad = lat1 * PI / 180.0f;
    float lat2_rad = lat2 * PI / 180.0f;
    float delta_lat = (lat2 - lat1) * PI / 180.0f;
    float delta_lon = (lon2 - lon1) * PI / 180.0f;
    
    float a = sinf(delta_lat / 2.0f) * sinf(delta_lat / 2.0f) +
              cosf(lat1_rad) * cosf(lat2_rad) * sinf(delta_lon / 2.0f) * sinf(delta_lon / 2.0f);
    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
    
    return R * c;
}

// Calculate mission statistics (distance, coverage, material usage)
// robot_width_m: width of coverage pattern (e.g., 3.0 meters)
// coverage_overlap_m: overlap between passes (e.g., 0.5 meters)
void Mission_CalculateStats(float robot_width_m, float coverage_overlap_m)
{
    if (g_current_mission.waypoint_count == 0)
    {
        printf("[MISSION] WARNING: No waypoints to calculate stats\r\n");
        return;
    }

    // Calculate total distance between waypoints
    float total_distance = 0.0f;
    for (uint16_t i = 0; i < (g_current_mission.waypoint_count - 1); i++)
    {
        Waypoint_t *wp1 = &g_current_mission.waypoints[i];
        Waypoint_t *wp2 = &g_current_mission.waypoints[i + 1];
        
        float segment_dist = Haversine_Distance(wp1->latitude, wp1->longitude,
                                                wp2->latitude, wp2->longitude);
        total_distance += segment_dist;
    }

    // Estimate coverage area
    // Assume robot follows waypoints back and forth (N passes)
    // For simplicity: coverage = total_distance * effective_width
    float effective_width = robot_width_m - coverage_overlap_m; // Avoid double-counting overlap
    if (effective_width < 0.5f) effective_width = 0.5f; // Minimum 0.5m
    float coverage_area = total_distance * effective_width;

    // Estimate dispersion material needed
    // Design spec: 200 m² per charge, assume ~1kg salt + 9L brine per 200m²
    // So: material = (coverage / 200) * base_amount
    float coverage_200m2_units = coverage_area / 200.0f;
    float salt_needed = coverage_200m2_units * 1.0f;    // 1 kg salt per 200m²
    float brine_needed = coverage_200m2_units * 9.0f;   // 9 L brine per 200m² (1:9 ratio)

    // Estimate time (assume 0.5 m/s average speed)
    float avg_speed_ms = 0.5f; // meters per second
    float estimated_time = total_distance / avg_speed_ms; // seconds

    // Update metadata
    g_current_mission.metadata.total_distance_m = total_distance;
    g_current_mission.metadata.total_coverage_m2 = coverage_area;
    g_current_mission.metadata.salt_needed_kg = salt_needed;
    g_current_mission.metadata.brine_needed_l = brine_needed;
    g_current_mission.metadata.estimated_time_s = (uint32_t)estimated_time;

    printf("[MISSION] Statistics calculated:\r\n");
    printf("  Distance: %.1f m\r\n", total_distance);
    printf("  Coverage: %.1f m²\r\n", coverage_area);
    printf("  Salt needed: %.2f kg\r\n", salt_needed);
    printf("  Brine needed: %.2f L\r\n", brine_needed);
    printf("  Est. time: %d minutes\r\n", (int)(estimated_time / 60));
}

// Get waypoint count in current mission
uint16_t Mission_GetWaypointCount(void)
{
    return g_current_mission.waypoint_count;
}

// Get current mission metadata
const MissionMetadata_t* Mission_GetMetadata(void)
{
    return &g_current_mission.metadata;
}

// Print mission summary to console
void Mission_PrintSummary(void)
{
    const MissionMetadata_t *meta = &g_current_mission.metadata;
    
    printf("\r\n========== MISSION SUMMARY ==========\r\n");
    printf("Name: %s\r\n", meta->name);
    printf("Waypoints: %d\r\n", meta->waypoint_count);
    printf("Total Distance: %.1f m\r\n", meta->total_distance_m);
    printf("Coverage Area: %.1f m²\r\n", meta->total_coverage_m2);
    printf("Salt Required: %.2f kg\r\n", meta->salt_needed_kg);
    printf("Brine Required: %.2f L\r\n", meta->brine_needed_l);
    printf("Est. Duration: %lu min\r\n", meta->estimated_time_s / 60);
    printf("====================================\r\n\r\n");
}

// Load mission from storage (index 0-9 for 10 mission slots)
// Returns: 1 on success, 0 if mission not found
uint8_t Mission_Load(uint8_t slot)
{
    if (slot >= 10)
    {
        printf("[MISSION] ERROR: Invalid slot %d (0-9)\r\n", slot);
        return 0;
    }

    // TODO: Load mission from flash/EEPROM at address (MISSION_BASE + slot * MISSION_SIZE)
    // For now, just log the request
    printf("[MISSION] Load from slot %d - TODO: Implement flash/EEPROM read\r\n", slot);

    return 0; // Placeholder
}

// Save current mission to storage
// Returns: 1 on success, 0 if storage error
uint8_t Mission_Save(uint8_t slot)
{
    if (slot >= 10)
    {
        printf("[MISSION] ERROR: Invalid slot %d (0-9)\r\n", slot);
        return 0;
    }

    if (g_current_mission.waypoint_count == 0)
    {
        printf("[MISSION] ERROR: Cannot save empty mission\r\n");
        return 0;
    }

    // TODO: Save mission to flash/EEPROM at address (MISSION_BASE + slot * MISSION_SIZE)
    // For now, just log the operation
    printf("[MISSION] Saved to slot %d - TODO: Implement flash/EEPROM write\r\n", slot);
    printf("  %d waypoints, %.1f m coverage\r\n",
           g_current_mission.waypoint_count,
           g_current_mission.metadata.total_coverage_m2);

    return 1; // Placeholder success
}

// Delete mission from storage slot
void Mission_Delete(uint8_t slot)
{
    if (slot >= 10)
    {
        printf("[MISSION] ERROR: Invalid slot %d (0-9)\r\n", slot);
        return;
    }

    // TODO: Erase mission from flash/EEPROM
    printf("[MISSION] Deleted slot %d - TODO: Implement flash/EEPROM erase\r\n", slot);
}
