#ifndef MISSION_H
#define MISSION_H

/**
 * @file mission.h
 * @brief Mission definition and persistence API.
 *
 * This module provides a small "mission" abstraction used by the autonomous
 * mode/state machine. A mission is represented as an ordered list of waypoints
 * plus a small metadata header that can be saved/loaded from persistent storage
 * (implementation lives in Core/Src/mission.c).
 *
 * Notes:
 * - The on-disk / persisted layout is versioned via MISSION_FORMAT_VERSION.
 * - MAX_WAYPOINTS limits RAM usage and bounds save files.
 * - Waypoint_t is declared in robot_sm.h (included here).
 */

#include <stdint.h>
#include "robot_sm.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Mission file format version for persisted missions. */
#define MISSION_FORMAT_VERSION 1

/** Maximum number of waypoints allowed in a single mission. */
#define MAX_WAYPOINTS 120

/**
 * @brief Human- and UI-facing mission summary data.
 *
 * These values are intended for display/telemetry and may be derived from the
 * waypoint list (e.g., totals computed by Mission_CalculateStats()).
 */
typedef struct {
    char name[32];                 /**< Mission name (null-terminated if shorter). */
    uint32_t created_timestamp;    /**< Creation time (units defined by implementation). */

    float total_distance_m;        /**< Total planned path length (meters). */
    float total_coverage_m2;       /**< Estimated coverage area (m^2). */

    float salt_needed_kg;          /**< Estimated salt required (kg). */
    float brine_needed_l;          /**< Estimated brine required (liters). */

    uint32_t estimated_time_s;     /**< Estimated mission time (seconds). */
    uint16_t waypoint_count;       /**< Number of waypoints in mission. */
} MissionMetadata_t;

/**
 * @brief Persisted mission header structure.
 *
 * This is the top-level structure written to/read from storage. The waypoint
 * array itself is stored/managed by the mission module (see mission.c).
 */
typedef struct {
    uint16_t format_version;       /**< Must equal MISSION_FORMAT_VERSION. */
    MissionMetadata_t metadata;    /**< Mission summary metadata. */
} MissionFile_t;

/**
 * @brief Minimal state used to resume an interrupted mission.
 */
typedef struct {
    uint8_t has_saved_mission;     /**< 1 if a persisted mission exists to restore. */
    uint16_t waypoint_count;       /**< Waypoint count in the saved mission. */
    uint16_t current_index;        /**< Index of current/next waypoint to execute. */
    uint8_t mission_active;        /**< 1 if mission was active when persisted. */
} MissionRestoreInfo_t;

/**
 * @brief Initialize the mission subsystem.
 *
 * Call once at boot after any required storage/peripheral initialization.
 */
void Mission_Init(void);

/**
 * @brief Clear the current in-RAM mission (waypoints + metadata).
 */
void Mission_ClearCurrent(void);

/**
 * @brief Clear any persisted mission data from storage.
 */
void Mission_ClearPersisted(void);

/**
 * @brief Create a new mission and reset current mission state.
 *
 * @param name        Human-readable mission name.
 * @param description Optional description (may be ignored depending on impl).
 * @return 1 on success, 0 on failure.
 */
uint8_t Mission_Create(const char *name, const char *description);

/**
 * @brief Load a mission from a numbered storage slot.
 * @param slot Storage slot index.
 * @return 1 on success, 0 on failure.
 */
uint8_t Mission_Load(uint8_t slot);

/**
 * @brief Save the current mission to a numbered storage slot.
 * @param slot Storage slot index.
 * @return 1 on success, 0 on failure.
 */
uint8_t Mission_Save(uint8_t slot);

/**
 * @brief Append a waypoint to the current mission.
 *
 * @param latitude   Waypoint latitude (decimal degrees).
 * @param longitude  Waypoint longitude (decimal degrees).
 * @param salt_rate  Salt dispersion rate (0-100%).
 * @param brine_rate Brine dispersion rate (0-100%).
 * @return 1 on success, 0 on failure (e.g., MAX_WAYPOINTS reached).
 */
uint8_t Mission_AddWaypoint(float latitude, float longitude, uint8_t salt_rate, uint8_t brine_rate);

/**
 * @brief Remove the last waypoint from the current mission (if any).
 */
void Mission_RemoveLastWaypoint(void);

/**
 * @brief Get the number of waypoints currently in RAM.
 */
uint16_t Mission_GetWaypointCount(void);

/**
 * @brief Compute derived mission totals (distance, coverage, materials, time).
 *
 * @param robot_width_m        Effective robot coverage width (meters).
 * @param coverage_overlap_m   Overlap between adjacent passes (meters).
 */
void Mission_CalculateStats(float robot_width_m, float coverage_overlap_m);

/**
 * @brief Get a pointer to the current mission metadata (read-only).
 */
const MissionMetadata_t* Mission_GetMetadata(void);

/**
 * @brief Print a human-readable mission summary to the console.
 */
void Mission_PrintSummary(void);

/**
 * @brief Delete a mission from a numbered storage slot.
 * @param slot Storage slot index.
 */
void Mission_Delete(uint8_t slot);

/**
 * @brief Persist minimal mission progress info for restore-on-boot behavior.
 *
 * @param current_index Current/next waypoint index.
 * @param mission_active 1 if mission should resume as active, 0 otherwise.
 * @return 1 on success, 0 on failure.
 */
uint8_t Mission_PersistCurrent(uint16_t current_index, uint8_t mission_active);

/**
 * @brief Read persisted restore info.
 * @param out_info Output structure to populate.
 * @return 1 on success, 0 on failure/no saved info.
 */
uint8_t Mission_RestoreInfo(MissionRestoreInfo_t *out_info);

/**
 * @brief Get direct access to the current waypoint array.
 *
 * Use with care: callers must respect Mission_GetWaypointCount() and
 * MAX_WAYPOINTS bounds.
 */
Waypoint_t* Mission_GetWaypoints(void);

#ifdef __cplusplus
}
#endif

#endif // MISSION_H
