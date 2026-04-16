#ifndef MISSION_H
#define MISSION_H

#include <stdint.h>
#include "robot_sm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MISSION_FORMAT_VERSION 1
#define MAX_WAYPOINTS 120

typedef struct {
    char name[32];
    uint32_t created_timestamp;
    float total_distance_m;
    float total_coverage_m2;
    float salt_needed_kg;
    float brine_needed_l;
    uint32_t estimated_time_s;
    uint16_t waypoint_count;
} MissionMetadata_t;

typedef struct {
    uint16_t format_version;
    MissionMetadata_t metadata;
} MissionFile_t;

typedef struct {
    uint8_t has_saved_mission;
    uint16_t waypoint_count;
    uint16_t current_index;
    uint8_t mission_active;
} MissionRestoreInfo_t;

void Mission_Init(void);
void Mission_ClearCurrent(void);
void Mission_ClearPersisted(void);
uint8_t Mission_Create(const char *name, const char *description);
uint8_t Mission_Load(uint8_t slot);
uint8_t Mission_Save(uint8_t slot);
uint8_t Mission_AddWaypoint(float latitude, float longitude, uint8_t salt_rate, uint8_t brine_rate);
void Mission_RemoveLastWaypoint(void);
uint16_t Mission_GetWaypointCount(void);
void Mission_CalculateStats(float robot_width_m, float coverage_overlap_m);
const MissionMetadata_t* Mission_GetMetadata(void);
void Mission_PrintSummary(void);
void Mission_Delete(uint8_t slot);
uint8_t Mission_PersistCurrent(uint16_t current_index, uint8_t mission_active);
uint8_t Mission_RestoreInfo(MissionRestoreInfo_t *out_info);
Waypoint_t* Mission_GetWaypoints(void);

#ifdef __cplusplus
}
#endif

#endif // MISSION_H
