#include "mission.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Mission persistence keeps the current waypoint list and execution index in
 * flash so autonomy can survive resets and be restored with explicit metadata
 * about whether the mission was active when the checkpoint was written.
 */

#define MISSION_FLASH_BASE         0x080E0000u
#define MISSION_FLASH_SECTOR       FLASH_SECTOR_11
#define MISSION_FLASH_VOLTAGE      FLASH_VOLTAGE_RANGE_3
#define MISSION_PERSIST_MAGIC      0x4D495331u
#define MISSION_PERSIST_VERSION    1u

typedef struct {
    MissionMetadata_t metadata;
    Waypoint_t waypoints[MAX_WAYPOINTS];
    uint16_t waypoint_count;
} Mission_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t waypoint_count;
    uint16_t current_index;
    uint8_t mission_active;
    uint8_t reserved[3];
    MissionMetadata_t metadata;
    Waypoint_t waypoints[MAX_WAYPOINTS];
    uint32_t checksum;
} MissionCheckpoint_t;

static Mission_t g_current_mission = {0};
static MissionRestoreInfo_t g_restore_info = {0};

// Lightweight checksum over the persisted mission blob so corrupted flash
// checkpoints can be rejected before they affect runtime mission state.
static uint32_t Mission_ChecksumBytes(const uint8_t *data, size_t len)
{
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        hash ^= data[i];
        hash *= 16777619u;
    }
    return hash;
}

static void Mission_ResetRuntime(void)
{
    memset(&g_current_mission, 0, sizeof(g_current_mission));
}

static uint8_t Mission_ReadCheckpoint(MissionCheckpoint_t *out_checkpoint)
{
    if (!out_checkpoint) {
        return 0;
    }

    const MissionCheckpoint_t *stored = (const MissionCheckpoint_t *)MISSION_FLASH_BASE;
    if (stored->magic != MISSION_PERSIST_MAGIC || stored->version != MISSION_PERSIST_VERSION) {
        return 0;
    }
    if (stored->waypoint_count == 0 || stored->waypoint_count > MAX_WAYPOINTS) {
        return 0;
    }
    if (stored->current_index > stored->waypoint_count) {
        return 0;
    }

    uint32_t expected = Mission_ChecksumBytes((const uint8_t *)stored, sizeof(MissionCheckpoint_t) - sizeof(uint32_t));
    if (expected != stored->checksum) {
        return 0;
    }

    memcpy(out_checkpoint, stored, sizeof(MissionCheckpoint_t));
    return 1;
}

static uint8_t Mission_WriteCheckpoint(const MissionCheckpoint_t *checkpoint)
{
    if (!checkpoint) {
        return 0;
    }

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = MISSION_FLASH_VOLTAGE;
    erase.Sector = MISSION_FLASH_SECTOR;
    erase.NbSectors = 1;

    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        printf("[MISSION] FLASH erase failed (sector=%lu err=%lu)\r\n", (unsigned long)erase.Sector, (unsigned long)sector_error);
        return 0;
    }

    const uint8_t *bytes = (const uint8_t *)checkpoint;
    for (uint32_t offset = 0; offset < sizeof(MissionCheckpoint_t); offset++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, MISSION_FLASH_BASE + offset, bytes[offset]) != HAL_OK) {
            HAL_FLASH_Lock();
            printf("[MISSION] FLASH program failed at +%lu\r\n", (unsigned long)offset);
            return 0;
        }
    }

    HAL_FLASH_Lock();
    return 1;
}

void Mission_ClearCurrent(void)
{
    Mission_ResetRuntime();
}

void Mission_ClearPersisted(void)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = MISSION_FLASH_VOLTAGE;
    erase.Sector = MISSION_FLASH_SECTOR;
    erase.NbSectors = 1;
    (void)HAL_FLASHEx_Erase(&erase, &sector_error);
    HAL_FLASH_Lock();

    memset(&g_restore_info, 0, sizeof(g_restore_info));
    printf("[MISSION] Cleared persisted mission checkpoint\r\n");
}

void Mission_Init(void)
{
    MissionCheckpoint_t checkpoint;

    Mission_ResetRuntime();
    memset(&g_restore_info, 0, sizeof(g_restore_info));

    if (Mission_ReadCheckpoint(&checkpoint)) {
        memcpy(&g_current_mission.metadata, &checkpoint.metadata, sizeof(MissionMetadata_t));
        memcpy(g_current_mission.waypoints, checkpoint.waypoints, sizeof(Waypoint_t) * checkpoint.waypoint_count);
        g_current_mission.waypoint_count = checkpoint.waypoint_count;
        g_current_mission.metadata.waypoint_count = checkpoint.waypoint_count;

        g_restore_info.has_saved_mission = 1;
        g_restore_info.waypoint_count = checkpoint.waypoint_count;
        g_restore_info.current_index = checkpoint.current_index;
        g_restore_info.mission_active = checkpoint.mission_active;

        printf("[MISSION] Restored checkpoint: %u waypoints, index %u, active=%u\r\n",
               checkpoint.waypoint_count,
               checkpoint.current_index,
               checkpoint.mission_active);
        return;
    }

    printf("[MISSION] Module initialized (no saved checkpoint)\r\n");
}

uint8_t Mission_Create(const char *name, const char *description)
{
    Mission_ResetRuntime();

    strncpy(g_current_mission.metadata.name, name ? name : "Mission", sizeof(g_current_mission.metadata.name) - 1);
    g_current_mission.metadata.name[sizeof(g_current_mission.metadata.name) - 1] = '\0';
    g_current_mission.metadata.created_timestamp = 0;

    printf("[MISSION] Created mission: %s\r\n", g_current_mission.metadata.name);
    if (description) {
        printf("[MISSION] Description: %s\r\n", description);
    }
    return 1;
}

uint8_t Mission_AddWaypoint(float latitude, float longitude, uint8_t salt_rate, uint8_t brine_rate)
{
    if (g_current_mission.waypoint_count >= MAX_WAYPOINTS) {
        printf("[MISSION] ERROR: Mission full (%d/%d waypoints)\r\n", g_current_mission.waypoint_count, MAX_WAYPOINTS);
        return 0;
    }

    Waypoint_t *wp = &g_current_mission.waypoints[g_current_mission.waypoint_count];
    wp->latitude = latitude;
    wp->longitude = longitude;
    wp->salt_rate = (float)salt_rate / 100.0f;
    wp->brine_rate = (float)brine_rate / 100.0f;

    g_current_mission.waypoint_count++;
    g_current_mission.metadata.waypoint_count = g_current_mission.waypoint_count;

    printf("[MISSION] Waypoint %d added: %.6f, %.6f (salt=%d%%, brine=%d%%)\r\n",
           g_current_mission.waypoint_count, latitude, longitude, salt_rate, brine_rate);
    return 1;
}

void Mission_RemoveLastWaypoint(void)
{
    if (g_current_mission.waypoint_count > 0) {
        g_current_mission.waypoint_count--;
        g_current_mission.metadata.waypoint_count = g_current_mission.waypoint_count;
        printf("[MISSION] Last waypoint removed (now %d waypoints)\r\n", g_current_mission.waypoint_count);
    } else {
        printf("[MISSION] WARNING: No waypoints to remove\r\n");
    }
}

static float Haversine_Distance(float lat1, float lon1, float lat2, float lon2)
{
    const float R = 6371000.0f;
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

void Mission_CalculateStats(float robot_width_m, float coverage_overlap_m)
{
    if (g_current_mission.waypoint_count == 0) {
        printf("[MISSION] WARNING: No waypoints to calculate stats\r\n");
        return;
    }

    float total_distance = 0.0f;
    for (uint16_t i = 0; i < (g_current_mission.waypoint_count - 1); i++) {
        Waypoint_t *wp1 = &g_current_mission.waypoints[i];
        Waypoint_t *wp2 = &g_current_mission.waypoints[i + 1];
        total_distance += Haversine_Distance(wp1->latitude, wp1->longitude, wp2->latitude, wp2->longitude);
    }

    float effective_width = robot_width_m - coverage_overlap_m;
    if (effective_width < 0.5f) effective_width = 0.5f;
    float coverage_area = total_distance * effective_width;
    float coverage_200m2_units = coverage_area / 200.0f;
    float salt_needed = coverage_200m2_units * 1.0f;
    float brine_needed = coverage_200m2_units * 9.0f;
    float estimated_time = total_distance / 0.5f;

    g_current_mission.metadata.total_distance_m = total_distance;
    g_current_mission.metadata.total_coverage_m2 = coverage_area;
    g_current_mission.metadata.salt_needed_kg = salt_needed;
    g_current_mission.metadata.brine_needed_l = brine_needed;
    g_current_mission.metadata.estimated_time_s = (uint32_t)estimated_time;
}

uint16_t Mission_GetWaypointCount(void)
{
    return g_current_mission.waypoint_count;
}

const MissionMetadata_t* Mission_GetMetadata(void)
{
    return &g_current_mission.metadata;
}

Waypoint_t* Mission_GetWaypoints(void)
{
    return g_current_mission.waypoints;
}

void Mission_PrintSummary(void)
{
    const MissionMetadata_t *meta = &g_current_mission.metadata;
    printf("\r\n========== MISSION SUMMARY ==========\r\n");
    printf("Name: %s\r\n", meta->name);
    printf("Waypoints: %d\r\n", meta->waypoint_count);
    printf("Total Distance: %.1f m\r\n", meta->total_distance_m);
    printf("Coverage Area: %.1f m^2\r\n", meta->total_coverage_m2);
    printf("Salt Required: %.2f kg\r\n", meta->salt_needed_kg);
    printf("Brine Required: %.2f L\r\n", meta->brine_needed_l);
    printf("Est. Duration: %lu min\r\n", meta->estimated_time_s / 60);
    printf("====================================\r\n\r\n");
}

uint8_t Mission_PersistCurrent(uint16_t current_index, uint8_t mission_active)
{
    if (g_current_mission.waypoint_count == 0) {
        // A waypoint reload briefly empties the runtime mission buffer before the
        // new batch arrives. Do not erase the last saved mission in that window.
        // Explicit clear and mission-complete paths already wipe the checkpoint.
        return 1;
    }

    if (current_index > g_current_mission.waypoint_count) {
        current_index = g_current_mission.waypoint_count;
    }

    MissionCheckpoint_t checkpoint;
    memset(&checkpoint, 0, sizeof(checkpoint));
    checkpoint.magic = MISSION_PERSIST_MAGIC;
    checkpoint.version = MISSION_PERSIST_VERSION;
    checkpoint.waypoint_count = g_current_mission.waypoint_count;
    checkpoint.current_index = current_index;
    checkpoint.mission_active = mission_active ? 1u : 0u;
    memcpy(&checkpoint.metadata, &g_current_mission.metadata, sizeof(MissionMetadata_t));
    memcpy(checkpoint.waypoints, g_current_mission.waypoints, sizeof(Waypoint_t) * g_current_mission.waypoint_count);
    checkpoint.checksum = Mission_ChecksumBytes((const uint8_t *)&checkpoint, sizeof(MissionCheckpoint_t) - sizeof(uint32_t));

    if (!Mission_WriteCheckpoint(&checkpoint)) {
        return 0;
    }

    g_restore_info.has_saved_mission = 1;
    g_restore_info.waypoint_count = checkpoint.waypoint_count;
    g_restore_info.current_index = checkpoint.current_index;
    g_restore_info.mission_active = checkpoint.mission_active;
    printf("[MISSION] Persisted checkpoint: wp=%u idx=%u active=%u\r\n",
           checkpoint.waypoint_count,
           checkpoint.current_index,
           checkpoint.mission_active);
    return 1;
}

uint8_t Mission_RestoreInfo(MissionRestoreInfo_t *out_info)
{
    if (!out_info) {
        return 0;
    }
    memcpy(out_info, &g_restore_info, sizeof(g_restore_info));
    return g_restore_info.has_saved_mission;
}

uint8_t Mission_Load(uint8_t slot)
{
    if (slot != 0u) {
        printf("[MISSION] ERROR: Only slot 0 is implemented for checkpoint restore\r\n");
        return 0;
    }

    MissionCheckpoint_t checkpoint;
    if (!Mission_ReadCheckpoint(&checkpoint)) {
        printf("[MISSION] No saved mission checkpoint in slot %u\r\n", slot);
        return 0;
    }

    Mission_ResetRuntime();
    memcpy(&g_current_mission.metadata, &checkpoint.metadata, sizeof(MissionMetadata_t));
    memcpy(g_current_mission.waypoints, checkpoint.waypoints, sizeof(Waypoint_t) * checkpoint.waypoint_count);
    g_current_mission.waypoint_count = checkpoint.waypoint_count;
    g_current_mission.metadata.waypoint_count = checkpoint.waypoint_count;

    g_restore_info.has_saved_mission = 1;
    g_restore_info.waypoint_count = checkpoint.waypoint_count;
    g_restore_info.current_index = checkpoint.current_index;
    g_restore_info.mission_active = checkpoint.mission_active;
    printf("[MISSION] Loaded checkpoint from slot %u\r\n", slot);
    return 1;
}

uint8_t Mission_Save(uint8_t slot)
{
    if (slot != 0u) {
        printf("[MISSION] ERROR: Only slot 0 is implemented for checkpoint save\r\n");
        return 0;
    }
    return Mission_PersistCurrent(g_restore_info.current_index, g_restore_info.mission_active);
}

void Mission_Delete(uint8_t slot)
{
    if (slot != 0u) {
        printf("[MISSION] ERROR: Only slot 0 is implemented for checkpoint delete\r\n");
        return;
    }
    Mission_ClearPersisted();
}
