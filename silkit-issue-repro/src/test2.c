#include "silkit/capi/SilKit.h"
#include "silkit/capi/Can.h"
#include "silkit/capi/Participant.h"
#include "silkit/capi/Orchestration.h"
#include <stdio.h>
#include <string.h>
#include <windows.h>

static SilKit_CanController* g_canCtrl = NULL;
static int g_stepCount = 0;

void SimStep(void* context, SilKit_TimeSyncService* ts,
             SilKit_NanosecondsTime now, SilKit_NanosecondsTime duration)
{
    (void)context; (void)ts; (void)duration;
    g_stepCount++;

    uint8_t data[1] = { 42 };
    SilKit_CanFrame frame;
    memset(&frame, 0, sizeof(frame));
    frame.id        = 0x100;
    frame.dlc       = 1;
    frame.data.data = data;
    frame.data.size = 1;
    SilKit_CanController_SendFrame(g_canCtrl, &frame, NULL);

    printf("[STEP #%d] sim_time=%llums | sent frame 0x100\n",
           g_stepCount,
           (unsigned long long)(now/1000000ULL));
    fflush(stdout);
}

int main(void) {
    printf("=== SIL Kit Coordinated Test B (FIXED) ===\n");
    fflush(stdout);

    SilKit_ParticipantConfiguration* cfg = NULL;
    SilKit_ReturnCode rc;

    rc = SilKit_ParticipantConfiguration_FromString(&cfg, "{}");
    if (rc != SilKit_ReturnCode_SUCCESS) {
        printf("ERROR config: %s\n", SilKit_GetLastErrorString());
        return 1;
    }

    SilKit_Participant* p = NULL;
    rc = SilKit_Participant_Create(&p, cfg, "Participant_B",
                                   "silkit://localhost:8500");
    if (rc != SilKit_ReturnCode_SUCCESS) {
        printf("ERROR connect: %s\n", SilKit_GetLastErrorString());
        return 1;
    }
    printf("[OK] Connected\n"); fflush(stdout);

    /* CAN controller */
    rc = SilKit_CanController_Create(&g_canCtrl, p, "CAN1", "CAN1");
    if (rc != SilKit_ReturnCode_SUCCESS) {
        printf("ERROR CAN: %s\n", SilKit_GetLastErrorString());
        return 1;
    }
    SilKit_CanController_Start(g_canCtrl);
    printf("[OK] CAN controller started\n"); fflush(stdout);

    /* FIX 1 — use SilKit_Struct_Init instead of memset */
    SilKit_LifecycleConfiguration lc;
    SilKit_Struct_Init(SilKit_LifecycleConfiguration, lc);
    lc.operationMode = SilKit_OperationMode_Coordinated;

    SilKit_LifecycleService* lifecycle = NULL;
    rc = SilKit_LifecycleService_Create(&lifecycle, p, &lc);
    if (rc != SilKit_ReturnCode_SUCCESS) {
        printf("ERROR lifecycle: %s\n", SilKit_GetLastErrorString());
        return 1;
    }
    printf("[OK] Lifecycle created\n"); fflush(stdout);

    SilKit_TimeSyncService* ts = NULL;
    rc = SilKit_TimeSyncService_Create(&ts, lifecycle);
    if (rc != SilKit_ReturnCode_SUCCESS) {
        printf("ERROR timesync: %s\n", SilKit_GetLastErrorString());
        return 1;
    }

    rc = SilKit_TimeSyncService_SetSimulationStepHandler(
             ts, NULL, &SimStep, 1000000ULL);
    if (rc != SilKit_ReturnCode_SUCCESS) {
        printf("ERROR step handler: %s\n", SilKit_GetLastErrorString());
        return 1;
    }
    printf("[OK] Sim step handler registered\n"); fflush(stdout);

    printf("[OK] Starting lifecycle (Coordinated)...\n");
    fflush(stdout);
    SilKit_LifecycleService_StartLifecycle(lifecycle);

    printf("[OK] Waiting for lifecycle to complete...\n");
    fflush(stdout);

    /* FIX 2 — pass &finalState not NULL */
    SilKit_ParticipantState finalState;
    rc = SilKit_LifecycleService_WaitForLifecycleToComplete(
             lifecycle, &finalState);

    printf("[DONE] Simulation ended after %d steps. "
           "Final state: %d\n", g_stepCount, (int)finalState);
    return 0;
}
