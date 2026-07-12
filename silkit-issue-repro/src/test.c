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
    frame.id = 0x100;
    frame.dlc = 1;
    frame.data.data = data;
    frame.data.size = 1;
    SilKit_CanController_SendFrame(g_canCtrl, &frame, NULL);

    printf("[STEP #%d] sim_time=%llums | sent frame 0x100\n",
           g_stepCount, (unsigned long long)(now/1000000ULL));
    fflush(stdout);

    /* explicitly NOT calling Stop() anywhere - testing if something
       implicit is ending the simulation after step 1 */
}

int main(void) {
    printf("=== SIL Kit Test - No explicit Stop(), step counter ===\n");
    fflush(stdout);

    SilKit_ParticipantConfiguration* cfg = NULL;
    SilKit_ParticipantConfiguration_FromString(&cfg, "{}");

    SilKit_Participant* p = NULL;
    SilKit_Participant_Create(&p, cfg, "vECU_Level3",
                               "silkit://localhost:8500");
    printf("[OK] Connected\n"); fflush(stdout);

    SilKit_CanController_Create(&g_canCtrl, p, "CAN1", "CAN1");
    SilKit_CanController_Start(g_canCtrl);
    printf("[OK] CAN controller started\n"); fflush(stdout);

    SilKit_LifecycleConfiguration lc;
    memset(&lc, 0, sizeof(lc));
    lc.operationMode = SilKit_OperationMode_Coordinated;

    SilKit_LifecycleService* lifecycle = NULL;
    SilKit_LifecycleService_Create(&lifecycle, p, &lc);

    SilKit_TimeSyncService* ts = NULL;
    SilKit_TimeSyncService_Create(&ts, lifecycle);
    SilKit_TimeSyncService_SetSimulationStepHandler(
        ts, NULL, &SimStep, 1000000ULL);

    printf("[OK] Starting lifecycle (no Stop() will be called)...\n");
    fflush(stdout);
    SilKit_LifecycleService_StartLifecycle(lifecycle);

    printf("[OK] StartLifecycle returned, waiting for completion...\n");
    fflush(stdout);

    SilKit_ParticipantState finalState;
    SilKit_LifecycleService_WaitForLifecycleToComplete(lifecycle, &finalState);

    printf("[DONE] Simulation ended after %d steps. Final state code: %d\n",
           g_stepCount, (int)finalState);
    return 0;
}
