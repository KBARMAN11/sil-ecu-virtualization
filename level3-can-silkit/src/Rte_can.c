#include "Rte_can.h"
#include "silkit/capi/SilKit.h"
#include "silkit/capi/Can.h"
#include "silkit/capi/Participant.h"
#include "silkit/capi/Orchestration.h"
#include <stdio.h>
#include <string.h>
#include <windows.h>

SilKit_Participant*             g_participant = NULL;
static SilKit_CanController*    g_canCtrl     = NULL;
static SilKit_LifecycleService* g_lifecycle   = NULL;
static SilKit_TimeSyncService*  g_timeSync    = NULL;

static volatile uint8_t  g_throttleCmd  = 0;
static volatile uint8_t  g_brakeCmd     = 0;
static volatile uint16_t g_targetSpeed  = 80;
static volatile uint16_t g_currentSpeed = 0;

void CanFrameHandler(void* context,
                     SilKit_CanController* ctrl,
                     SilKit_CanFrameEvent* frameEvent)
{
    (void)context; (void)ctrl;
    SilKit_CanFrame* frame = frameEvent->frame;
    uint32_t id = frame->id;
    const uint8_t* data = frame->data.data;

    if (id == CAN_ID_THROTTLE_CMD && frame->data.size >= 1)
        g_throttleCmd  = data[0];
    else if (id == CAN_ID_BRAKE_CMD && frame->data.size >= 1)
        g_brakeCmd     = data[0];
    else if (id == CAN_ID_TARGET_SPEED && frame->data.size >= 2)
        g_targetSpeed  = (uint16_t)((data[0] << 8) | data[1]);
}

static void SendCanFrame(uint32_t id, uint8_t* data, size_t len)
{
    SilKit_CanFrame frame;
    memset(&frame, 0, sizeof(frame));
    frame.id        = id;
    frame.dlc       = (uint8_t)len;
    frame.data.data = data;
    frame.data.size = len;
    SilKit_CanController_SendFrame(g_canCtrl, &frame, NULL);

    printf("[BUS] TX | ID=0x%03X | ", id);
    switch(id) {
        case 0x101: printf("ThrottleCmd  = %3d%%\n", data[0]); break;
        case 0x102: printf("BrakeCmd     = %3d%%\n", data[0]); break;
        case 0x201: printf("CurrentSpeed = %3d km/h\n",
                           (int)((data[0]<<8)|data[1])); break;
        case 0x202: printf("TargetSpeed  = %3d km/h\n",
                           (int)((data[0]<<8)|data[1])); break;
        default:
            for (size_t i=0;i<len;i++) printf("%02X ",data[i]);
            printf("\n");
    }
    fflush(stdout);
}

void Rte_Init_Can(void)
{
    SilKit_ParticipantConfiguration* cfg = NULL;
    SilKit_ParticipantConfiguration_FromString(&cfg, "{}");

    SilKit_ReturnCode rc = SilKit_Participant_Create(
        &g_participant, cfg, "vECU_Level3",
        "silkit://localhost:8500");
    if (rc != SilKit_ReturnCode_SUCCESS) {
        printf("[RTE] ERROR: %s\n", SilKit_GetLastErrorString());
        exit(1);
    }
    printf("[RTE] Connected to SIL Kit registry\n");

    SilKit_CanController_Create(&g_canCtrl, g_participant, "CAN1", "CAN1");
    SilKit_HandlerId h;
    SilKit_CanController_AddFrameHandler(
        g_canCtrl, NULL, &CanFrameHandler,
        SilKit_Direction_Receive, &h);
    SilKit_CanController_Start(g_canCtrl);
    printf("[RTE] Virtual CAN1 started\n");

    uint8_t d[2] = {(uint8_t)(g_targetSpeed>>8),
                    (uint8_t)(g_targetSpeed&0xFF)};
    SendCanFrame(CAN_ID_TARGET_SPEED, d, 2);
    printf("[RTE] Initial target speed sent: %d km/h\n\n", g_targetSpeed);
}

/* Simulation step handler — driven by SIL Kit's coordinated time sync */
void SimStepHandler(void* context,
                    SilKit_TimeSyncService* timeSyncService,
                    SilKit_NanosecondsTime now,
                    SilKit_NanosecondsTime duration)
{
    (void)context; (void)timeSyncService; (void)duration;
    static SilKit_NanosecondsTime lastSim = 0;
    if (now - lastSim >= 100000000ULL) {  /* every 100ms sim time */
        Rte_SimulateStep();
        lastSim = now;
    }
}

void Rte_Start_Lifecycle(void)
{
    SilKit_LifecycleConfiguration lc;
    memset(&lc, 0, sizeof(lc));
    lc.operationMode = SilKit_OperationMode_Coordinated;
    SilKit_LifecycleService_Create(&g_lifecycle, g_participant, &lc);

    SilKit_TimeSyncService_Create(&g_timeSync, g_lifecycle);
    SilKit_TimeSyncService_SetSimulationStepHandler(
        g_timeSync, NULL, &SimStepHandler, 1000000ULL);

    SilKit_LifecycleService_StartLifecycle(g_lifecycle);
    printf("[RTE] Lifecycle started (Coordinated + TimeSync)\n");
}

void Rte_Can_Shutdown(void)
{
    if (g_canCtrl)    SilKit_CanController_Stop(g_canCtrl);
    if (g_participant) SilKit_Participant_Destroy(g_participant);
}

SilKit_Participant* Rte_GetParticipant(void) { return g_participant; }

uint16_t Rte_Read_CurrentSpeed(void) { return g_currentSpeed; }
uint16_t Rte_Read_TargetSpeed(void)  { return g_targetSpeed;  }

void Rte_Write_ThrottleCmd(uint8_t val) {
    g_throttleCmd = val;
    uint8_t d[1] = {val};
    SendCanFrame(CAN_ID_THROTTLE_CMD, d, 1);
}

void Rte_Write_BrakeCmd(uint8_t val) {
    g_brakeCmd = val;
    uint8_t d[1] = {val};
    SendCanFrame(CAN_ID_BRAKE_CMD, d, 1);
}

uint8_t Rte_Read_ThrottleCmd(void) { return g_throttleCmd; }
uint8_t Rte_Read_BrakeCmd(void)    { return g_brakeCmd;    }

void Rte_SimulateStep(void)
{
    uint8_t throttle = g_throttleCmd;
    uint8_t brake    = g_brakeCmd;

    if (brake > 0) {
        int16_t d = brake/5;
        g_currentSpeed = (g_currentSpeed>(uint16_t)d)
                          ? g_currentSpeed-d : 0;
    } else if (throttle > 0) {
        g_currentSpeed += (throttle/5)+1;
    } else {
        if (g_currentSpeed>0) g_currentSpeed--;
    }
    if (g_currentSpeed>200) g_currentSpeed=200;

    printf("\n[ECU] Speed=%3d km/h | Target=%3d km/h | "
           "Throttle=%3d%% | Brake=%3d%%\n",
           g_currentSpeed, g_targetSpeed, throttle, brake);

    uint8_t d[2] = {(uint8_t)(g_currentSpeed>>8),
                    (uint8_t)(g_currentSpeed&0xFF)};
    SendCanFrame(CAN_ID_CURRENT_SPEED, d, 2);
}
