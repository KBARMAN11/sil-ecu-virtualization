#include "silkit/capi/SilKit.h"
#include "silkit/capi/Can.h"
#include "silkit/capi/Participant.h"
#include "CanMonitor.h"
#include <stdio.h>
#include <string.h>
#include <windows.h>

/* Forward declaration — defined in Rte_can.c */
SilKit_Participant* Rte_GetParticipant(void);

static SilKit_CanController* g_monCtrl = NULL;
static int g_frameCount = 0;

const char* mon_decode_id(uint32_t id) {
    switch(id) {
        case 0x101: return "ThrottleCmd ";
        case 0x102: return "BrakeCmd    ";
        case 0x201: return "CurrentSpeed";
        case 0x202: return "TargetSpeed ";
        default:    return "Unknown     ";
    }
}

void MonFrameHandler(void* context,
                     SilKit_CanController* ctrl,
                     SilKit_CanFrameEvent* frameEvent)
{
    (void)context; (void)ctrl;
    g_frameCount++;
    SilKit_CanFrame* frame = frameEvent->frame;
    uint32_t id = frame->id;
    const uint8_t* data = frame->data.data;
    size_t len = frame->data.size;

    printf("[MON #%04d] ID=0x%03X | %s | ",
           g_frameCount, id, mon_decode_id(id));

    if ((id == 0x101 || id == 0x102) && len >= 1) {
        printf("%3d%%\n", data[0]);
    } else if ((id == 0x201 || id == 0x202) && len >= 2) {
        uint16_t val = (uint16_t)((data[0] << 8) | data[1]);
        printf("%3d km/h\n", val);
    } else {
        for (size_t i = 0; i < len; i++) printf("%02X ", data[i]);
        printf("\n");
    }
    fflush(stdout);
}

void CanMonitor_Init(void)
{
    SilKit_Participant* participant = Rte_GetParticipant();

    SilKit_CanController_Create(&g_monCtrl, participant,
                                 "CAN_Monitor", "CAN1");
    SilKit_HandlerId handlerId;
    SilKit_CanController_AddFrameHandler(
        g_monCtrl, NULL, &MonFrameHandler,
        SilKit_Direction_SendReceive, &handlerId);
    SilKit_CanController_Start(g_monCtrl);
    printf("[MON] CAN monitor started on CAN1\n");
    fflush(stdout);
}

void CanMonitor_PrintStats(void) {
    printf("[MON] Total frames: %d\n", g_frameCount);
}
