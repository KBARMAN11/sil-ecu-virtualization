#include "BrakeController.h"
#include "Rte_queue.h"

static int g_emergency = 0;

void BrakeController_Init(void) {}

void BrakeController_SetEmergency(int active) {
    g_emergency = active;
}

void BrakeController_RunStep(void) {
    uint16_t current = Rte_Read_CurrentSpeed();
    uint8_t  brake   = 0;

    if (g_emergency) {
        brake = 100;          /* full emergency brake */
    } else if (current > 90) {
        brake = 30;           /* gentle speed limiter above 90 km/h */
    }

    Rte_Write_BrakeCmd(brake);
}
