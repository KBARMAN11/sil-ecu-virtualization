#include "ActuatorSWC.h"
#include "Rte_can.h"
#include <stdio.h>

void ActuatorSWC_Init(void) {}

void ActuatorSWC_RunStep(void) {
    uint8_t throttle = Rte_Read_ThrottleCmd();
    uint8_t brake    = Rte_Read_BrakeCmd();

    if (brake > 0 && throttle > 0) {
        printf("[ACT] BRAKE overrides THROTTLE: brake=%d%%\n", brake);
    }
}
