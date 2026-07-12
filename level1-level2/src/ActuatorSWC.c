#include "ActuatorSWC.h"
#include "Rte_queue.h"
#include <stdio.h>

void ActuatorSWC_Init(void) {}

void ActuatorSWC_RunStep(void) {
    uint8_t throttle = Rte_Read_ThrottleCmd();
    uint8_t brake    = Rte_Read_BrakeCmd();

    /* In a real ECU this would write PWM registers.
       Here we just show arbitration logic. */
    if (brake > 0 && throttle > 0) {
        /* Brake always wins — safety rule */
        printf("[ACT] BRAKE overrides THROTTLE: brake=%d%%\n", brake);
    }
    /* Real HW output would go here: Pwm_SetDutyCycle(...) */
}
