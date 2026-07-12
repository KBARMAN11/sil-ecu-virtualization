#include "SpeedController.h"
#include "Rte_queue.h"

#define DEADBAND 2

void SpeedController_Init(void) {}

void SpeedController_RunStep(void) {
    uint16_t current = Rte_Read_CurrentSpeed();
    uint16_t target  = Rte_Read_TargetSpeed();
    int16_t  error   = (int16_t)(target - current);
    uint8_t  throttle = 0;

    if (error > DEADBAND) {
        throttle = (uint8_t)(error / 2);
        if (throttle < 5)   throttle = 5;
        if (throttle > 100) throttle = 100;
    }

    Rte_Write_ThrottleCmd(throttle);  /* writes to queue, not global */
}
