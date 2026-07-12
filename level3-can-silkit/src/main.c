#include <stdio.h>
#include <windows.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Rte_can.h"
#include "SpeedController.h"
#include "BrakeController.h"
#include "ActuatorSWC.h"
#include "CanMonitor.h"

/* 10ms task — fast control loop (still runs on FreeRTOS, free-running) */
void Task_10ms(void* p) {
    (void)p;
    for (;;) {
        SpeedController_RunStep();
        BrakeController_RunStep();
        ActuatorSWC_RunStep();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* 100ms task — now ONLY handles scenario scripting.
   Rte_SimulateStep() is driven by SIL Kit's TimeSyncService instead. */
void Task_100ms(void* p) {
    (void)p;
    uint32_t tick = 0;
    for (;;) {
        tick++;

        if (tick == 50) {
            printf("\n[SIM] *** EMERGENCY BRAKE ***\n\n");
            BrakeController_SetEmergency(1);
        }
        if (tick == 80) {
            printf("\n[SIM] *** BRAKE RELEASED ***\n\n");
            BrakeController_SetEmergency(0);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main(void) {
    printf("==========================================\n");
    printf("   vECU Level 3 - Virtual CAN via SIL Kit\n");
    printf("==========================================\n");

    /* Connect to SIL Kit — CAN frames replace queues */
    Rte_Init_Can();

    SpeedController_Init();
    BrakeController_Init();
    ActuatorSWC_Init();

    xTaskCreate(Task_10ms,  "Task_10ms",  1024, NULL, 2, NULL);
    xTaskCreate(Task_100ms, "Task_100ms", 1024, NULL, 1, NULL);

    /* Start lifecycle LAST — StartLifecycle blocks here and drives
       Rte_SimulateStep() via the TimeSync simulation step handler,
       while FreeRTOS tasks above keep running on Windows threads */
    Rte_Start_Lifecycle();

    vTaskStartScheduler();

    Rte_Can_Shutdown();
    return 0;
}

void vApplicationIdleHook(void) {}
void vApplicationMallocFailedHook(void) {}
