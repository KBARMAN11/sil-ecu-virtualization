#include <stdio.h>
#include <windows.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Rte_queue.h"
#include "SpeedController.h"
#include "BrakeController.h"
#include "ActuatorSWC.h"

/* 10ms — fast control loop */
void Task_10ms(void* p) {
    (void)p;
    for (;;) {
        SpeedController_RunStep();
        BrakeController_RunStep();
        ActuatorSWC_RunStep();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* 100ms — plant simulation + slow logic */
void Task_100ms(void* p) {
    (void)p;
    uint32_t tick = 0;
    for (;;) {
        Rte_SimulateStep();
        tick++;

        /* Simulate emergency brake at t=5s */
        if (tick == 50) {
            printf("\n[SIM] *** EMERGENCY BRAKE TRIGGERED ***\n\n");
            BrakeController_SetEmergency(1);
        }
        /* Release after 3s */
        if (tick == 80) {
            printf("\n[SIM] *** BRAKE RELEASED ***\n\n");
            BrakeController_SetEmergency(0);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main(void) {
    printf("==========================================\n");
    printf("   vECU Level 2 — Multi-SWC + Queues\n");
    printf("==========================================\n");

    Rte_Init_Queues();
    SpeedController_Init();
    BrakeController_Init();
    ActuatorSWC_Init();

    xTaskCreate(Task_10ms,  "Task_10ms",  1024, NULL, 2, NULL);
    xTaskCreate(Task_100ms, "Task_100ms", 1024, NULL, 1, NULL);

    vTaskStartScheduler();
    return 0;
}

void vApplicationIdleHook(void) {}
void vApplicationMallocFailedHook(void) {}
