#include "Rte_queue.h"
#include <stdio.h>

/* Queue handles */
QueueHandle_t xQueue_ThrottleCmd;
QueueHandle_t xQueue_BrakeCmd;
QueueHandle_t xQueue_CurrentSpeed;
QueueHandle_t xQueue_TargetSpeed;

/* Simulated plant state */
static uint16_t g_currentSpeed = 0;
static uint16_t g_targetSpeed  = 80;

void Rte_Init_Queues(void) {
    /* Each queue holds 1 item — latest value wins (like RTE last-is-best) */
    xQueue_ThrottleCmd   = xQueueCreate(1, sizeof(uint8_t));
    xQueue_BrakeCmd      = xQueueCreate(1, sizeof(uint8_t));
    xQueue_CurrentSpeed  = xQueueCreate(1, sizeof(uint16_t));
    xQueue_TargetSpeed   = xQueueCreate(1, sizeof(uint16_t));

    /* Seed initial values */
    xQueueOverwrite(xQueue_CurrentSpeed, &g_currentSpeed);
    xQueueOverwrite(xQueue_TargetSpeed,  &g_targetSpeed);

    uint8_t zero = 0;
    xQueueOverwrite(xQueue_ThrottleCmd, &zero);
    xQueueOverwrite(xQueue_BrakeCmd,    &zero);
}

/* ── SpeedController reads ── */
uint16_t Rte_Read_CurrentSpeed(void) {
    uint16_t val = 0;
    xQueuePeek(xQueue_CurrentSpeed, &val, 0);
    return val;
}
uint16_t Rte_Read_TargetSpeed(void) {
    uint16_t val = 0;
    xQueuePeek(xQueue_TargetSpeed, &val, 0);
    return val;
}

/* ── SpeedController write ── */
void Rte_Write_ThrottleCmd(uint8_t val) {
    xQueueOverwrite(xQueue_ThrottleCmd, &val);
}

/* ── BrakeController write ── */
void Rte_Write_BrakeCmd(uint8_t val) {
    xQueueOverwrite(xQueue_BrakeCmd, &val);
}

/* ── ActuatorSWC reads ── */
uint8_t Rte_Read_ThrottleCmd(void) {
    uint8_t val = 0;
    xQueuePeek(xQueue_ThrottleCmd, &val, 0);
    return val;
}
uint8_t Rte_Read_BrakeCmd(void) {
    uint8_t val = 0;
    xQueuePeek(xQueue_BrakeCmd, &val, 0);
    return val;
}

/* ── Plant simulation (runs every 100ms) ── */
void Rte_SimulateStep(void) {
    uint8_t throttle = 0, brake = 0;
    xQueuePeek(xQueue_ThrottleCmd, &throttle, 0);
    xQueuePeek(xQueue_BrakeCmd,    &brake,    0);

    /* Throttle accelerates, brake decelerates */
    if (brake > 0) {
        int16_t decel = brake / 5;
        g_currentSpeed = (g_currentSpeed > decel)
                         ? g_currentSpeed - decel : 0;
    } else if (throttle > 0) {
        g_currentSpeed += (throttle / 5) + 1;
    } else {
        if (g_currentSpeed > 0) g_currentSpeed--;
    }

    if (g_currentSpeed > 200) g_currentSpeed = 200;

    /* Write updated speed back to queue */
    xQueueOverwrite(xQueue_CurrentSpeed, &g_currentSpeed);

    printf("[vECU] Speed=%3d km/h | Target=%3d km/h | Throttle=%3d%% | Brake=%3d%%\n",
           g_currentSpeed, g_targetSpeed, throttle, brake);
}
