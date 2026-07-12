#include "Rte_stub.h"
#include <stdio.h>

static uint16_t g_currentSpeed = 0;
static uint16_t g_targetSpeed  = 80;
static uint8_t  g_throttle     = 0;

uint16_t Rte_Read_CurrentSpeed(void) { return g_currentSpeed; }
uint16_t Rte_Read_TargetSpeed(void)  { return g_targetSpeed;  }

void Rte_Write_Throttle(uint8_t val) {
    g_throttle = val;
}

void Rte_SimulateStep(void) {
    /* Better plant model: even 1% throttle moves the speed */
    if (g_throttle > 0) {
        g_currentSpeed += (g_throttle / 5) + 1;
    } else {
        /* Friction: slow down when no throttle */
        if (g_currentSpeed > 0) g_currentSpeed--;
    }
    if (g_currentSpeed > 200) g_currentSpeed = 200;

    printf("[vECU] Speed=%3d km/h | Target=%3d km/h | Throttle=%3d%%\n",
           g_currentSpeed, g_targetSpeed, g_throttle);
}
