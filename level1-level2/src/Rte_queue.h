#ifndef RTE_QUEUE_H
#define RTE_QUEUE_H
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

/* Queue handles — one per signal (mimics RTE buffer per port) */
extern QueueHandle_t xQueue_ThrottleCmd;
extern QueueHandle_t xQueue_BrakeCmd;
extern QueueHandle_t xQueue_CurrentSpeed;
extern QueueHandle_t xQueue_TargetSpeed;

/* RTE API — same names, now backed by queues */
void     Rte_Init_Queues(void);

/* SpeedController ports */
uint16_t Rte_Read_CurrentSpeed(void);
uint16_t Rte_Read_TargetSpeed(void);
void     Rte_Write_ThrottleCmd(uint8_t val);

/* BrakeController ports */
void     Rte_Write_BrakeCmd(uint8_t val);

/* ActuatorSWC ports (receiver) */
uint8_t  Rte_Read_ThrottleCmd(void);
uint8_t  Rte_Read_BrakeCmd(void);

/* Plant simulation */
void     Rte_SimulateStep(void);

#endif
