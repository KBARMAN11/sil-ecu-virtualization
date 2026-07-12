#ifndef RTE_STUB_H
#define RTE_STUB_H
#include <stdint.h>

uint16_t Rte_Read_CurrentSpeed(void);
uint16_t Rte_Read_TargetSpeed(void);
void     Rte_Write_Throttle(uint8_t val);
void     Rte_SimulateStep(void);

#endif
