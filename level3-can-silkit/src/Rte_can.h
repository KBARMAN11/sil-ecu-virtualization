#ifndef RTE_CAN_H
#define RTE_CAN_H
#include <stdint.h>

/* CAN Message IDs — one per signal */
#define CAN_ID_THROTTLE_CMD    0x101
#define CAN_ID_BRAKE_CMD       0x102
#define CAN_ID_CURRENT_SPEED   0x201
#define CAN_ID_TARGET_SPEED    0x202

/* Init — connects to SIL Kit registry */
void Rte_Init_Can(void);
void Rte_Can_Shutdown(void);

/* SpeedController ports */
uint16_t Rte_Read_CurrentSpeed(void);
uint16_t Rte_Read_TargetSpeed(void);
void     Rte_Write_ThrottleCmd(uint8_t val);

/* BrakeController ports */
void     Rte_Write_BrakeCmd(uint8_t val);

/* ActuatorSWC ports */
uint8_t  Rte_Read_ThrottleCmd(void);
uint8_t  Rte_Read_BrakeCmd(void);

/* Plant simulation */
void     Rte_SimulateStep(void);

#endif
void Rte_Start_Lifecycle(void);
