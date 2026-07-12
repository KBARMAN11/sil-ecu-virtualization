#include "fmi3Functions.h"
#include "fmi3LsBusCan.h"
#include "fmi3LsBusUtil.h"
#include "fmi3LsBusUtilCan.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define CAN_ID_THROTTLE_CMD   0x101
#define CAN_ID_BRAKE_CMD      0x102
#define CAN_ID_CURRENT_SPEED  0x201
#define CAN_ID_TARGET_SPEED   0x202

typedef struct {
    fmi3UInt16  currentSpeed;
    fmi3UInt16  targetSpeed;
    fmi3UInt8   throttleCmd;
    fmi3UInt8   brakeCmd;
    int         emergency;
    fmi3UInt8   txBuffer[1024];
    size_t      txSize;
} FMUInstance;

static void SpeedController_Run(FMUInstance* s) {
    int16_t error = (int16_t)(s->targetSpeed - s->currentSpeed);
    if (error > 2) {
        s->throttleCmd = (fmi3UInt8)(error / 2);
        if (s->throttleCmd < 5)   s->throttleCmd = 5;
        if (s->throttleCmd > 100) s->throttleCmd = 100;
    } else {
        s->throttleCmd = 0;
    }
}

static void BrakeController_Run(FMUInstance* s) {
    if (s->emergency)              s->brakeCmd = 100;
    else if (s->currentSpeed > 90) s->brakeCmd = 30;
    else                           s->brakeCmd = 0;
}

static void PlantSim_Run(FMUInstance* s) {
    if (s->brakeCmd > 0) {
        int16_t d = s->brakeCmd / 5;
        s->currentSpeed = (s->currentSpeed > (fmi3UInt16)d)
                          ? s->currentSpeed - d : 0;
    } else if (s->throttleCmd > 0) {
        s->currentSpeed += (s->throttleCmd / 5) + 1;
    } else {
        if (s->currentSpeed > 0) s->currentSpeed--;
    }
    if (s->currentSpeed > 200) s->currentSpeed = 200;
}

static void PackTxFrames(FMUInstance* s) {
    fmi3LsBusUtilBufferInfo bufInfo;
    FMI3_LS_BUS_BUFFER_INFO_INIT(&bufInfo, s->txBuffer, sizeof(s->txBuffer));

    fmi3UInt8 td[1] = { s->throttleCmd };
    FMI3_LS_BUS_CAN_CREATE_OP_CAN_TRANSMIT(&bufInfo,
        CAN_ID_THROTTLE_CMD, FMI3_LS_BUS_FALSE, FMI3_LS_BUS_FALSE, 1, td);

    fmi3UInt8 bd[1] = { s->brakeCmd };
    FMI3_LS_BUS_CAN_CREATE_OP_CAN_TRANSMIT(&bufInfo,
        CAN_ID_BRAKE_CMD, FMI3_LS_BUS_FALSE, FMI3_LS_BUS_FALSE, 1, bd);

    fmi3UInt8 sd[2] = {
        (fmi3UInt8)(s->currentSpeed >> 8),
        (fmi3UInt8)(s->currentSpeed & 0xFF)
    };
    FMI3_LS_BUS_CAN_CREATE_OP_CAN_TRANSMIT(&bufInfo,
        CAN_ID_CURRENT_SPEED, FMI3_LS_BUS_FALSE, FMI3_LS_BUS_FALSE, 2, sd);

    s->txSize = (size_t)FMI3_LS_BUS_BUFFER_LENGTH(&bufInfo);
}

static void UnpackRxFrames(FMUInstance* s,
                            const fmi3UInt8* data, size_t size) {
    const fmi3UInt8* ptr = data;
    const fmi3UInt8* end = data + size;
    while (ptr + sizeof(fmi3LsBusOperationHeader) <= end) {
        const fmi3LsBusOperationHeader* hdr =
            (const fmi3LsBusOperationHeader*)ptr;
        if (hdr->length == 0) break;
        if (hdr->opCode == FMI3_LS_BUS_CAN_OP_CAN_TRANSMIT) {
            const fmi3LsBusCanOperationCanTransmit* tx =
                (const fmi3LsBusCanOperationCanTransmit*)ptr;
            if (tx->id == CAN_ID_TARGET_SPEED && tx->dataLength >= 2)
                s->targetSpeed = (fmi3UInt16)((tx->data[0]<<8)|tx->data[1]);
        }
        ptr += hdr->length;
    }
}

/* ── FMI 3.0 exports ── */

FMI3_Export fmi3InstantiateCoSimulationTYPE fmi3InstantiateCoSimulation;
fmi3Instance fmi3InstantiateCoSimulation(
    fmi3String                instanceName,
    fmi3String                instantiationToken,
    fmi3String                resourcePath,
    fmi3Boolean               visible,
    fmi3Boolean               loggingOn,
    fmi3Boolean               eventModeUsed,
    fmi3Boolean               earlyReturnAllowed,
    const fmi3ValueReference  requiredIntermediateVariables[],
    size_t                    nRequiredIntermediateVariables,
    fmi3InstanceEnvironment   instanceEnvironment,
    fmi3LogMessageCallback    logMessage,
    fmi3IntermediateUpdateCallback intermediateUpdate)
{
    FMUInstance* s = (FMUInstance*)calloc(1, sizeof(FMUInstance));
    if (!s) return NULL;
    s->targetSpeed  = 80;
    s->currentSpeed = 0;
    s->throttleCmd  = 0;
    s->brakeCmd     = 0;
    s->emergency    = 0;
    printf("[FMU] vECU SpeedController instantiated\n");
    fflush(stdout);
    return (fmi3Instance)s;
}

FMI3_Export fmi3FreeInstanceTYPE fmi3FreeInstance;
void fmi3FreeInstance(fmi3Instance instance) {
    if (instance) free(instance);
}

FMI3_Export fmi3EnterInitializationModeTYPE fmi3EnterInitializationMode;
fmi3Status fmi3EnterInitializationMode(
    fmi3Instance instance,
    fmi3Boolean  toleranceDefined,
    fmi3Float64  tolerance,
    fmi3Float64  startTime,
    fmi3Boolean  stopTimeDefined,
    fmi3Float64  stopTime)
{ return fmi3OK; }

FMI3_Export fmi3ExitInitializationModeTYPE fmi3ExitInitializationMode;
fmi3Status fmi3ExitInitializationMode(fmi3Instance instance) {
    FMUInstance* s = (FMUInstance*)instance;
    PackTxFrames(s);
    return fmi3OK;
}

FMI3_Export fmi3DoStepTYPE fmi3DoStep;
fmi3Status fmi3DoStep(
    fmi3Instance instance,
    fmi3Float64  currentCommunicationPoint,
    fmi3Float64  communicationStepSize,
    fmi3Boolean  noSetFMUStatePriorToCurrentPoint,
    fmi3Boolean* eventHandlingNeeded,
    fmi3Boolean* terminateSimulation,
    fmi3Boolean* earlyReturn,
    fmi3Float64* lastSuccessfulTime)
{
    FMUInstance* s = (FMUInstance*)instance;
    SpeedController_Run(s);
    BrakeController_Run(s);
    PlantSim_Run(s);
    PackTxFrames(s);
    printf("[FMU] Speed=%3d km/h | Target=%3d km/h | "
           "Throttle=%3d%% | Brake=%3d%%\n",
           s->currentSpeed, s->targetSpeed,
           s->throttleCmd, s->brakeCmd);
    fflush(stdout);
    *eventHandlingNeeded = fmi3False;
    *terminateSimulation = fmi3False;
    *earlyReturn         = fmi3False;
    *lastSuccessfulTime  = currentCommunicationPoint + communicationStepSize;
    return fmi3OK;
}

FMI3_Export fmi3TerminateTYPE fmi3Terminate;
fmi3Status fmi3Terminate(fmi3Instance instance) { return fmi3OK; }

FMI3_Export fmi3ResetTYPE fmi3Reset;
fmi3Status fmi3Reset(fmi3Instance instance) {
    FMUInstance* s = (FMUInstance*)instance;
    s->currentSpeed = 0;
    s->throttleCmd  = 0;
    s->brakeCmd     = 0;
    return fmi3OK;
}

FMI3_Export fmi3GetBinaryTYPE fmi3GetBinary;
fmi3Status fmi3GetBinary(
    fmi3Instance             instance,
    const fmi3ValueReference valueReferences[],
    size_t                   nValueReferences,
    size_t                   valueSizes[],
    fmi3Binary               values[],
    size_t                   nValues)
{
    FMUInstance* s = (FMUInstance*)instance;
    for (size_t i = 0; i < nValueReferences; i++) {
        if (valueReferences[i] == 2) {
            values[i]     = (fmi3Binary)s->txBuffer;
            valueSizes[i] = s->txSize;
        }
    }
    return fmi3OK;
}

FMI3_Export fmi3SetBinaryTYPE fmi3SetBinary;
fmi3Status fmi3SetBinary(
    fmi3Instance             instance,
    const fmi3ValueReference valueReferences[],
    size_t                   nValueReferences,
    const size_t             valueSizes[],
    const fmi3Binary         values[],
    size_t                   nValues)
{
    FMUInstance* s = (FMUInstance*)instance;
    for (size_t i = 0; i < nValueReferences; i++) {
        if (valueReferences[i] == 4)
            UnpackRxFrames(s, (const fmi3UInt8*)values[i], valueSizes[i]);
    }
    return fmi3OK;
}

FMI3_Export fmi3GetClockTYPE fmi3GetClock;
fmi3Status fmi3GetClock(
    fmi3Instance             instance,
    const fmi3ValueReference valueReferences[],
    size_t                   nValueReferences,
    fmi3Clock                values[])
{
    for (size_t i = 0; i < nValueReferences; i++)
        values[i] = fmi3ClockActive;
    return fmi3OK;
}

FMI3_Export fmi3SetClockTYPE fmi3SetClock;
fmi3Status fmi3SetClock(
    fmi3Instance             instance,
    const fmi3ValueReference valueReferences[],
    size_t                   nValueReferences,
    const fmi3Clock          values[])
{ return fmi3OK; }

FMI3_Export fmi3EnterEventModeTYPE fmi3EnterEventMode;
fmi3Status fmi3EnterEventMode(fmi3Instance instance) { return fmi3OK; }


/* Required version function */
FMI3_Export fmi3GetVersionTYPE fmi3GetVersion;
const char* fmi3GetVersion(void) { return "3.0"; }
