#include "fmi3Functions.h"
#include "fmi3LsBusEthernet.h"
#include "fmi3LsBusUtil.h"
#include "fmi3LsBusUtilEthernet.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

static const fmi3LsBusEthernetMacAddressOctet SRC_MAC[6] = {0xAA,0xBB,0xCC,0x00,0x00,0x01};
static const fmi3LsBusEthernetMacAddressOctet DST_MAC[6] = {0xAA,0xBB,0xCC,0x00,0x00,0x02};
#define ETH_TYPE_THROTTLE  ((fmi3LsBusEthernetTypeOrLength)0x8801)
#define ETH_TYPE_BRAKE     ((fmi3LsBusEthernetTypeOrLength)0x8802)
#define ETH_TYPE_SPEED     ((fmi3LsBusEthernetTypeOrLength)0x8803)
#define ETH_SFD            ((fmi3LsBusEthernetStartDelimiter)0xD5)

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
    if (error > 0) {
        /* Proportional gain + feedforward tuned for 1000kg plant with DRAG_AREA=15
           feedforward=44 sustains ~80 km/h against drag at equilibrium */
        int16_t throttle = (error * 2) + 44;
        if (throttle < 5)   throttle = 5;
        if (throttle > 100) throttle = 100;
        s->throttleCmd = (fmi3UInt8)throttle;
    } else if (error < -1) {
        /* Slight overshoot — apply gentle brake */
        s->throttleCmd = 0;
    } else {
        /* error == 0 or -1: hold current throttle, avoid oscillation */
        s->throttleCmd = 44;
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
        s->currentSpeed = (s->currentSpeed > (fmi3UInt16)d) ? s->currentSpeed - d : 0;
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
    FMI3_LS_BUS_ETHERNET_CREATE_OP_TRANSMIT(&bufInfo, ETH_SFD,
        (fmi3LsBusEthernetMPacketFragmentCounter)0, FMI3_LS_BUS_TRUE,
        DST_MAC, SRC_MAC, ETH_TYPE_THROTTLE, 1, td);
    fmi3UInt8 bd[1] = { s->brakeCmd };
    FMI3_LS_BUS_ETHERNET_CREATE_OP_TRANSMIT(&bufInfo, ETH_SFD,
        (fmi3LsBusEthernetMPacketFragmentCounter)0, FMI3_LS_BUS_TRUE,
        DST_MAC, SRC_MAC, ETH_TYPE_BRAKE, 1, bd);
    fmi3UInt8 sd[2] = {
        (fmi3UInt8)(s->currentSpeed >> 8),
        (fmi3UInt8)(s->currentSpeed & 0xFF)
    };
    FMI3_LS_BUS_ETHERNET_CREATE_OP_TRANSMIT(&bufInfo, ETH_SFD,
        (fmi3LsBusEthernetMPacketFragmentCounter)0, FMI3_LS_BUS_TRUE,
        DST_MAC, SRC_MAC, ETH_TYPE_SPEED, 2, sd);
    s->txSize = (size_t)FMI3_LS_BUS_BUFFER_LENGTH(&bufInfo);
}

static void UnpackRxFrames(FMUInstance* s, const fmi3UInt8* data, size_t size) {
    const fmi3UInt8* ptr = data;
    const fmi3UInt8* end = data + size;
    while (ptr < end) {
        const fmi3LsBusOperationHeader* hdr = (const fmi3LsBusOperationHeader*)ptr;
        if (hdr->length == 0 || ptr + hdr->length > end) break;
        if (hdr->opCode == FMI3_LS_BUS_ETHERNET_OP_TRANSMIT) {
            const fmi3LsBusEthernetOperationTransmit* tx =
                (const fmi3LsBusEthernetOperationTransmit*)ptr;
            /* ETH_TYPE_SPEED on Rx = plant speed feedback → update currentSpeed */
            if (tx->typeOrLength == ETH_TYPE_SPEED && tx->dataLength >= 2)
                s->currentSpeed = (fmi3UInt16)(((fmi3UInt16)tx->data[0] << 8) | tx->data[1]);
        }
        ptr += hdr->length;
    }
}

FMI3_Export fmi3InstantiateCoSimulationTYPE fmi3InstantiateCoSimulation;
fmi3Instance fmi3InstantiateCoSimulation(
    fmi3String instanceName, fmi3String instantiationToken,
    fmi3String resourcePath, fmi3Boolean visible, fmi3Boolean loggingOn,
    fmi3Boolean eventModeUsed, fmi3Boolean earlyReturnAllowed,
    const fmi3ValueReference requiredIntermediateVariables[],
    size_t nRequiredIntermediateVariables,
    fmi3InstanceEnvironment instanceEnvironment,
    fmi3LogMessageCallback logMessage,
    fmi3IntermediateUpdateCallback intermediateUpdate)
{
    FMUInstance* s = (FMUInstance*)calloc(1, sizeof(FMUInstance));
    if (!s) return NULL;
    s->targetSpeed = 80;
    return (fmi3Instance)s;
}

FMI3_Export fmi3FreeInstanceTYPE fmi3FreeInstance;
void fmi3FreeInstance(fmi3Instance instance) { free(instance); }

FMI3_Export fmi3EnterInitializationModeTYPE fmi3EnterInitializationMode;
fmi3Status fmi3EnterInitializationMode(fmi3Instance i, fmi3Boolean td,
    fmi3Float64 tol, fmi3Float64 st, fmi3Boolean sd, fmi3Float64 stop)
{ return fmi3OK; }

FMI3_Export fmi3ExitInitializationModeTYPE fmi3ExitInitializationMode;
fmi3Status fmi3ExitInitializationMode(fmi3Instance instance) { return fmi3OK; }

FMI3_Export fmi3EnterConfigurationModeTYPE fmi3EnterConfigurationMode;
fmi3Status fmi3EnterConfigurationMode(fmi3Instance instance) { return fmi3OK; }

FMI3_Export fmi3ExitConfigurationModeTYPE fmi3ExitConfigurationMode;
fmi3Status fmi3ExitConfigurationMode(fmi3Instance instance) { return fmi3OK; }

FMI3_Export fmi3EnterStepModeTYPE fmi3EnterStepMode;
fmi3Status fmi3EnterStepMode(fmi3Instance instance) { return fmi3OK; }

FMI3_Export fmi3EnterEventModeTYPE fmi3EnterEventMode;
fmi3Status fmi3EnterEventMode(fmi3Instance instance) { return fmi3OK; }

FMI3_Export fmi3DoStepTYPE fmi3DoStep;
fmi3Status fmi3DoStep(fmi3Instance instance,
    fmi3Float64 currentCommunicationPoint, fmi3Float64 communicationStepSize,
    fmi3Boolean noSetFMUStatePriorToCurrentPoint,
    fmi3Boolean* eventHandlingNeeded, fmi3Boolean* terminateSimulation,
    fmi3Boolean* earlyReturn, fmi3Float64* lastSuccessfulTime)
{
    FMUInstance* s = (FMUInstance*)instance;
    SpeedController_Run(s); BrakeController_Run(s);
    /* PlantSim_Run removed — currentSpeed is now driven by the external Plant FMU
       via ETH Rx frames set by the master algorithm each step */
    PackTxFrames(s);
    printf("[FMU-ETH] Speed=%3d | Target=%3d | Throttle=%3d%% | Brake=%3d%%\n",
           s->currentSpeed, s->targetSpeed, s->throttleCmd, s->brakeCmd);
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
    s->currentSpeed = 0; s->throttleCmd = 0; s->brakeCmd = 0;
    return fmi3OK;
}

FMI3_Export fmi3GetBinaryTYPE fmi3GetBinary;
fmi3Status fmi3GetBinary(fmi3Instance instance,
    const fmi3ValueReference valueReferences[], size_t nValueReferences,
    size_t valueSizes[], fmi3Binary values[], size_t nValues)
{
    FMUInstance* s = (FMUInstance*)instance;
    for (size_t i = 0; i < nValueReferences; i++)
        if (valueReferences[i] == 2) {
            values[i] = (fmi3Binary)s->txBuffer;
            valueSizes[i] = s->txSize;
        }
    return fmi3OK;
}

FMI3_Export fmi3SetBinaryTYPE fmi3SetBinary;
fmi3Status fmi3SetBinary(fmi3Instance instance,
    const fmi3ValueReference valueReferences[], size_t nValueReferences,
    const size_t valueSizes[], const fmi3Binary values[], size_t nValues)
{
    FMUInstance* s = (FMUInstance*)instance;
    for (size_t i = 0; i < nValueReferences; i++)
        if (valueReferences[i] == 4)
            UnpackRxFrames(s, (const fmi3UInt8*)values[i], valueSizes[i]);
    return fmi3OK;
}

FMI3_Export fmi3GetClockTYPE fmi3GetClock;
fmi3Status fmi3GetClock(fmi3Instance instance,
    const fmi3ValueReference valueReferences[], size_t nValueReferences,
    fmi3Clock values[])
{
    for (size_t i = 0; i < nValueReferences; i++) values[i] = fmi3ClockActive;
    return fmi3OK;
}

FMI3_Export fmi3SetClockTYPE fmi3SetClock;
fmi3Status fmi3SetClock(fmi3Instance instance,
    const fmi3ValueReference valueReferences[], size_t nValueReferences,
    const fmi3Clock values[]) { return fmi3OK; }

FMI3_Export fmi3GetIntervalDecimalTYPE fmi3GetIntervalDecimal;
fmi3Status fmi3GetIntervalDecimal(fmi3Instance instance,
    const fmi3ValueReference valueReferences[], size_t nValueReferences,
    fmi3Float64 intervals[], fmi3IntervalQualifier qualifiers[])
{
    for (size_t i = 0; i < nValueReferences; i++) {
        intervals[i]  = 0.001;
        qualifiers[i] = fmi3IntervalChanged;
    }
    return fmi3OK;
}

FMI3_Export fmi3UpdateDiscreteStatesTYPE fmi3UpdateDiscreteStates;
fmi3Status fmi3UpdateDiscreteStates(fmi3Instance instance,
    fmi3Boolean* discreteStatesNeedUpdate, fmi3Boolean* terminateSimulation,
    fmi3Boolean* nominalsOfContinuousStatesChanged,
    fmi3Boolean* valuesOfContinuousStatesChanged,
    fmi3Boolean* nextEventTimeDefined, fmi3Float64* nextEventTime)
{
    *discreteStatesNeedUpdate          = fmi3False;
    *terminateSimulation               = fmi3False;
    *nominalsOfContinuousStatesChanged = fmi3False;
    *valuesOfContinuousStatesChanged   = fmi3False;
    *nextEventTimeDefined              = fmi3False;
    *nextEventTime                     = 0.0;
    return fmi3OK;
}

FMI3_Export fmi3GetVersionTYPE fmi3GetVersion;
const char* fmi3GetVersion(void) { return "3.0"; }

FMI3_Export fmi3SetDebugLoggingTYPE fmi3SetDebugLogging;
fmi3Status fmi3SetDebugLogging(fmi3Instance instance,
    fmi3Boolean loggingOn, size_t nCategories,
    const fmi3String categories[]) { return fmi3OK; }

FMI3_Export fmi3GetFloat32TYPE fmi3GetFloat32;
fmi3Status fmi3GetFloat32(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, fmi3Float32 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3GetFloat64TYPE fmi3GetFloat64;
fmi3Status fmi3GetFloat64(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, fmi3Float64 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3GetInt8TYPE fmi3GetInt8;
fmi3Status fmi3GetInt8(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, fmi3Int8 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3GetInt16TYPE fmi3GetInt16;
fmi3Status fmi3GetInt16(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, fmi3Int16 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3GetInt32TYPE fmi3GetInt32;
fmi3Status fmi3GetInt32(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, fmi3Int32 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3GetInt64TYPE fmi3GetInt64;
fmi3Status fmi3GetInt64(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, fmi3Int64 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3GetUInt8TYPE fmi3GetUInt8;
fmi3Status fmi3GetUInt8(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, fmi3UInt8 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3GetUInt16TYPE fmi3GetUInt16;
fmi3Status fmi3GetUInt16(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, fmi3UInt16 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3GetUInt32TYPE fmi3GetUInt32;
fmi3Status fmi3GetUInt32(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, fmi3UInt32 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3GetUInt64TYPE fmi3GetUInt64;
fmi3Status fmi3GetUInt64(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, fmi3UInt64 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3GetBooleanTYPE fmi3GetBoolean;
fmi3Status fmi3GetBoolean(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, fmi3Boolean v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3GetStringTYPE fmi3GetString;
fmi3Status fmi3GetString(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, fmi3String v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3SetFloat32TYPE fmi3SetFloat32;
fmi3Status fmi3SetFloat32(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, const fmi3Float32 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3SetFloat64TYPE fmi3SetFloat64;
fmi3Status fmi3SetFloat64(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, const fmi3Float64 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3SetInt8TYPE fmi3SetInt8;
fmi3Status fmi3SetInt8(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, const fmi3Int8 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3SetInt16TYPE fmi3SetInt16;
fmi3Status fmi3SetInt16(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, const fmi3Int16 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3SetInt32TYPE fmi3SetInt32;
fmi3Status fmi3SetInt32(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, const fmi3Int32 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3SetInt64TYPE fmi3SetInt64;
fmi3Status fmi3SetInt64(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, const fmi3Int64 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3SetUInt8TYPE fmi3SetUInt8;
fmi3Status fmi3SetUInt8(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, const fmi3UInt8 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3SetUInt16TYPE fmi3SetUInt16;
fmi3Status fmi3SetUInt16(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, const fmi3UInt16 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3SetUInt32TYPE fmi3SetUInt32;
fmi3Status fmi3SetUInt32(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, const fmi3UInt32 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3SetUInt64TYPE fmi3SetUInt64;
fmi3Status fmi3SetUInt64(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, const fmi3UInt64 v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3SetBooleanTYPE fmi3SetBoolean;
fmi3Status fmi3SetBoolean(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, const fmi3Boolean v[], size_t nv) { return fmi3OK; }

FMI3_Export fmi3SetStringTYPE fmi3SetString;
fmi3Status fmi3SetString(fmi3Instance i, const fmi3ValueReference vr[], size_t nvr, const fmi3String v[], size_t nv) { return fmi3OK; }
