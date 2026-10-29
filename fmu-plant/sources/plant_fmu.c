/*
 * plant_fmu.c
 *
 * FMI 3.0 Co-Simulation FMU — Longitudinal Vehicle Plant Model
 *
 * Inputs  (from vECU FMU):
 *   throttle_cmd  [0..100]   VR 0
 *   brake_cmd     [0..100]   VR 1
 *
 * Output  (to vECU FMU):
 *   current_speed [km/h]     VR 2
 *
 * Physics (per 0.1 s step):
 *   throttle_force = throttle_cmd * 30.0          (max 3000 N)
 *   brake_force    = brake_cmd    * 50.0          (max 5000 N)
 *   drag_force     = 0.5 * Cd * rho * v^2        (Cd=0.3, rho=1.2 kg/m^3)
 *   net_force      = throttle_force - brake_force - drag_force
 *   acceleration   = net_force / 1000.0           (mass = 1000 kg)
 *   v_ms          += acceleration * 0.1            (integrate, clamped >= 0)
 *   current_speed  = v_ms * 3.6                   (m/s -> km/h)
 *
 * Build (Windows, MinGW-w64):
 *   gcc -O0 -g3 -Wall -shared -o binaries/x86_64-windows/plant_fmu.dll sources/plant_fmu.c
 *
 * Package:
 *   jar cf ../vECU_Plant.fmu modelDescription.xml binaries sources documentation
 *
 * Validate:
 *   fmu-check.herokuapp.com -> upload vECU_Plant.fmu -> expect zero errors
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ── FMI 3.0 type definitions (minimal, no external headers needed) ── */
typedef void*        fmi3Instance;
typedef unsigned int fmi3ValueReference;
typedef int          fmi3Boolean;
typedef double       fmi3Float64;
typedef int          fmi3Int32;
typedef unsigned int fmi3UInt32;
typedef char         fmi3Char;
typedef const char*  fmi3String;
typedef void*        fmi3FMUState;
typedef void*        fmi3Binary;
typedef size_t       fmi3BinarySize;
typedef unsigned long long fmi3UInt64;
typedef unsigned char      fmi3Byte;

#define fmi3True  1
#define fmi3False 0

typedef enum {
    fmi3OK      = 0,
    fmi3Warning = 1,
    fmi3Discard = 2,
    fmi3Error   = 3,
    fmi3Fatal   = 4
} fmi3Status;

typedef enum {
    fmi3ModeInstantiated          = 0,
    fmi3ModeInitializationMode    = 1,
    fmi3ModeEventMode             = 2,
    fmi3ModeStepMode              = 3,
    fmi3ModeTerminated            = 4
} fmi3Mode;

/* Callback typedefs */
typedef void (*fmi3LogMessageCallback)(
    fmi3Instance instance, fmi3String instanceName,
    fmi3Status status, fmi3String category, fmi3String message);

typedef void (*fmi3ClockUpdateCallback)(fmi3Instance instance);
typedef void (*fmi3LockPreemptionCallback)(void);
typedef void (*fmi3UnlockPreemptionCallback)(void);
typedef void (*fmi3IntermediateUpdateCallback)(
    fmi3Instance instance, fmi3Float64 intermediateUpdateTime,
    fmi3Boolean clocksTicked, fmi3Boolean intermediateVariableSetRequested,
    fmi3Boolean intermediateVariableGetAllowed,
    fmi3Boolean intermediateStepFinished, fmi3Boolean canReturnEarly,
    fmi3Boolean* earlyReturnRequested, fmi3Float64* earlyReturnTime);

/* ── Value references ── */
#define VR_THROTTLE_CMD  0u
#define VR_BRAKE_CMD     1u
#define VR_CURRENT_SPEED 2u

/* ── Physics constants ── */
#define VEHICLE_MASS_KG     1000.0
#define THROTTLE_GAIN       30.0       /* N per throttle unit */
#define BRAKE_GAIN          50.0       /* N per brake unit    */
#define DRAG_CD             0.3
#define AIR_DENSITY         1.2        /* kg/m^3 */
#define DRAG_AREA           15.0       /* effective drag area — tuned so throttle=80 converges ~80 km/h */
#define DT                  0.1        /* step size, seconds  */

/* ── Internal FMU state ── */
typedef struct {
    /* inputs */
    fmi3Float64 throttle_cmd;
    fmi3Float64 brake_cmd;

    /* output */
    fmi3Float64 current_speed_kmh;

    /* internal state */
    fmi3Float64 v_ms;          /* speed in m/s */
    fmi3Float64 sim_time;

    /* FMI bookkeeping */
    fmi3Mode    mode;
    char        instance_name[64];

    /* callbacks */
    fmi3LogMessageCallback logMessage;
} PlantFMU;

static void log_msg(PlantFMU* s, fmi3Status status, const char* category,
                    const char* msg)
{
    if (s && s->logMessage)
        s->logMessage((fmi3Instance)s, s->instance_name,
                      status, category, msg);
    else
        printf("[PlantFMU][%s] %s\n", category, msg);
}

/* ── Physics step ── */
static void physics_step(PlantFMU* s)
{
    double throttle_force = s->throttle_cmd * THROTTLE_GAIN;
    double brake_force    = s->brake_cmd    * BRAKE_GAIN;
    double drag_force     = 0.5 * DRAG_CD * AIR_DENSITY * DRAG_AREA
                            * s->v_ms * s->v_ms;

    double net_force   = throttle_force - brake_force - drag_force;
    double accel       = net_force / VEHICLE_MASS_KG;

    s->v_ms += accel * DT;
    if (s->v_ms < 0.0) s->v_ms = 0.0;

    s->current_speed_kmh = s->v_ms * 3.6;

    printf("[PlantFMU] t=%.2f  thr=%.1f  brk=%.1f  v=%.2f km/h\n",
           s->sim_time, s->throttle_cmd, s->brake_cmd,
           s->current_speed_kmh);
}

/* ═══════════════════════════════════════════════════
   FMI 3.0 LIFECYCLE FUNCTIONS
   ═══════════════════════════════════════════════════ */

#ifdef _WIN32
  #define FMI3_EXPORT __declspec(dllexport)
#else
  #define FMI3_EXPORT __attribute__((visibility("default")))
#endif

/* ── fmi3GetVersion ── */
FMI3_EXPORT fmi3String fmi3GetVersion(void)
{
    return "3.0";
}

/* ── fmi3InstantiateCoSimulation ── */
FMI3_EXPORT fmi3Instance fmi3InstantiateCoSimulation(
    fmi3String instanceName,
    fmi3String instantiationToken,
    fmi3String resourcePath,
    fmi3Boolean visible,
    fmi3Boolean loggingOn,
    fmi3Boolean eventModeUsed,
    fmi3Boolean earlyReturnAllowed,
    const fmi3ValueReference* requiredIntermediateVariables,
    size_t nRequiredIntermediateVariables,
    fmi3Instance instanceEnvironment,
    fmi3LogMessageCallback logMessage,
    fmi3ClockUpdateCallback clockUpdate,
    fmi3LockPreemptionCallback lockPreemption,
    fmi3UnlockPreemptionCallback unlockPreemption,
    fmi3IntermediateUpdateCallback intermediateUpdate)
{
    (void)instantiationToken; (void)resourcePath; (void)visible;
    (void)loggingOn; (void)eventModeUsed; (void)earlyReturnAllowed;
    (void)requiredIntermediateVariables; (void)nRequiredIntermediateVariables;
    (void)instanceEnvironment; (void)clockUpdate;
    (void)lockPreemption; (void)unlockPreemption; (void)intermediateUpdate;

    PlantFMU* s = (PlantFMU*)calloc(1, sizeof(PlantFMU));
    if (!s) return NULL;

    strncpy(s->instance_name, instanceName ? instanceName : "PlantFMU",
            sizeof(s->instance_name) - 1);
    s->logMessage        = logMessage;
    s->throttle_cmd      = 0.0;
    s->brake_cmd         = 0.0;
    s->v_ms              = 0.0;
    s->current_speed_kmh = 0.0;
    s->sim_time          = 0.0;
    s->mode              = fmi3ModeInstantiated;

    log_msg(s, fmi3OK, "info", "PlantFMU instantiated");
    return (fmi3Instance)s;
}

/* ── fmi3FreeInstance ── */
FMI3_EXPORT void fmi3FreeInstance(fmi3Instance instance)
{
    if (instance) free(instance);
}

/* ── fmi3EnterInitializationMode ── */
FMI3_EXPORT fmi3Status fmi3EnterInitializationMode(
    fmi3Instance instance,
    fmi3Boolean toleranceDefined, fmi3Float64 tolerance,
    fmi3Float64 startTime,
    fmi3Boolean stopTimeDefined, fmi3Float64 stopTime)
{
    (void)toleranceDefined; (void)tolerance;
    (void)stopTimeDefined; (void)stopTime;
    PlantFMU* s = (PlantFMU*)instance;
    s->sim_time = startTime;
    s->mode = fmi3ModeInitializationMode;
    log_msg(s, fmi3OK, "info", "Entering initialization mode");
    return fmi3OK;
}

/* ── fmi3ExitInitializationMode ── */
FMI3_EXPORT fmi3Status fmi3ExitInitializationMode(fmi3Instance instance)
{
    PlantFMU* s = (PlantFMU*)instance;
    s->mode = fmi3ModeStepMode;
    log_msg(s, fmi3OK, "info", "Exiting initialization mode");
    return fmi3OK;
}

/* ── fmi3DoStep ── */
FMI3_EXPORT fmi3Status fmi3DoStep(
    fmi3Instance instance,
    fmi3Float64  currentCommunicationPoint,
    fmi3Float64  communicationStepSize,
    fmi3Boolean  noSetFMUStatePriorToCurrentPoint,
    fmi3Boolean* eventHandlingNeeded,
    fmi3Boolean* terminate,
    fmi3Boolean* earlyReturn,
    fmi3Float64* lastSuccessfulTime)
{
    (void)noSetFMUStatePriorToCurrentPoint;
    PlantFMU* s = (PlantFMU*)instance;

    /* Validate communicationStepSize is a positive integer multiple of DT.
     * Fix: previously physics_step() ran once regardless of h, causing elapsed time mismatch for h != DT.
     */
    int steps = (int)(communicationStepSize / DT + 0.5);
    if (steps < 1 ||
        fabs(steps * DT - communicationStepSize) > 1e-9) {
        fprintf(stderr,
            "[PlantFMU] Invalid step size h=%.6f — "
            "must be a positive integer multiple of DT=%.6f\n",
            communicationStepSize, DT);
        if (terminate) *terminate = fmi3True;
        return fmi3Error;
    }

    /* Run physics exactly 'steps' times with held inputs */
    s->sim_time = currentCommunicationPoint;
    for (int i = 0; i < steps; i++) {
        physics_step(s);
        s->sim_time += DT;
    }

    if (eventHandlingNeeded) *eventHandlingNeeded = fmi3False;
    if (terminate)           *terminate           = fmi3False;
    if (earlyReturn)         *earlyReturn         = fmi3False;
    if (lastSuccessfulTime)  *lastSuccessfulTime  = s->sim_time;

    return fmi3OK;
}

/* ── fmi3Terminate ── */
FMI3_EXPORT fmi3Status fmi3Terminate(fmi3Instance instance)
{
    PlantFMU* s = (PlantFMU*)instance;
    s->mode = fmi3ModeTerminated;
    log_msg(s, fmi3OK, "info", "PlantFMU terminated");
    return fmi3OK;
}

/* ── fmi3Reset ── */
FMI3_EXPORT fmi3Status fmi3Reset(fmi3Instance instance)
{
    PlantFMU* s = (PlantFMU*)instance;
    s->throttle_cmd      = 0.0;
    s->brake_cmd         = 0.0;
    s->v_ms              = 0.0;
    s->current_speed_kmh = 0.0;
    s->sim_time          = 0.0;
    s->mode              = fmi3ModeInstantiated;
    return fmi3OK;
}

/* ═══════════════════════════════════════════════════
   GET / SET — Float64 only (all variables are Float64)
   ═══════════════════════════════════════════════════ */

FMI3_EXPORT fmi3Status fmi3GetFloat64(
    fmi3Instance instance,
    const fmi3ValueReference* vrs, size_t nValueReferences,
    fmi3Float64* values, size_t nValues)
{
    (void)nValues;
    PlantFMU* s = (PlantFMU*)instance;
    for (size_t i = 0; i < nValueReferences; i++) {
        switch (vrs[i]) {
            case VR_THROTTLE_CMD:  values[i] = s->throttle_cmd;      break;
            case VR_BRAKE_CMD:     values[i] = s->brake_cmd;         break;
            case VR_CURRENT_SPEED: values[i] = s->current_speed_kmh; break;
            default:
                log_msg(s, fmi3Warning, "warn", "GetFloat64: unknown VR");
                return fmi3Warning;
        }
    }
    return fmi3OK;
}

FMI3_EXPORT fmi3Status fmi3SetFloat64(
    fmi3Instance instance,
    const fmi3ValueReference* vrs, size_t nValueReferences,
    const fmi3Float64* values, size_t nValues)
{
    (void)nValues;
    PlantFMU* s = (PlantFMU*)instance;
    for (size_t i = 0; i < nValueReferences; i++) {
        switch (vrs[i]) {
            case VR_THROTTLE_CMD: s->throttle_cmd = values[i]; break;
            case VR_BRAKE_CMD:    s->brake_cmd    = values[i]; break;
            default:
                log_msg(s, fmi3Warning, "warn", "SetFloat64: unknown/read-only VR");
                return fmi3Warning;
        }
    }
    return fmi3OK;
}

/* ═══════════════════════════════════════════════════
   REQUIRED STUBS — fmu-check expects these symbols
   ═══════════════════════════════════════════════════ */

FMI3_EXPORT fmi3Status fmi3SetDebugLogging(
    fmi3Instance i, fmi3Boolean l, size_t n, const fmi3String* c)
{ (void)i;(void)l;(void)n;(void)c; return fmi3OK; }

FMI3_EXPORT fmi3Status fmi3EnterConfigurationMode(fmi3Instance i)
{ (void)i; return fmi3OK; }

FMI3_EXPORT fmi3Status fmi3ExitConfigurationMode(fmi3Instance i)
{ (void)i; return fmi3OK; }

FMI3_EXPORT fmi3Status fmi3GetNumberOfVariableDependencies(
    fmi3Instance i, fmi3ValueReference v, size_t* n)
{ (void)i;(void)v; *n=0; return fmi3OK; }

FMI3_EXPORT fmi3Status fmi3GetVariableDependencies(
    fmi3Instance i, fmi3ValueReference vr, size_t* ei,
    fmi3ValueReference* dvr, size_t* di, int* dk, size_t n)
{ (void)i;(void)vr;(void)ei;(void)dvr;(void)di;(void)dk;(void)n; return fmi3OK; }

FMI3_EXPORT fmi3Status fmi3GetFMUState(fmi3Instance i, fmi3FMUState* s)
{ (void)i;(void)s; return fmi3Error; }

FMI3_EXPORT fmi3Status fmi3SetFMUState(fmi3Instance i, fmi3FMUState s)
{ (void)i;(void)s; return fmi3Error; }

FMI3_EXPORT fmi3Status fmi3FreeFMUState(fmi3Instance i, fmi3FMUState* s)
{ (void)i;(void)s; return fmi3Error; }

FMI3_EXPORT fmi3Status fmi3SerializedFMUStateSize(
    fmi3Instance i, fmi3FMUState s, size_t* n)
{ (void)i;(void)s;(void)n; return fmi3Error; }

FMI3_EXPORT fmi3Status fmi3SerializeFMUState(
    fmi3Instance i, fmi3FMUState s, fmi3Byte* b, size_t n)
{ (void)i;(void)s;(void)b;(void)n; return fmi3Error; }

FMI3_EXPORT fmi3Status fmi3DeSerializeFMUState(
    fmi3Instance i, const fmi3Byte* b, size_t n, fmi3FMUState* s)
{ (void)i;(void)b;(void)n;(void)s; return fmi3Error; }

FMI3_EXPORT fmi3Status fmi3GetDirectionalDerivative(
    fmi3Instance i,
    const fmi3ValueReference* ur, size_t nu,
    const fmi3ValueReference* yr, size_t ny,
    const fmi3Float64* dv, size_t nd,
    fmi3Float64* dz, size_t ndz)
{ (void)i;(void)ur;(void)nu;(void)yr;(void)ny;
  (void)dv;(void)nd;(void)dz;(void)ndz; return fmi3Error; }

FMI3_EXPORT fmi3Status fmi3GetAdjointDerivative(
    fmi3Instance i,
    const fmi3ValueReference* ur, size_t nu,
    const fmi3ValueReference* yr, size_t ny,
    const fmi3Float64* dz, size_t ndz,
    fmi3Float64* dv, size_t nd)
{ (void)i;(void)ur;(void)nu;(void)yr;(void)ny;
  (void)dz;(void)ndz;(void)dv;(void)nd; return fmi3Error; }

/* Typed Get/Set stubs — fmu-check requires these symbols to be present */
FMI3_EXPORT fmi3Status fmi3GetFloat32(fmi3Instance i,const fmi3ValueReference*v,size_t n,float*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetFloat32(fmi3Instance i,const fmi3ValueReference*v,size_t n,const float*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetInt8(fmi3Instance i,const fmi3ValueReference*v,size_t n,signed char*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetInt8(fmi3Instance i,const fmi3ValueReference*v,size_t n,const signed char*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetUInt8(fmi3Instance i,const fmi3ValueReference*v,size_t n,unsigned char*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetUInt8(fmi3Instance i,const fmi3ValueReference*v,size_t n,const unsigned char*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetInt16(fmi3Instance i,const fmi3ValueReference*v,size_t n,short*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetInt16(fmi3Instance i,const fmi3ValueReference*v,size_t n,const short*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetUInt16(fmi3Instance i,const fmi3ValueReference*v,size_t n,unsigned short*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetUInt16(fmi3Instance i,const fmi3ValueReference*v,size_t n,const unsigned short*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetInt32(fmi3Instance i,const fmi3ValueReference*v,size_t n,int*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetInt32(fmi3Instance i,const fmi3ValueReference*v,size_t n,const int*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetUInt32(fmi3Instance i,const fmi3ValueReference*v,size_t n,unsigned int*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetUInt32(fmi3Instance i,const fmi3ValueReference*v,size_t n,const unsigned int*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetInt64(fmi3Instance i,const fmi3ValueReference*v,size_t n,long long*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetInt64(fmi3Instance i,const fmi3ValueReference*v,size_t n,const long long*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetUInt64(fmi3Instance i,const fmi3ValueReference*v,size_t n,unsigned long long*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetUInt64(fmi3Instance i,const fmi3ValueReference*v,size_t n,const unsigned long long*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetBoolean(fmi3Instance i,const fmi3ValueReference*v,size_t n,int*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetBoolean(fmi3Instance i,const fmi3ValueReference*v,size_t n,const int*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetString(fmi3Instance i,const fmi3ValueReference*v,size_t n,const char**o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetString(fmi3Instance i,const fmi3ValueReference*v,size_t n,const char*const*o,size_t no){(void)i;(void)v;(void)n;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetBinary(fmi3Instance i,const fmi3ValueReference*v,size_t n,size_t*s,const void**o,size_t no){(void)i;(void)v;(void)n;(void)s;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetBinary(fmi3Instance i,const fmi3ValueReference*v,size_t n,const size_t*s,const void*const*o,size_t no){(void)i;(void)v;(void)n;(void)s;(void)o;(void)no;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetClock(fmi3Instance i,const fmi3ValueReference*v,size_t n,int*o){(void)i;(void)v;(void)n;(void)o;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetClock(fmi3Instance i,const fmi3ValueReference*v,size_t n,const int*o){(void)i;(void)v;(void)n;(void)o;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetIntervalDecimal(fmi3Instance i,const fmi3ValueReference*v,size_t n,fmi3Float64*iv,fmi3Float64*ie,int*iq){(void)i;(void)v;(void)n;(void)iv;(void)ie;(void)iq;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetIntervalDecimal(fmi3Instance i,const fmi3ValueReference*v,size_t n,const fmi3Float64*iv){(void)i;(void)v;(void)n;(void)iv;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetIntervalFraction(fmi3Instance i,const fmi3ValueReference*v,size_t n,fmi3UInt64*c,fmi3UInt64*r,int*q){(void)i;(void)v;(void)n;(void)c;(void)r;(void)q;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetIntervalFraction(fmi3Instance i,const fmi3ValueReference*v,size_t n,const fmi3UInt64*c,const fmi3UInt64*r){(void)i;(void)v;(void)n;(void)c;(void)r;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetShiftDecimal(fmi3Instance i,const fmi3ValueReference*v,size_t n,fmi3Float64*s){(void)i;(void)v;(void)n;(void)s;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetShiftDecimal(fmi3Instance i,const fmi3ValueReference*v,size_t n,const fmi3Float64*s){(void)i;(void)v;(void)n;(void)s;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3GetShiftFraction(fmi3Instance i,const fmi3ValueReference*v,size_t n,fmi3UInt64*c,fmi3UInt64*r){(void)i;(void)v;(void)n;(void)c;(void)r;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3SetShiftFraction(fmi3Instance i,const fmi3ValueReference*v,size_t n,const fmi3UInt64*c,const fmi3UInt64*r){(void)i;(void)v;(void)n;(void)c;(void)r;return fmi3Error;}
FMI3_EXPORT fmi3Status fmi3EvaluateDiscreteStates(fmi3Instance i){(void)i;return fmi3OK;}
FMI3_EXPORT fmi3Status fmi3UpdateDiscreteStates(fmi3Instance i,int*dsc,int*ter,int*nc,int*vch,int*nvch,fmi3Float64*nt){(void)i;if(dsc)*dsc=0;if(ter)*ter=0;if(nc)*nc=0;if(vch)*vch=0;if(nvch)*nvch=0;if(nt)*nt=0;return fmi3OK;}
FMI3_EXPORT fmi3Status fmi3EnterEventMode(fmi3Instance i){(void)i;return fmi3OK;}
FMI3_EXPORT fmi3Status fmi3EnterStepMode(fmi3Instance i){(void)i;return fmi3OK;}
