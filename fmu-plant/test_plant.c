/*
 * test_plant.c
 *
 * Standalone test importer for the Plant FMU.
 * Loads plant_fmu.dll, drives 30 seconds of simulation
 * with a fixed ThrottleCmd=80, BrakeCmd=0 and prints
 * speed each step — expect convergence near 80 km/h.
 *
 * Build (Git Bash):
 *   gcc -O0 -Wall -o test_plant.exe test_plant.c
 *
 * Run:
 *   ./test_plant.exe
 */

#include <stdio.h>
#include <stdlib.h>
#include <windows.h>

/* ── Minimal FMI 3.0 types ── */
typedef void*        fmi3Instance;
typedef unsigned int fmi3ValueReference;
typedef int          fmi3Boolean;
typedef double       fmi3Float64;
typedef const char*  fmi3String;
#define fmi3True  1
#define fmi3False 0
typedef enum { fmi3OK=0, fmi3Warning=1, fmi3Error=3 } fmi3Status;

typedef void (*fmi3LogMessageCallback)(
    fmi3Instance, fmi3String, fmi3Status, fmi3String, fmi3String);
typedef void (*fmi3ClockUpdateCallback)(fmi3Instance);
typedef void (*fmi3LockPreemptionCallback)(void);
typedef void (*fmi3UnlockPreemptionCallback)(void);
typedef void (*fmi3IntermediateUpdateCallback)(
    fmi3Instance, fmi3Float64, fmi3Boolean, fmi3Boolean,
    fmi3Boolean, fmi3Boolean, fmi3Boolean,
    fmi3Boolean*, fmi3Float64*);

/* ── Function pointer types ── */
typedef fmi3Instance (*fmi3InstantiateCoSimulation_t)(
    fmi3String, fmi3String, fmi3String,
    fmi3Boolean, fmi3Boolean, fmi3Boolean, fmi3Boolean,
    const fmi3ValueReference*, size_t,
    fmi3Instance,
    fmi3LogMessageCallback, fmi3ClockUpdateCallback,
    fmi3LockPreemptionCallback, fmi3UnlockPreemptionCallback,
    fmi3IntermediateUpdateCallback);

typedef void          (*fmi3FreeInstance_t)(fmi3Instance);
typedef fmi3Status    (*fmi3EnterInitializationMode_t)(
    fmi3Instance, fmi3Boolean, fmi3Float64,
    fmi3Float64, fmi3Boolean, fmi3Float64);
typedef fmi3Status    (*fmi3ExitInitializationMode_t)(fmi3Instance);
typedef fmi3Status    (*fmi3DoStep_t)(
    fmi3Instance, fmi3Float64, fmi3Float64, fmi3Boolean,
    fmi3Boolean*, fmi3Boolean*, fmi3Boolean*, fmi3Float64*);
typedef fmi3Status    (*fmi3Terminate_t)(fmi3Instance);
typedef fmi3Status    (*fmi3GetFloat64_t)(
    fmi3Instance, const fmi3ValueReference*, size_t,
    fmi3Float64*, size_t);
typedef fmi3Status    (*fmi3SetFloat64_t)(
    fmi3Instance, const fmi3ValueReference*, size_t,
    const fmi3Float64*, size_t);

/* ── Value references (must match modelDescription.xml) ── */
#define VR_THROTTLE_CMD  0u
#define VR_BRAKE_CMD     1u
#define VR_CURRENT_SPEED 2u

static void logger(fmi3Instance inst, fmi3String name,
                   fmi3Status status, fmi3String cat, fmi3String msg)
{
    (void)inst; (void)status;
    printf("[%s][%s] %s\n", name, cat, msg);
}

int main(void)
{
    printf("=== Plant FMU Test Importer ===\n\n");

    /* ── Load DLL ── */
    HMODULE dll = LoadLibraryA(
        "C:\\Users\\Mechatronics\\fmu-plant\\binaries\\"
        "x86_64-windows\\plant_fmu.dll");
    if (!dll) {
        printf("ERROR: Failed to load DLL (error %lu)\n", GetLastError());
        printf("       Build with: make\n");
        return 1;
    }
    printf("[OK] DLL loaded\n");

    /* ── Resolve symbols ── */
#define LOAD(name) \
    name##_t name = (name##_t)GetProcAddress(dll, #name); \
    if (!name) { printf("ERROR: missing " #name "\n"); return 1; }

    LOAD(fmi3InstantiateCoSimulation)
    LOAD(fmi3FreeInstance)
    LOAD(fmi3EnterInitializationMode)
    LOAD(fmi3ExitInitializationMode)
    LOAD(fmi3DoStep)
    LOAD(fmi3Terminate)
    LOAD(fmi3GetFloat64)
    LOAD(fmi3SetFloat64)
    printf("[OK] All symbols resolved\n\n");

    /* ── Instantiate ── */
    fmi3Instance inst = fmi3InstantiateCoSimulation(
        "PlantTest",
        "{11111111-2222-3333-4444-555555555555}",
        NULL,
        fmi3False, fmi3False, fmi3False, fmi3False,
        NULL, 0, NULL,
        logger, NULL, NULL, NULL, NULL);
    if (!inst) { printf("ERROR: InstantiateCoSimulation failed\n"); return 1; }
    printf("[OK] FMU instantiated\n");

    /* ── Initialization ── */
    fmi3EnterInitializationMode(inst, fmi3False, 0.0, 0.0, fmi3True, 30.0);
    fmi3ExitInitializationMode(inst);
    printf("[OK] Initialization complete\n\n");
    printf("%-8s  %-14s  %-14s  %-14s\n",
           "Time(s)", "ThrottleCmd", "BrakeCmd", "Speed(km/h)");
    printf("------------------------------------------------------\n");

    /* ── Simulation loop ── */
    double t = 0.0;
    double dt = 0.1;
    double stop = 30.0;

    fmi3ValueReference vr_throttle = VR_THROTTLE_CMD;
    fmi3ValueReference vr_brake    = VR_BRAKE_CMD;
    fmi3ValueReference vr_speed    = VR_CURRENT_SPEED;

    /* Fixed inputs: full throttle, no brake */
    double throttle_val = 80.0;
    double brake_val    = 0.0;

    while (t < stop - 1e-9) {
        /* Set inputs */
        fmi3SetFloat64(inst, &vr_throttle, 1, &throttle_val, 1);
        fmi3SetFloat64(inst, &vr_brake,    1, &brake_val,    1);

        /* Step */
        fmi3Boolean evtNeeded=0, terminate=0, earlyRet=0;
        fmi3Float64 lastTime = 0.0;
        fmi3DoStep(inst, t, dt, fmi3False,
                   &evtNeeded, &terminate, &earlyRet, &lastTime);

        /* Read output */
        double speed = 0.0;
        fmi3GetFloat64(inst, &vr_speed, 1, &speed, 1);

        if (((int)(t * 10 + 0.5)) % 10 == 0)   /* print every 1s */
            printf("%-8.1f  %-14.1f  %-14.1f  %-14.2f\n",
                   t, throttle_val, brake_val, speed);

        if (terminate) { printf("[DONE] FMU requested terminate\n"); break; }
        t += dt;
    }

    printf("\n[OK] Simulation complete — speed should be near 80 km/h\n");

    /* ── Shutdown ── */
    fmi3Terminate(inst);
    fmi3FreeInstance(inst);
    FreeLibrary(dll);
    printf("[OK] FMU freed\n");
    return 0;
}
