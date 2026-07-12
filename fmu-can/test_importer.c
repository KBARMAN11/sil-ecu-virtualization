#include <windows.h>
#include <stdio.h>
#include "fmi3Functions.h"

typedef fmi3Instance (*InstantiateFn)(
    fmi3String, fmi3String, fmi3String,
    fmi3Boolean, fmi3Boolean, fmi3Boolean, fmi3Boolean,
    const fmi3ValueReference*, size_t,
    fmi3InstanceEnvironment,
    fmi3LogMessageCallback,
    fmi3IntermediateUpdateCallback);

typedef fmi3Status (*EnterInitFn)(
    fmi3Instance, fmi3Boolean, fmi3Float64,
    fmi3Float64, fmi3Boolean, fmi3Float64);

typedef fmi3Status (*ExitInitFn)(fmi3Instance);

typedef fmi3Status (*DoStepFn)(
    fmi3Instance, fmi3Float64, fmi3Float64, fmi3Boolean,
    fmi3Boolean*, fmi3Boolean*, fmi3Boolean*, fmi3Float64*);

typedef fmi3Status (*TerminateFn)(fmi3Instance);
typedef void       (*FreeInstanceFn)(fmi3Instance);

/* Correct callback signature */
void logMessage(fmi3InstanceEnvironment env,
                fmi3Status status,
                fmi3String category,
                fmi3String message)
{
    printf("[LOG] %s: %s\n", category, message);
}

int main(void) {
    printf("=== FMI 3.0 Importer Test ===\n");

    HMODULE dll = LoadLibraryA(
        "C:\\Users\\Mechatronics\\fmu-can\\fmu\\binaries\\"
        "x86_64-windows\\vECU_SpeedController.dll");
    if (!dll) {
        printf("ERROR: Failed to load DLL: %lu\n", GetLastError());
        return 1;
    }
    printf("[OK] DLL loaded\n");

    InstantiateFn  instantiate = (InstantiateFn) GetProcAddress(dll, "fmi3InstantiateCoSimulation");
    EnterInitFn    enterInit   = (EnterInitFn)   GetProcAddress(dll, "fmi3EnterInitializationMode");
    ExitInitFn     exitInit    = (ExitInitFn)    GetProcAddress(dll, "fmi3ExitInitializationMode");
    DoStepFn       doStep      = (DoStepFn)      GetProcAddress(dll, "fmi3DoStep");
    TerminateFn    terminate   = (TerminateFn)   GetProcAddress(dll, "fmi3Terminate");
    FreeInstanceFn freeInst    = (FreeInstanceFn)GetProcAddress(dll, "fmi3FreeInstance");

    if (!instantiate || !doStep || !enterInit || !exitInit) {
        printf("ERROR: Missing FMI functions\n");
        return 1;
    }
    printf("[OK] All FMI functions found\n");

    fmi3Instance inst = instantiate(
        "vECU", "{1234-5678-vecu-speed}", "",
        0, 0, 0, 0,
        NULL, 0,
        NULL, logMessage, NULL);

    if (!inst) {
        printf("ERROR: Instantiation failed\n");
        return 1;
    }
    printf("[OK] FMU instantiated\n");

    enterInit(inst, 0, 0.0, 0.0, 0, 0.0);
    exitInit(inst);
    printf("[OK] Initialization complete\n");
    printf("\n--- Simulation (20 steps x 100ms) ---\n");

    for (int i = 0; i < 20; i++) {
        fmi3Boolean eventNeeded = 0, terminate_sim = 0, earlyReturn = 0;
        fmi3Float64 lastTime = 0;
        doStep(inst, i * 0.1, 0.1, 0,
               &eventNeeded, &terminate_sim,
               &earlyReturn, &lastTime);
        if (terminate_sim) break;
    }

    terminate(inst);
    freeInst(inst);
    FreeLibrary(dll);
    printf("\n[OK] Simulation complete — FMU works!\n");
    return 0;
}
