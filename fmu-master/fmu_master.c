/*
 * fmu_master.c
 * ============================================================
 * Custom FMI Co-Simulation Master Algorithm
 *
 * Loads two FMU DLLs and drives them in a closed-loop:
 *   FMU A: vecu_eth_fmu.dll  — vECU (SpeedController + BrakeController)
 *                               communicates via FMI-LS-BUS Ethernet binary
 *   FMU B: plant_fmu.dll     — Vehicle plant (1000kg longitudinal dynamics)
 *                               communicates via plain FMI 3.0 Float64 scalars
 *
 * SIGNAL FLOW per step:
 *   vECU doStep()
 *   → get ETH1.Tx_Data  (Ethernet frames: ThrottleCmd + BrakeCmd)
 *   → unpack frames     → set plant throttle_cmd, brake_cmd (Float64)
 *   → plant doStep()
 *   → get current_speed (Float64)
 *   → pack as Ethernet frame → set vECU ETH1.Rx_Data
 *   repeat
 *
 * BUILD:
 *   gcc -O0 -g3 -Wall -Iheaders -o fmu_master.exe fmu_master.c
 *
 * RUN (add both DLL folders to PATH first):
 *   export PATH=$PATH:/c/Users/Mechatronics/fmu-ethernet/binaries/x86_64-windows
 *   export PATH=$PATH:/c/Users/Mechatronics/fmu-plant/binaries/x86_64-windows
 *   ./fmu_master.exe
 * ============================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <windows.h>

/* ── FMI-LS-BUS headers (from fmu-ethernet/headers/) ── */
#include "fmi3PlatformTypes.h"   /* fmi3Boolean, fmi3Binary, fmi3True/False */
#include "fmi3LsBus.h"           /* fmi3LsBusOperationHeader, opCode field  */
#include "fmi3LsBusEthernet.h"   /* Ethernet op structures, EtherType defs  */
#include "fmi3LsBusUtil.h"       /* fmi3LsBusUtilBufferInfo, BUFFER_INFO_INIT, BUFFER_LENGTH */
#include "fmi3LsBusUtilEthernet.h" /* FMI3_LS_BUS_ETHERNET_CREATE_OP_TRANSMIT */

/* ────────────────────────────────────────────────────────────
   SECTION 1: ADDITIONAL FMI TYPES NOT IN PLATFORM HEADERS
   ──────────────────────────────────────────────────────────── */
typedef void*           fmi3Instance;
typedef unsigned int    fmi3ValueReference;
typedef double          fmi3Float64;
typedef const char*     fmi3String;
typedef size_t          fmi3BinarySize;
typedef unsigned long long fmi3UInt64;

typedef enum {
    fmi3OK      = 0,
    fmi3Warning = 1,
    fmi3Discard = 2,
    fmi3Error   = 3,
    fmi3Fatal   = 4
} fmi3Status;

/* Callback types */
typedef void (*fmi3LogMessageCallback)(
    fmi3Instance, fmi3String, fmi3Status, fmi3String, fmi3String);
typedef void (*fmi3ClockUpdateCallback)(fmi3Instance);
typedef void (*fmi3LockPreemptionCallback)(void);
typedef void (*fmi3UnlockPreemptionCallback)(void);
typedef void (*fmi3IntermediateUpdateCallback)(
    fmi3Instance, fmi3Float64, fmi3Boolean, fmi3Boolean,
    fmi3Boolean, fmi3Boolean, fmi3Boolean,
    fmi3Boolean*, fmi3Float64*);

/* ────────────────────────────────────────────────────────────
   SECTION 2: FMI FUNCTION POINTER TYPES
   ──────────────────────────────────────────────────────────── */
typedef fmi3Instance (*fmi3InstantiateCoSimulation_t)(
    fmi3String, fmi3String, fmi3String,
    fmi3Boolean, fmi3Boolean, fmi3Boolean, fmi3Boolean,
    const fmi3ValueReference*, size_t, fmi3Instance,
    fmi3LogMessageCallback, fmi3ClockUpdateCallback,
    fmi3LockPreemptionCallback, fmi3UnlockPreemptionCallback,
    fmi3IntermediateUpdateCallback);
typedef void       (*fmi3FreeInstance_t)(fmi3Instance);
typedef fmi3Status (*fmi3EnterInitializationMode_t)(
    fmi3Instance, fmi3Boolean, fmi3Float64, fmi3Float64,
    fmi3Boolean, fmi3Float64);
typedef fmi3Status (*fmi3ExitInitializationMode_t)(fmi3Instance);
typedef fmi3Status (*fmi3DoStep_t)(
    fmi3Instance, fmi3Float64, fmi3Float64, fmi3Boolean,
    fmi3Boolean*, fmi3Boolean*, fmi3Boolean*, fmi3Float64*);
typedef fmi3Status (*fmi3Terminate_t)(fmi3Instance);
typedef fmi3Status (*fmi3GetFloat64_t)(
    fmi3Instance, const fmi3ValueReference*, size_t,
    fmi3Float64*, size_t);
typedef fmi3Status (*fmi3SetFloat64_t)(
    fmi3Instance, const fmi3ValueReference*, size_t,
    const fmi3Float64*, size_t);
typedef fmi3Status (*fmi3GetBinary_t)(
    fmi3Instance, const fmi3ValueReference*, size_t,
    size_t*, const void**, size_t);
typedef fmi3Status (*fmi3SetBinary_t)(
    fmi3Instance, const fmi3ValueReference*, size_t,
    const size_t*, const void* const*, size_t);
typedef fmi3Status (*fmi3SetClock_t)(
    fmi3Instance, const fmi3ValueReference*, size_t,
    const fmi3Boolean*);

/* ────────────────────────────────────────────────────────────
   SECTION 3: FMU HANDLE STRUCT
   ──────────────────────────────────────────────────────────── */
typedef struct {
    HMODULE                         dll;
    fmi3Instance                    inst;
    fmi3InstantiateCoSimulation_t   instantiate;
    fmi3FreeInstance_t              freeInstance;
    fmi3EnterInitializationMode_t   enterInit;
    fmi3ExitInitializationMode_t    exitInit;
    fmi3DoStep_t                    doStep;
    fmi3Terminate_t                 terminate;
    fmi3GetFloat64_t                getFloat64;
    fmi3SetFloat64_t                setFloat64;
    fmi3GetBinary_t                 getBinary;
    fmi3SetBinary_t                 setBinary;
    fmi3SetClock_t                  setClock;
} FmuHandle;

/* ────────────────────────────────────────────────────────────
   SECTION 4: VALUE REFERENCES (from modelDescription.xml)

   Plant FMU:
     VR 0 = throttle_cmd   (input,  Float64)
     VR 1 = brake_cmd      (input,  Float64)
     VR 2 = current_speed  (output, Float64)

   Ethernet vECU FMU:
     VR 0 = time            (independent, Float64)
     VR 1 = ETH1.Tx_Clock   (output, Clock)
     VR 2 = ETH1.Tx_Data    (output, Binary)
     VR 3 = ETH1.Rx_Clock   (input,  Clock)
     VR 4 = ETH1.Rx_Data    (input,  Binary)
   ──────────────────────────────────────────────────────────── */
#define PLANT_VR_THROTTLE   0u
#define PLANT_VR_BRAKE      1u
#define PLANT_VR_SPEED      2u

#define VECU_VR_TX_CLOCK    1u
#define VECU_VR_TX_DATA     2u
#define VECU_VR_RX_CLOCK    3u
#define VECU_VR_RX_DATA     4u

/* ────────────────────────────────────────────────────────────
   SECTION 5: ETHERNET FRAME CONSTANTS
   Must match vecu_eth_fmu.c exactly
   ──────────────────────────────────────────────────────────── */
static const fmi3LsBusEthernetMacAddressOctet SRC_MAC[6] =
    {0xAA, 0xBB, 0xCC, 0x00, 0x00, 0x02};
static const fmi3LsBusEthernetMacAddressOctet DST_MAC[6] =
    {0xAA, 0xBB, 0xCC, 0x00, 0x00, 0x01};

#define ETH_TYPE_THROTTLE  ((fmi3LsBusEthernetTypeOrLength)0x8801)
#define ETH_TYPE_BRAKE     ((fmi3LsBusEthernetTypeOrLength)0x8802)
#define ETH_TYPE_SPEED     ((fmi3LsBusEthernetTypeOrLength)0x8803)
#define ETH_SFD            ((fmi3LsBusEthernetStartDelimiter)0xD5)

/* ────────────────────────────────────────────────────────────
   SECTION 6: SIMULATION PARAMETERS
   ──────────────────────────────────────────────────────────── */
#define SIM_START    0.0
#define SIM_STOP   120.0
#define SIM_STEP     0.1
#define TARGET_SPEED 80.0

/* ────────────────────────────────────────────────────────────
   SECTION 7: HELPERS
   ──────────────────────────────────────────────────────────── */
static void logger(fmi3Instance inst, fmi3String name,
                   fmi3Status status, fmi3String cat, fmi3String msg)
{
    (void)inst;
    if (status >= fmi3Warning)
        printf("  [%s][%s] %s\n", name, cat, msg);
}

static int load_fmu(FmuHandle* h, const char* dll_path)
{
    h->dll = LoadLibraryA(dll_path);
    if (!h->dll) {
        printf("ERROR: Cannot load %s  (error %lu)\n", dll_path, GetLastError());
        return 0;
    }
    h->instantiate  = (fmi3InstantiateCoSimulation_t)
                      GetProcAddress(h->dll, "fmi3InstantiateCoSimulation");
    h->freeInstance = (fmi3FreeInstance_t)
                      GetProcAddress(h->dll, "fmi3FreeInstance");
    h->enterInit    = (fmi3EnterInitializationMode_t)
                      GetProcAddress(h->dll, "fmi3EnterInitializationMode");
    h->exitInit     = (fmi3ExitInitializationMode_t)
                      GetProcAddress(h->dll, "fmi3ExitInitializationMode");
    h->doStep       = (fmi3DoStep_t)
                      GetProcAddress(h->dll, "fmi3DoStep");
    h->terminate    = (fmi3Terminate_t)
                      GetProcAddress(h->dll, "fmi3Terminate");
    h->getFloat64   = (fmi3GetFloat64_t)
                      GetProcAddress(h->dll, "fmi3GetFloat64");
    h->setFloat64   = (fmi3SetFloat64_t)
                      GetProcAddress(h->dll, "fmi3SetFloat64");
    h->getBinary    = (fmi3GetBinary_t)
                      GetProcAddress(h->dll, "fmi3GetBinary");
    h->setBinary    = (fmi3SetBinary_t)
                      GetProcAddress(h->dll, "fmi3SetBinary");
    h->setClock     = (fmi3SetClock_t)
                      GetProcAddress(h->dll, "fmi3SetClock");

    if (!h->instantiate || !h->freeInstance || !h->enterInit ||
        !h->exitInit    || !h->doStep       || !h->terminate ||
        !h->getFloat64  || !h->setFloat64) {
        printf("ERROR: Missing required FMI symbols in %s\n", dll_path);
        return 0;
    }
    printf("  [OK] %s\n", dll_path);
    return 1;
}

/* ────────────────────────────────────────────────────────────
   SECTION 8: ETHERNET FRAME PACK / UNPACK

   unpack_vecu_tx():
     Walk the vECU's Tx binary buffer.
     Each operation starts with fmi3LsBusOperationHeader.
     Check opCode == FMI3_LS_BUS_ETHERNET_OP_TRANSMIT,
     then check typeOrLength (EtherType) to identify the signal.

   pack_speed_to_rx():
     Pack current_speed (Float64) into an Ethernet frame
     using EtherType 0x8803, big-endian uint16 payload.
     This is what the vECU expects on its Rx_Data input.
   ──────────────────────────────────────────────────────────── */
static void unpack_vecu_tx(const uint8_t* buf, size_t len,
                            double* throttle_out, double* brake_out)
{
    const uint8_t* ptr = buf;
    const uint8_t* end = buf + len;

    while (ptr + sizeof(fmi3LsBusOperationHeader) <= end) {
        const fmi3LsBusOperationHeader* hdr =
            (const fmi3LsBusOperationHeader*)ptr;

        /* opCode (not 'type') — field name from fmi3LsBus.h */
        if (hdr->opCode == FMI3_LS_BUS_ETHERNET_OP_TRANSMIT) {
            const fmi3LsBusEthernetOperationTransmit* tx =
                (const fmi3LsBusEthernetOperationTransmit*)ptr;

            if (tx->typeOrLength == ETH_TYPE_THROTTLE && tx->dataLength >= 1)
                *throttle_out = (double)tx->data[0];

            if (tx->typeOrLength == ETH_TYPE_BRAKE && tx->dataLength >= 1)
                *brake_out = (double)tx->data[0];
        }
        if (hdr->length == 0) break;
        ptr += hdr->length;
    }
}

#define RX_BUF_SIZE 512
static uint8_t g_rx_buf[RX_BUF_SIZE];

static size_t pack_speed_to_rx(double speed_kmh)
{
    /* Declare bufInfo as a local variable, pass its address to macros */
    fmi3LsBusUtilBufferInfo bufInfo;
    FMI3_LS_BUS_BUFFER_INFO_INIT(&bufInfo, g_rx_buf, RX_BUF_SIZE);

    uint16_t speed_u16 = (uint16_t)(speed_kmh);   /* raw km/h integer, matches vECU fmi3UInt16 */
    uint8_t sd[2] = {
        (uint8_t)(speed_u16 >> 8),
        (uint8_t)(speed_u16 & 0xFF)
    };

    /* 9 args: &bufInfo, SFD, FragmentCounter, LastFragment,
               DstMAC, SrcMAC, EtherType, DataLength, Data */
    FMI3_LS_BUS_ETHERNET_CREATE_OP_TRANSMIT(&bufInfo, ETH_SFD,
        (fmi3LsBusEthernetMPacketFragmentCounter)0, FMI3_LS_BUS_TRUE,
        DST_MAC, SRC_MAC, ETH_TYPE_SPEED, 2, sd);

    return (size_t)FMI3_LS_BUS_BUFFER_LENGTH(&bufInfo);
}

/* ════════════════════════════════════════════════════════════
   MAIN
   ════════════════════════════════════════════════════════════ */
int main(void)
{
    printf("==========================================================\n");
    printf("  FMU Co-Simulation Master\n");
    printf("  vECU (Ethernet) + Vehicle Plant (Float64)\n");
    printf("  Target: %.0f km/h   Stop: %.0f s   dt: %.1f s\n",
           TARGET_SPEED, SIM_STOP, SIM_STEP);
    printf("==========================================================\n\n");

    /* ── 1. Load DLLs ── */
    printf("[1] Loading FMU DLLs...\n");
    FmuHandle vecu  = {0};
    FmuHandle plant = {0};

    if (!load_fmu(&vecu,
        "C:\\Users\\Mechatronics\\fmu-ethernet\\binaries\\x86_64-windows\\vecu_eth_fmu.dll"))
        return 1;
    if (!load_fmu(&plant,
        "C:\\Users\\Mechatronics\\fmu-plant\\binaries\\x86_64-windows\\plant_fmu.dll"))
        return 1;
    printf("\n");

    /* ── 2. Instantiate ── */
    printf("[2] Instantiating...\n");
    vecu.inst = vecu.instantiate(
        "vECU",
        "{22222222-3333-4444-5555-666666666666}",
        NULL, fmi3False, fmi3False, fmi3False, fmi3False,
        NULL, 0, NULL, logger, NULL, NULL, NULL, NULL);
    if (!vecu.inst) { printf("ERROR: vECU instantiate failed\n"); return 1; }
    printf("  [OK] vECU\n");

    plant.inst = plant.instantiate(
        "Plant",
        "{11111111-2222-3333-4444-555555555555}",
        NULL, fmi3False, fmi3False, fmi3False, fmi3False,
        NULL, 0, NULL, logger, NULL, NULL, NULL, NULL);
    if (!plant.inst) { printf("ERROR: Plant instantiate failed\n"); return 1; }
    printf("  [OK] Plant\n\n");

    /* ── 3. Initialization ── */
    printf("[3] Initialization...\n");
    plant.enterInit(plant.inst, fmi3False, 0.0, SIM_START, fmi3True, SIM_STOP);
    plant.exitInit(plant.inst);
    printf("  [OK] Plant\n");

    vecu.enterInit(vecu.inst, fmi3False, 0.0, SIM_START, fmi3True, SIM_STOP);

    /* Send initial speed = 0 to vECU before it exits init */
    fmi3ValueReference rx_vr  = VECU_VR_RX_DATA;
    fmi3ValueReference rxc_vr = VECU_VR_RX_CLOCK;
    size_t      rx_len = pack_speed_to_rx(0.0);
    const void* rx_ptr = g_rx_buf;
    fmi3Boolean clk_tick = fmi3True;
    if (vecu.setClock)  vecu.setClock(vecu.inst,  &rxc_vr, 1, &clk_tick);
    if (vecu.setBinary) vecu.setBinary(vecu.inst, &rx_vr,  1, &rx_len, &rx_ptr, 1);
    vecu.exitInit(vecu.inst);
    printf("  [OK] vECU\n\n");

    /* ── 4. Co-simulation loop ── */
    printf("[4] Running...\n");
    printf("%-8s  %-12s  %-12s  %-12s\n",
           "Time(s)", "Throttle", "Brake", "Speed(km/h)");
    printf("--------------------------------------------------\n");

    double t        = SIM_START;
    double throttle = 0.0;
    double brake    = 0.0;
    double speed    = 0.0;
    int    steps    = 0;

    fmi3ValueReference tx_vr = VECU_VR_TX_DATA;
    fmi3ValueReference p_thr = PLANT_VR_THROTTLE;
    fmi3ValueReference p_brk = PLANT_VR_BRAKE;
    fmi3ValueReference p_spd = PLANT_VR_SPEED;
    const void* tx_ptr = NULL;
    size_t      tx_len = 0;

    while (t < SIM_STOP - 1e-9) {

        /* 4a. Step vECU */
        fmi3Boolean evt=0, term=0, early=0;
        fmi3Float64 last_t = 0.0;
        fmi3Status st = vecu.doStep(vecu.inst, t, SIM_STEP, fmi3False,
                                    &evt, &term, &early, &last_t);
        if (st > fmi3Warning) {
            printf("ERROR: vECU doStep failed at t=%.2f\n", t);
            break;
        }

        /* 4b. Unpack vECU Tx_Data → ThrottleCmd, BrakeCmd */
        throttle = 0.0; brake = 0.0;
        if (vecu.getBinary) {
            vecu.getBinary(vecu.inst, &tx_vr, 1, &tx_len, &tx_ptr, 1);
            if (tx_ptr && tx_len > 0)
                unpack_vecu_tx((const uint8_t*)tx_ptr, tx_len,
                               &throttle, &brake);
        }

        /* 4c. Set plant inputs */
        plant.setFloat64(plant.inst, &p_thr, 1, &throttle, 1);
        plant.setFloat64(plant.inst, &p_brk, 1, &brake,    1);

        /* 4d. Step plant */
        plant.doStep(plant.inst, t, SIM_STEP, fmi3False,
                     &evt, &term, &early, &last_t);

        /* 4e. Get plant speed */
        plant.getFloat64(plant.inst, &p_spd, 1, &speed, 1);

        /* 4f. Pack speed → vECU Rx_Data */
        rx_len = pack_speed_to_rx(speed);
        rx_ptr = g_rx_buf;
        if (vecu.setClock)
            vecu.setClock(vecu.inst, &rxc_vr, 1, &clk_tick);
        if (vecu.setBinary)
            vecu.setBinary(vecu.inst, &rx_vr, 1, &rx_len, &rx_ptr, 1);

        /* Print every 1 second */
        if (steps % 10 == 0)
            printf("%-8.1f  %-12.1f  %-12.1f  %-12.2f\n",
                   t, throttle, brake, speed);
        steps++;

        if (term) break;
        t += SIM_STEP;
    }

    /* ── 5. Shutdown ── */
    printf("\n[5] Shutdown\n");
    vecu.terminate(vecu.inst);
    plant.terminate(plant.inst);
    vecu.freeInstance(vecu.inst);
    plant.freeInstance(plant.inst);
    FreeLibrary(vecu.dll);
    FreeLibrary(plant.dll);

    printf("\n==========================================================\n");
    printf("  Final speed: %.2f km/h  (target: %.0f km/h)\n",
           speed, TARGET_SPEED);
    printf("==========================================================\n");
    return 0;
}
