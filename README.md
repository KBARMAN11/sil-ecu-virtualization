# SIL ECU Virtualization — Level 1 vECU to FMU Co-Simulation

> **Personal project.** Built independently in personal time using open-source tools only — no commercial licenses, no proprietary components, no employer IP. Views and work are my own.

A from-scratch, open-source-only SIL (Software-in-the-Loop) ECU virtualization stack built on Windows, progressing through seven stages without ever changing the underlying SWC (Software Component) application logic — only the RTE layer underneath it changes.

```
Level 1  →  Level 2  →  Level 3  →  FMI 3.0    →  FMI 3.0     →  Plant  →  FMU
stub RTE   queue RTE   real CAN    CAN FMU        Ethernet FMU    FMU       Co-Sim
                        (SIL Kit)  FMI-LS-BUS     FMI-LS-BUS               Master
                                   CAN Low-Cut     Eth Low-Cut              80 km/h
                                                   (v1.3-beta)
```

The same `SpeedController`, `BrakeController`, and `ActuatorSWC` source files run unchanged from Level 1 all the way to the FMU co-simulation — only the RTE (Runtime Environment) underneath is swapped out each stage, mirroring how AUTOSAR-style layered architectures keep application logic portable across very different execution and communication environments.

## Why

Most public FMI-LS-BUS examples stop at CAN. This project pushes through to:
- A working **FMI-LS-BUS Ethernet** FMU (v1.3-beta prerelease) validated end-to-end against a real importer
- A **custom C FMI master algorithm** orchestrating two-FMU closed-loop co-simulation — no commercial orchestrator
- **Closed-loop speed convergence to 80 km/h** across two separate FMUs communicating through the master

## Toolchain

| Component | Version / Detail |
|---|---|
| Compiler | GCC (MinGW-w64) on Windows, via Git Bash |
| RTOS | FreeRTOS, WIN32 (simulation) port |
| SIL bus simulation | Vector SIL Kit 5.0.5 (open source) |
| FMI standard | FMI 3.0 Co-Simulation |
| FMI layered standard | FMI-LS-BUS — CAN Low-Cut (stable) and Ethernet Low-Cut (v1.2.0-alpha, part of v1.3-beta prerelease) |
| FMU import/run | Vector SIL Kit FMU Importer 1.7.0 (xPlatform-x64) |
| .NET runtime | .NET 10 Desktop Runtime (required — .NET 8 is not enough for Importer 1.7.0) |
| FMU validation | [fmu-check](https://fmu-check.herokuapp.com) — zero errors on all FMUs |

No commercial licenses used anywhere in this stack.

## Repo layout

```
level1-level2/        Level 1 & 2 — stubbed RTE, then FreeRTOS-queue RTE, multi-SWC
level3-can-silkit/    Level 3 — real virtual CAN bus via Vector SIL Kit
silkit-issue-repro/   Minimal repro for an open SIL Kit GitHub issue (see below)
fmu-can/              FMI 3.0 CAN FMU — FMI-LS-BUS CAN Low-Cut terminals
fmu-ethernet/         FMI 3.0 Ethernet FMU — FMI-LS-BUS Ethernet Low-Cut terminals
fmu-plant/            FMI 3.0 Plant FMU — 1000 kg longitudinal vehicle dynamics (plain Float64 scalars)
fmu-master/           Custom C FMI master algorithm — two-FMU closed-loop co-simulation
```

## Stage details

### Level 1 — stubbed RTE
`SpeedController` SWC running on a FreeRTOS cyclic scheduler, with signals backed by plain global variables — just enough to prove the control loop and scheduling work before any real signal routing exists.

### Level 2 — queue-based RTE, multi-SWC
Adds `BrakeController` and `ActuatorSWC`. The RTE is rebuilt on FreeRTOS queues (`xQueueOverwrite`, "latest value wins" per signal), and `ActuatorSWC` implements brake-overrides-throttle safety arbitration.

### Level 3 — virtual CAN bus via Vector SIL Kit
Same SWC source, new RTE (`Rte_can.c`) backed by a real virtual CAN bus over SIL Kit's C API. A separate `CanMonitor` participant passively decodes all bus traffic for visibility.

| CAN ID | Signal | Direction |
|---|---|---|
| 0x101 | ThrottleCmd | TX |
| 0x102 | BrakeCmd | TX |
| 0x201 | CurrentSpeed | TX |
| 0x202 | TargetSpeed | RX |

### FMI 3.0 CAN FMU
The vECU control logic re-implemented as a standalone FMI 3.0 Co-Simulation FMU, exposing CAN traffic through FMI-LS-BUS CAN Low-Cut binary clocks/variables instead of SIL Kit's native API — making it importable by any FMI-LS-BUS-compliant tool. Validated with zero errors on fmu-check.

### FMI 3.0 Ethernet FMU
Extends the CAN FMU to FMI-LS-BUS Ethernet Low-Cut (v1.2.0-alpha, part of the v1.3-beta prerelease). Each signal is packed as a synthetic Ethernet frame (MAC addresses, EtherType, start-frame delimiter). Validated with zero errors on fmu-check, then run through the SIL Kit FMU Importer 1.7.0, with `SpeedController` converging to 80 km/h entirely over virtual Ethernet frames.

### Plant FMU — vehicle dynamics
Standalone FMI 3.0 Co-Simulation FMU modelling a 1000 kg longitudinal vehicle with aerodynamic drag. Uses plain FMI 3.0 Float64 scalar variables (no bus encoding) — throttle_cmd and brake_cmd as inputs, current_speed as output. Validated with zero errors on fmu-check.

| Parameter | Value |
|---|---|
| Vehicle mass | 1000 kg |
| Step size (dt) | 0.1 s |
| Throttle gain | 30 N per unit |
| Brake gain | 50 N per unit |
| Drag coefficient | 0.3 |
| Open-loop terminal velocity (throttle=80) | ~105 km/h |

### FMU Co-Simulation — custom C master algorithm
A custom C FMI master algorithm (`fmu_master.c`) loads both FMU DLLs at runtime and orchestrates them in a closed loop — demonstrating exactly what every commercial co-simulation tool (dSPACE, Simulink, Modelon Impact) does internally.

Signal flow per step:
```
vECU FMU doStep()
→ get ETH1.Tx_Data (Ethernet frames: ThrottleCmd + BrakeCmd)
→ unpack frames → set plant throttle_cmd, brake_cmd (Float64)
→ Plant FMU doStep()
→ get current_speed (Float64)
→ pack as Ethernet frame → set vECU ETH1.Rx_Data
→ repeat
```

Result: SpeedController closed-loop converges to **80.00 km/h** and holds — throttle stabilising at 44%, the exact feedforward value that balances aerodynamic drag at target speed.

## Building

```bash
# Level 1/2
cd level1-level2 && make && ./vecu.exe

# Level 3 (start SIL Kit registry + system controller first — see Running)
cd level3-can-silkit && make && ./vecu_l3.exe

# CAN FMU
cd fmu-can/fmu
jar cf ../vECU_SpeedController.fmu modelDescription.xml binaries extra terminalsAndIcons

# Ethernet FMU
cd fmu-ethernet
gcc -O0 -g3 -Wall -Iheaders -shared -o binaries/x86_64-windows/vecu_eth_fmu.dll sources/vecu_eth_fmu.c
jar cf ../vECU_Ethernet.fmu modelDescription.xml binaries extra headers terminalsAndIcons.xml documentation

# Plant FMU
cd fmu-plant
make        # builds plant_fmu.dll
make fmu    # packages vECU_Plant.fmu

# FMU master
cd fmu-master
make        # builds fmu_master.exe
```

> **Windows packaging note:** use `jar cf` (any JDK), not PowerShell's `Compress-Archive` — the latter writes backslash-separated paths inside the zip, which fails `fmu-check`.

## Running

**Level 3 (native SIL Kit):**
```bash
# Terminal 1
sil-kit-registry.exe
# Terminal 2
sil-kit-system-controller.exe --connect-uri silkit://localhost:8500
# Terminal 3
./vecu_l3.exe
```

**Ethernet FMU via SIL Kit FMU Importer:**
```bash
# Terminal 1
sil-kit-registry.exe
# Terminal 2
sil-kit-system-controller.exe --connect-uri silkit://localhost:8500
# Terminal 3
FmuImporter.exe -f "vECU_Ethernet.fmu" --silkit-config-file Config.silkit.yaml
```

**FMU co-simulation master (Plant FMU + Ethernet vECU FMU):**
```bash
# Add both DLL folders to PATH first
export PATH=$PATH:/c/Users/Mechatronics/fmu-ethernet/binaries/x86_64-windows
export PATH=$PATH:/c/Users/Mechatronics/fmu-plant/binaries/x86_64-windows

cd fmu-master
./fmu_master.exe
```

Expected output: speed climbs from 0, SpeedController reduces throttle as target approaches, converges and holds at **80.00 km/h**.

## Fixes encountered

### Ethernet FMU

| Symptom | Fix |
|---|---|
| Importer won't run | Install .NET 10 Desktop Runtime — .NET 8 is insufficient for FmuImporter 1.7.0 |
| Importer rejects the FMU | Clock `intervalVariability` must be `"triggered"`, not `"changing"` |
| modelDescription rejected | Add `<DefaultExperiment startTime stopTime stepSize/>` |
| fmu-check fails on Binary variables | Every `Binary` ModelVariable needs an explicit empty `<Start value=""/>` child |
| fmu-check fails on path separators | Package with `jar cf`, not `Compress-Archive` |
| CLI flag rejected | Use `-f`, not `--fmu` |
| System controller flag rejected | Use `--connect-uri`, not `--registry-uri`, on SIL Kit 5.0.5 |
| Coordinated lifecycle ends after 0 steps | Ensure registry + system controller + all participants running simultaneously before `StartLifecycle` |

### Plant FMU

| Symptom | Fix |
|---|---|
| fmu-check: `<Start>` element not expected | Use `start=` as XML attribute on `<Float64>`, not a child element |
| fmu-check: `DefaultExperiment` not expected | Remove it — optional in FMI 3.0, causes schema issues on some validators |
| fmu-check: `initial=calculated` but provides start value | Add `initial="exact"` to continuous output variables |
| fmu-check: one independent variable required | Add `<Float64 name="time" causality="independent" variability="continuous"/>` |

### FMU master

| Symptom | Fix |
|---|---|
| `fmi3Boolean` redefined | Include `fmi3PlatformTypes.h` instead of redefining types manually |
| `fmi3LsBusOperationHeader` has no member `type` | Field is named `opCode` in this header version |
| `FMI3_LS_BUS_ETHERNET_OP_TYPE_TRANSMIT` undeclared | Correct constant is `FMI3_LS_BUS_ETHERNET_OP_TRANSMIT` |
| `BUFFER_INFO_INIT` arrow errors | Pass `&bufInfo` (pointer), not `bufInfo` (value) |
| `CREATE_OP_TRANSMIT` wrong arg count | First arg after `&bufInfo` is `SFD`, then `FragmentCounter` — 9 args total |
| Speed stays 0, throttle stays 0 | `UnpackRxFrames` was writing to `targetSpeed` not `currentSpeed` — fixed |
| Speed converges below target | P-controller deadband too wide; added feedforward offset and hold state at error=0 |

## Known open issue

See [Issue #1](https://github.com/KBARMAN11/sil-ecu-virtualization/issues/1) 
for a minimal repro of a SIL Kit coordinated lifecycle bug on Windows/MinGW —`SimStepHandler` never invoked despite all participants connecting. Also filed on [vectorgrp/sil-kit](https://github.com/vectorgrp/sil-kit). **Status: open.**

## Acknowledgements

Thanks to Christian Bertsch (FMI Project Leader, Bosch Research / Modelica Association) for confirming Direct Communication mode for the Ethernet beta and pointing to FMI-LS-BUS GitHub Discussions for support.

## License

MIT — see [LICENSE](LICENSE).
