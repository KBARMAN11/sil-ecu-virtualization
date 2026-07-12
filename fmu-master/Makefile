# fmu_master Makefile
# Builds the FMI co-simulation master algorithm
#
# Usage (Git Bash on Windows):
#   make        — compile fmu_master.exe
#   make clean  — remove build artefacts
#
# Requires:
#   headers/    — FMI-LS-BUS headers (copy from fmu-ethernet/headers/)
#   vecu_eth_fmu.dll and plant_fmu.dll on PATH or in same folder at runtime

CC     = gcc
CFLAGS = -O0 -g3 -Wall -Wextra -Iheaders
SRC    = fmu_master.c
EXE    = fmu_master.exe

.PHONY: all clean

all: $(EXE)

$(EXE): $(SRC)
	$(CC) $(CFLAGS) -o $@ $<
	@echo "[OK] Built: $(EXE)"
	@echo ""
	@echo "Run with:"
	@echo "  ./fmu_master.exe"
	@echo ""
	@echo "Make sure both DLLs are reachable:"
	@echo "  fmu-ethernet/binaries/x86_64-windows/vecu_eth_fmu.dll"
	@echo "  fmu-plant/binaries/x86_64-windows/plant_fmu.dll"

clean:
	rm -f $(EXE)
