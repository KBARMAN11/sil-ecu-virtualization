# Plant FMU — Makefile
# Builds plant_fmu.dll and packages vECU_Plant.fmu
#
# Usage (Git Bash on Windows):
#   make          — compile DLL
#   make fmu      — compile + package FMU
#   make clean    — remove build artefacts

CC      = gcc
CFLAGS  = -O0 -g3 -Wall -Wextra
SRC     = sources/plant_fmu.c
DLL     = binaries/x86_64-windows/plant_fmu.dll
FMU     = ../vECU_Plant.fmu

.PHONY: all fmu clean

all: $(DLL)

$(DLL): $(SRC)
	$(CC) $(CFLAGS) -shared -o $@ $< -lm
	@echo "[OK] DLL built: $(DLL)"

fmu: $(DLL)
	jar cf $(FMU) modelDescription.xml binaries sources documentation
	@echo "[OK] FMU packaged: $(FMU)"
	@echo "     Upload to fmu-check.herokuapp.com to validate"

clean:
	rm -f $(DLL) $(FMU)
