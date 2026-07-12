CC      := gcc
FMI3    := C:/Users/Mechatronics/fmi3/headers
LSBUS   := C:/Users/Mechatronics/fmi3/lsbus

INCLUDES := -Isrc -I$(FMI3) -I$(LSBUS)
CFLAGS   := -O0 -g3 -Wall

TARGET   := fmu/binaries/x86_64-windows/vECU_SpeedController.dll

all: $(TARGET)

$(TARGET): src/vECU_SpeedController.c
	$(CC) $(CFLAGS) $(INCLUDES) -shared -o $@ $^

clean:
	rm -f $(TARGET)
