CC       := gcc
FREERTOS := C:/Users/Mechatronics/FreeRTOS/FreeRTOS
KERNEL   := $(FREERTOS)/Source
PORT     := $(KERNEL)/portable/MSVC-MingW
SILKIT   := C:/SilKit/SilKit

INCLUDES := \
    -Isrc \
    -I$(KERNEL)/include \
    -I$(PORT) \
    -I$(SILKIT)/include

CFLAGS   := -O0 -g3 -Wall -D_WIN32_WINNT=0x0601

SRCS := \
    src/main.c \
    src/SpeedController.c \
    src/BrakeController.c \
    src/ActuatorSWC.c \
    src/Rte_can.c \
    src/CanMonitor.c \
    $(KERNEL)/tasks.c \
    $(KERNEL)/queue.c \
    $(KERNEL)/list.c \
    $(KERNEL)/timers.c \
    $(KERNEL)/event_groups.c \
    $(KERNEL)/stream_buffer.c \
    $(PORT)/port.c \
    $(KERNEL)/portable/MemMang/heap_4.c

TARGET   := vecu_l3.exe
LIBS     := -LC:/SilKit/SilKit/lib -lSilKit -lwinmm -lws2_32

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)

clean:
	rm -f $(TARGET)

run: $(TARGET)
	./$(TARGET)
