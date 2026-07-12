CC      := gcc
FREERTOS := $(HOME)/FreeRTOS/FreeRTOS
KERNEL  := $(FREERTOS)/Source
PORT    := $(KERNEL)/portable/MSVC-MingW

INCLUDES := -Isrc -I$(KERNEL)/include -I$(PORT)
CFLAGS   := -O0 -g3 -Wall -D_WIN32_WINNT=0x0601

SRCS := \
    src/main.c \
    src/SpeedController.c \
    src/BrakeController.c \
    src/ActuatorSWC.c \
    src/Rte_queue.c \
    $(KERNEL)/tasks.c \
    $(KERNEL)/queue.c \
    $(KERNEL)/list.c \
    $(KERNEL)/timers.c \
    $(KERNEL)/event_groups.c \
    $(KERNEL)/stream_buffer.c \
    $(PORT)/port.c \
    $(KERNEL)/portable/MemMang/heap_4.c

TARGET := vecu.exe

all: $(TARGET)
$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -o $@ -lwinmm
clean:
	rm -f $(TARGET)
run: $(TARGET)
	./$(TARGET)
