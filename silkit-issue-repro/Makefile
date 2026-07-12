CC := gcc
SILKIT := C:/SilKit/SilKit
INCLUDES := -Isrc -I$(SILKIT)/include
CFLAGS := -O0 -g3 -Wall
LIBS := -LC:/SilKit/SilKit/lib -lSilKit -lwinmm -lws2_32

all: testA.exe testB.exe
testA.exe: src/test.c
	$(CC) $(CFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
testB.exe: src/test2.c
	$(CC) $(CFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
clean:
	rm -f testA.exe testB.exe
