# Build for QNX Neutrino with qcc.
#   x86_64 VM  :  make                      (default)
#   aarch64    :  make TARGET=gcc_ntoaarch64le
# Run "source ~/qnx800/qnxsdp-env.sh" (or the qnx710 equivalent) first so qcc is on PATH.
TARGET ?= gcc_ntox86_64
CC      = qcc -V$(TARGET)
CFLAGS  = -Wall -Wextra -O2 -std=gnu11
LDLIBS  = -lm

BINS = bin/sensor_proc bin/acc_ctrl bin/actuator_proc bin/safety_monitor \
       bin/hmi_proc bin/diag_proc bin/vehicle_sim bin/cpu_hog

all: $(BINS)

bin:
	mkdir -p bin

bin/%: src/%.c src/acc_util.c src/acc_common.h | bin
	$(CC) $(CFLAGS) -Isrc -o $@ $< src/acc_util.c $(LDLIBS)

clean:
	rm -rf bin logs

.PHONY: all clean
