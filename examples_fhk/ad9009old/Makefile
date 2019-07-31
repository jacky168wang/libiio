# Makefile
#
# Copyright (C) 2018 FACC, Inc.
# Author: Jacky Wang <kenwj@sina.com>
#
# License: GPL, version 2.1
#

TARGETS := ad9009init.arm ad9009stream.arm

CFLAGS += -I. -I.. -Italise -Imykonos

# NOTICE: comment the below 2 lines when using libraries produced by buildroot
CC = /opt/arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc
LD_LIBRARY_PATH = ./fhk_arm_libs
LDFLAGS = -L${LD_LIBRARY_PATH} -Wl,-rpath=${LD_LIBRARY_PATH}

LDFLAGS += -liio -lusb-1.0 -lserialport -lxml2 -ludev -lpthread

.PHONY: all clean

all: $(TARGETS)

ad9009init.arm : ad9009init.c
	$(eval DEFS = -DBUILDING_PFRAWS_UDP)
	$(CC) -o $@ $^ $(CFLAGS) $(DEFS) $(LDFLAGS)
	@file ad9009init.arm

ad9009stream.arm : ad9009stream.c
	$(eval DEFS = -DBUILDING_PFRAWS_UDP)
	$(CC) -o $@ $^ $(CFLAGS) $(DEFS) $(LDFLAGS)
	@file ad9009stream.arm
	
clean :
	rm -f $(TARGETS) $(TARGETS:%=%.o)
