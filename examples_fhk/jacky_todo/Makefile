# Makefile
#
# Copyright (C) 2018 FACC, Inc.
# Author: Jacky Wang <kenwj@sina.com>
#
# License: GPL, version 2.1
#

TARGETS := iio_phycfg.arm iio_stream.arm

CFLAGS += -I. -I.. -Italise -Imykonos

# NOTICE: comment the below 2 lines when using libraries produced by buildroot
CC = /opt/arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc
LD_LIBRARY_PATH = ./fhk_arm_libs
LDFLAGS = -L${LD_LIBRARY_PATH} -Wl,-rpath=${LD_LIBRARY_PATH}

LDFLAGS += -liio -lusb-1.0 -lserialport -lxml2 -ludev -lpthread

.PHONY: all clean

all: $(TARGETS)

phy_init.arm : phy_common.c phy_init.c ad9371profile20mhz.c ad9009profile100mhz.c
	$(eval DEFS = -DBUILDING_PFRAWS_UDP)
	$(CC) -o $@ $^ $(CFLAGS) $(DEFS) $(LDFLAGS)
	@file phy_init.arm

phy_stream.arm : phy_common.c rru_bbu.c phy_stream.c
	$(eval DEFS = -DBUILDING_PFRAWS_UDP)
	$(CC) -o $@ $^ $(CFLAGS) $(DEFS) $(LDFLAGS)
	@file phy_stream.arm
	
clean :
	rm -f $(TARGETS) $(TARGETS:%=%.o)
