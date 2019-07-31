# libiio - Library for interfacing industrial I/O (IIO) devices
#
# Copyright (C) 2014 Analog Devices, Inc.
# Author: Paul Cercueil <paul.cercueil@analog.com>
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.


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
