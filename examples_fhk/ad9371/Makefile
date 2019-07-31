# Makefile
#
# Copyright (C) 2018 FACC, Inc.
# Author: Jacky Wang <kenwj@sina.com>
#
# License: GPL, version 2.1
#

TARGETS := ad9371_jacky.arm
#TARGETS += ad9371stream_junyi.arm
TARGETS += ad9371cp_zhiqiang.arm

#CFLAGS += -Wall
CFLAGS += -I. -I..

# NOTICE: comment the below 2 lines when using libraries produced by buildroot
LD_LIBRARY_PATH = ./arm_cc_libs
LDFLAGS = -L${LD_LIBRARY_PATH} -Wl,-rpath=${LD_LIBRARY_PATH}

LDFLAGS += -liio -lusb-1.0 -lserialport -lxml2 -ludev -lpthread

.PHONY: all clean

all: $(TARGETS)

ad9371_jacky.arm : rru_bbu.c ad9371_jacky.c
	$(eval DEFS = -DBUILDING_PFRAWS_UDP)
	#@echo $(DEFS)
	$(CC) -o $@ $^ $(CFLAGS) $(DEFS) $(LDFLAGS)
	@file ad9371_jacky.arm

ad9371cp_zhiqiang.arm : ad9371cp_zhiqiang.c
	$(eval DEFS = -DBUILDING_PFRAWS_UDP)
	#@echo $(DEFS)
	$(CC) -o $@ $^ $(CFLAGS) $(DEFS) $(LDFLAGS)
	@file ad9371cp_zhiqiang.arm


clean :
	rm -f $(TARGETS) $(TARGETS:%=%.o)
