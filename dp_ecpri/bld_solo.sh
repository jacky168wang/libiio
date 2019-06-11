#!/bin/sh
#
#   preprocess/compile/assemble/link with cross-compiler
#
# Copyright (C) 2018~2020 FACC Inc.
# Author: Jacky Wang <kenwj@sina.com>
#
# License: GPL, version 2.1
#

#== Clean
if [ "$1" == "clean" ];then
	#rm -f ad9371org.arm
	rm -f ad9371jacky.arm 
	rm -f rru_tm_tmr.arm rru_tm_irq.arm rru_tm_pps.arm
	exit 0
fi

#== Public Flags for PreCompile/Compile/Assemble/Link
CFLAGS="$CFLAGS -I. -I../.."
#CFLAGS="$CFLAGS -Wall"
#LD_LIBRARY_PATH=../arm_cc_libs
#LDFLAGS="-L${LD_LIBRARY_PATH} -Wl,-rpath=${LD_LIBRARY_PATH}"
LDFLAGS="$LDFLAGS -liio -lusb-1.0 -lserialport -lxml2 -ludev -lpthread"

#-- workable files on both platforms (for evaluation at the first stage)
#gcc psock_tpacket.c -o psock_tpacket.x86 -lpthread
#$CC $CFLAGS $LDFLAGS psock_tpacket.c -o psock_tpacket.arm

#gcc psock_mmap_rx_fanout.c -o psock_mmap_rx_fanout.x86 -lpthread
#$CC $CFLAGS $LDFLAGS psock_mmap_rx_fanout.c -o psock_mmap_rx_fanout.arm

#gcc psock_mmap_rx_tpv3.c -o psock_mmap_rx_tpv3.x86 -lpthread
#$CC $CFLAGS $LDFLAGS psock_mmap_rx_tpv3.c -o psock_mmap_rx_tpv3.arm

#gcc psock_mmap_tx.c -o psock_mmap_tx.x86 -lpthread -O0 -g
#$CC $CFLAGS $LDFLAGS psock_mmap_tx.c -o psock_mmap_tx.arm

#-- workable files on CC platform only (for evaluation and testing)

#----------------

#$CC $CFLAGS $LDFLAGS -O2 ad9371-iiostream.c -o ad9371org.arm
#[ $? -ne 0 ] && exit 1 || file ad9371org.arm
#$CC $CFLAGS $LDFLAGS -O2 ad9371_jacky.c rru_bbu.c -o ad9371jacky.arm
#[ $? -ne 0 ] && exit 1 || file ad9371jacky.arm

#----------------

DEF="-DBUILDING_TMPPS_SOLO -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DTMPPS_TRG_UTIMER" #-Os
$CC $DEF $CFLAGS $LDFLAGS ad9371_jacky.c rru_bbu.c psock_rru.c -o rru_tm_tmr.arm
[ $? -ne 0 ] && exit 1 || file rru_tm_tmr.arm

#----------------

#$CC $CFLAGS $LDFLAGS -O2 irq_gpiofs.c -o irq_gpiofs.arm
DEF="-DBUILDING_TMPPS_SOLO -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DTMPPS_TRG_UGPIOIRQ" #-Os
$CC $DEF $CFLAGS $LDFLAGS ad9371_jacky.c rru_bbu.c psock_rru.c irq_gpiofs.c -o rru_tm_irq.arm
[ $? -ne 0 ] && exit 1 || file rru_tm_irq.arm

#----------------

#$CC $CFLAGS $LDFLAGS -O2 ppstest.c -o ppstest.arm
DEF="-DBUILDING_TMPPS_SOLO -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DTMPPS_TRG_KSIGIO_NOWAIT" #-Os
$CC $DEF $CFLAGS $LDFLAGS ad9371_jacky.c rru_bbu.c psock_rru.c ppstest.c -o rru_tm_pps.arm
[ $? -ne 0 ] && exit 1 || file rru_tm_pps.arm

exit 0
