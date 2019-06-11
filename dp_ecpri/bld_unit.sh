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
	#rm -f rruue_ul_tmr.arm rruue_ul_irq.arm rruue_ul_pps.arm
	rm -f rruue_tmr.arm rruue_irq.arm rruue_pps.arm
	#rm -f rrubs_ul_tmr.arm rrubs_ul_irq.arm rrubs_ul_pps.arm
	#rm -f rrubs_tmr.arm rrubs_irq.arm rrubs_pps.arm
	exit 0
fi

#== Public Flags for PreCompile/Compile/Assemble/Link
CFLAGS="$CFLAGS -I. -I../.."
#CFLAGS="$CFLAGS -Wall"
#LD_LIBRARY_PATH=../arm_cc_libs
#LDFLAGS="-L${LD_LIBRARY_PATH} -Wl,-rpath=${LD_LIBRARY_PATH}"
LDFLAGS="$LDFLAGS -liio -lusb-1.0 -lserialport -lxml2 -ludev -lpthread"

#== workable files on CC platform only for single-process/individual-processes

#----------------

#DEF="-DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_UE_RRU -DTMPPS_TRG_UTIMER -O0" #-Os
#$CC $DEF $CFLAGS $LDFLAGS psock_mmap_tx.c ad9371_jacky.c rru_bbu.c psock_rru.c -o rruue_ul_tmr.arm
#[ $? -ne 0 ] && exit 1 || file rruue_ul_tmr.arm
DEF="-DBUILDING_RRU_DL -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_UE_RRU -DTMPPS_TRG_UTIMER -O0" #-Os
$CC $DEF $CFLAGS $LDFLAGS psock_mmap_tx.c psock_mmap_rx_tpv3.c ad9371_jacky.c rru_bbu.c psock_rru.c -o rruue_tmr.arm
[ $? -ne 0 ] && exit 1 || file rruue_tmr.arm
##DEF="-DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_BS_RRU -DTMPPS_TRG_UTIMER -O0" #-Os
##$CC $DEF $CFLAGS $LDFLAGS psock_mmap_tx.c ad9371_jacky.c rru_bbu.c psock_rru.c -o rrubs_ul_tmr.arm
##[ $? -ne 0 ] && exit 1 || file rrubs_ul_tmr.arm
#DEF="-DBUILDING_RRU_DL -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_BS_RRU -DTMPPS_TRG_UTIMER -O0" #-Os
#$CC $DEF $CFLAGS $LDFLAGS psock_mmap_tx.c psock_mmap_rx_tpv3.c ad9371_jacky.c rru_bbu.c psock_rru.c -o rrubs_tmr.arm
#[ $? -ne 0 ] && exit 1 || file rrubs_tmr.arm

#----------------

#DEF="-DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_UE_RRU -DTMPPS_TRG_UGPIOIRQ -O0" #-Os
#$CC $DEF $CFLAGS $LDFLAGS psock_mmap_tx.c ad9371_jacky.c rru_bbu.c psock_rru.c irq_gpiofs.c -o rruue_ul_irq.arm
#[ $? -ne 0 ] && exit 1 || file rruue_ul_irq.arm
DEF="-DBUILDING_RRU_DL -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_UE_RRU -DTMPPS_TRG_UGPIOIRQ -O0" #-Os
$CC $DEF $CFLAGS $LDFLAGS psock_mmap_tx.c psock_mmap_rx_tpv3.c ad9371_jacky.c rru_bbu.c psock_rru.c irq_gpiofs.c -o rruue_irq.arm
[ $? -ne 0 ] && exit 1 || file rruue_irq.arm
##DEF="-DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_BS_RRU -DTMPPS_TRG_UGPIOIRQ -O0" #-Os
##$CC $DEF $CFLAGS $LDFLAGS psock_mmap_tx.c ad9371_jacky.c rru_bbu.c psock_rru.c irq_gpiofs.c -o rrubs_ul_irq.arm
##[ $? -ne 0 ] && exit 1 || file rrubs_ul_irq.arm
#DEF="-DBUILDING_RRU_DL -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_BS_RRU -DTMPPS_TRG_UGPIOIRQ -O0" #-Os
#$CC $DEF $CFLAGS $LDFLAGS psock_mmap_tx.c psock_mmap_rx_tpv3.c ad9371_jacky.c rru_bbu.c psock_rru.c irq_gpiofs.c -o rrubs_irq.arm
#[ $? -ne 0 ] && exit 1 || file rrubs_irq.arm

#----------------

#DEF="-DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_UE_RRU -DTMPPS_TRG_KSIGIO_NOWAIT -O0" #-Os
#$CC $DEF $CFLAGS $LDFLAGS psock_mmap_tx.c ad9371_jacky.c rru_bbu.c psock_rru.c ppstest.c -o rruue_ul_pps.arm
#[ $? -ne 0 ] && exit 1 || file rruue_ul_pps.arm
DEF="-DBUILDING_RRU_DL -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_UE_RRU -DTMPPS_TRG_KSIGIO_NOWAIT -O0" #-Os
$CC $DEF $CFLAGS $LDFLAGS psock_mmap_tx.c psock_mmap_rx_tpv3.c ad9371_jacky.c rru_bbu.c psock_rru.c ppstest.c -o rruue_pps.arm
[ $? -ne 0 ] && exit 1 || file rruue_pps.arm
##DEF="-DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_BS_RRU -DTMPPS_TRG_KSIGIO_NOWAIT -O0" #-Os
##$CC $DEF $CFLAGS $LDFLAGS psock_mmap_tx.c ad9371_jacky.c rru_bbu.c psock_rru.c ppstest.c -o rrubs_ul_pps.arm
##[ $? -ne 0 ] && exit 1 || file rrubs_ul_pps.arm
#DEF="-DBUILDING_RRU_DL -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_BS_RRU -DTMPPS_TRG_KSIGIO_NOWAIT -O0" #-Os
#$CC $DEF $CFLAGS $LDFLAGS psock_mmap_tx.c psock_mmap_rx_tpv3.c ad9371_jacky.c rru_bbu.c psock_rru.c ppstest.c -o rrubs_pps.arm
#[ $? -ne 0 ] && exit 1 || file rrubs_pps.arm

exit 0
