#!/bin/bash

CC=arm-linux-gnueabihf-gcc
#$CC -print-sysroot
#echo $SYSROOT
#$CC--sysroot=$SYSROOT

LIBS="-liio -lusb-1.0 -lserialport -lxml2 -ludev -lpthread"
LIBP="-L${LIBP_TARGET} -Wl,-rpath=${LIBP_TARGET}"
#LIBP='-L${LIBP_TARGET} -L${LIBP_TARGET}/../../lib -Wl,-rpath=$LIBP_TARGET'
#$CC -I.. $LIBS $LIBP $1 -o $2

#----------------

DEFS="-DBUILDING_RRU_DL -DBUILDING_PFRAWS_UDP -O0" #-Os
#$CC -I.. $DEFS $LIBS $LIBP psock_mmap_rx_tpv3.c ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c -o rruue_dl.arm
DEFS="-DBUILDING_RRU_DL -DBUILDING_PFRAWS_UDP  -DBUILDING_BS_RRU -O0" #-Os
#$CC -I.. $DEFS $LIBS $LIBP psock_mmap_rx_tpv3.c ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c -o rrubs_dl.arm

DEFS="-DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_UE_RRU -DTMPPS_TRG_UTIMER -O0" #-Os
$CC -I.. $DEFS $LIBS $LIBP psock_mmap_tx.c ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c -o rruue_ul_tmr.arm
DEFS="-DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_BS_RRU -DTMPPS_TRG_UTIMER -O0" #-Os
$CC -I.. $DEFS $LIBS $LIBP psock_mmap_tx.c ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c -o rrubs_ul_tmr.arm
DEFS="-DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_UE_RRU -DTMPPS_TRG_UGPIOIRQ -O0" #-Os
$CC -I.. $DEFS $LIBS $LIBP psock_mmap_tx.c ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c irq_gpio_sysfs.c -o rruue_ul_irq.arm
DEFS="-DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_BS_RRU -DTMPPS_TRG_UGPIOIRQ -O0" #-Os
$CC -I.. $DEFS $LIBS $LIBP psock_mmap_tx.c ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c irq_gpio_sysfs.c -o rrubs_ul_irq.arm
DEFS="-DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_UE_RRU -DTMPPS_TRG_KSIGIO_NOWAIT -O0" #-Os
$CC -I.. $DEFS $LIBS $LIBP psock_mmap_tx.c ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c ppstest.c -o rruue_ul_pps.arm
DEFS="-DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_BS_RRU -DTMPPS_TRG_KSIGIO_NOWAIT -O0" #-Os
$CC -I.. $DEFS $LIBS $LIBP psock_mmap_tx.c ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c ppstest.c -o rrubs_ul_pps.arm

DEFS="-DBUILDING_RRU_DL -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_UE_RRU -DTMPPS_TRG_UTIMER -O0" #-Os
$CC -I.. $DEFS $LIBS $LIBP psock_mmap_tx.c psock_mmap_rx_tpv3.c ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c -o rruue_tmr.arm
DEFS="-DBUILDING_RRU_DL -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_BS_RRU -DTMPPS_TRG_UTIMER -O0" #-Os
$CC -I.. $DEFS $LIBS $LIBP psock_mmap_tx.c psock_mmap_rx_tpv3.c ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c -o rrubs_tmr.arm
DEFS="-DBUILDING_RRU_DL -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_UE_RRU -DTMPPS_TRG_UGPIOIRQ -O0" #-Os
$CC -I.. $DEFS $LIBS $LIBP psock_mmap_tx.c psock_mmap_rx_tpv3.c ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c irq_gpio_sysfs.c -o rruue_irq.arm
DEFS="-DBUILDING_RRU_DL -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_BS_RRU -DTMPPS_TRG_UGPIOIRQ -O0" #-Os
$CC -I.. $DEFS $LIBS $LIBP psock_mmap_tx.c psock_mmap_rx_tpv3.c ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c irq_gpio_sysfs.c -o rrubs_irq.arm
DEFS="-DBUILDING_RRU_DL -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_UE_RRU -DTMPPS_TRG_KSIGIO_NOWAIT -O0" #-Os
$CC -I.. $DEFS $LIBS $LIBP psock_mmap_tx.c psock_mmap_rx_tpv3.c ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c ppstest.c -o rruue_pps.arm
DEFS="-DBUILDING_RRU_DL -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DBUILDING_BS_RRU -DTMPPS_TRG_KSIGIO_NOWAIT -O0" #-Os
$CC -I.. $DEFS $LIBS $LIBP psock_mmap_tx.c psock_mmap_rx_tpv3.c ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c ppstest.c -o rrubs_pps.arm

