#!/bin/bash

CC=arm-linux-gnueabihf-gcc
#$CC -print-sysroot
#echo $SYSROOT
#$CC--sysroot=$SYSROOT

LIBS="-liio -lusb-1.0 -lserialport -lxml2 -ludev -lpthread"
LIBP="-L${LIBP_TARGET} -Wl,-rpath=${LIBP_TARGET}"
#LIBP='-L${LIBP_TARGET} -L${LIBP_TARGET}/../../lib -Wl,-rpath=$LIBP_TARGET'
#$CC -I.. $LIBS $LIBP $1 -o $2

#---------------- libiio only

#gcc psock_tpacket.c -o psock_tpacket.x86 -lpthread
#$CC -I.. $LIBS $LIBP psock_tpacket.c -o psock_tpacket.arm

#gcc psock_mmap_rx_fanout.c -o psock_mmap_rx_fanout.x86 -lpthread
#$CC -I.. $LIBS $LIBP psock_mmap_rx_fanout.c -o psock_mmap_rx_fanout.arm

#gcc psock_mmap_rx_tpv3.c -o psock_mmap_rx_tpv3.x86 -lpthread
#$CC -I.. $LIBS $LIBP psock_mmap_rx_tpv3.c -o psock_mmap_rx_tpv3.arm

#gcc psock_mmap_tx.c -o psock_mmap_tx.x86 -lpthread -O0 -g
#$CC -I.. $LIBS $LIBP psock_mmap_tx.c -o psock_mmap_tx.arm

#$CC -I.. $LIBS $LIBP -O2 ad9371-iiostream.c -o ad9371org.arm
#$CC -I.. $LIBS $LIBP -O2 ad9371-iiostream-jacky.c rru_bbu.c -o iiojacky.arm

DEFS="-DBUILDING_TMPPS_SOLO -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DTMPPS_TRG_UTIMER" #-Os
$CC -I.. $DEFS $LIBS $LIBP ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c -o rru_tm_tmr.arm

#$CC -I.. $LIBS $LIBP -O2 irq_gpio_sysfs.c -o irq_gpio_sysfs.arm
DEFS="-DBUILDING_TMPPS_SOLO -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DTMPPS_TRG_UGPIOIRQ" #-Os
$CC -I.. $DEFS $LIBS $LIBP ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c irq_gpio_sysfs.c -o rru_tm_irq.arm

$CC -I.. $LIBS $LIBP -O2 ppstest.c -o ppstest.arm
DEFS="-DBUILDING_TMPPS_SOLO -DBUILDING_RRU_UL -DBUILDING_PFRAWS_UDP -DTMPPS_TRG_KSIGIO_NOWAIT" #-Os
$CC -I.. $DEFS $LIBS $LIBP ad9371-iiostream-jacky.c rru_bbu.c psock_rru.c ppstest.c -o rru_tm_pps.arm

