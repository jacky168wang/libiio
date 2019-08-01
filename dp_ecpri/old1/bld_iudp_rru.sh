#!/bin/bash

CC=arm-linux-gnueabihf-gcc
#$CC -print-sysroot
#echo $SYSROOT
#$CC--sysroot=$SYSROOT

LIBS="-liio -lusb-1.0 -lserialport -lxml2 -ludev -lpthread"
LIBP="-L${LIBP_TARGET} -Wl,-rpath=${LIBP_TARGET}"
#LIBP='-L${LIBP_TARGET} -L${LIBP_TARGET}/../../lib -Wl,-rpath=$LIBP_TARGET'

#$CC -I.. $LIBS $LIBP $1 -o $2

#ENDIAN_MODE_BE # in case the target system uses big_endian byte order

DEFS=" -DBUILDING_IP_UDP -DBUILDING_RRU_DL"
DEFS1="-DBUILDING_IP_UDP -DBUILDING_RRU_DL "
#gcc $DEFS  rru_bbu.c rru_udps.c -o iu_rru_dl.x86 -lpthread
#gcc $DEFS1 rru_bbu.c rru_udps.c -o iu_rru_dl_dbg.x86 -lpthread
#$CC $DEFS  $LIBS $LIBP rru_bbu.c rru_udps.c -o iu_rru_dl.arm
#$CC $DEFS1 $LIBS $LIBP rru_bbu.c rru_udps.c -o iu_rru_dl_dbg.arm
DEFS=" -DBUILDING_IP_UDP -DBUILDING_RRU_UL"
DEFS1="-DBUILDING_IP_UDP -DBUILDING_RRU_UL "
#gcc $DEFS  rru_bbu.c rru_udps.c -o iu_rru_ul.x86 -lpthread
#gcc $DEFS1 rru_bbu.c rru_udps.c -o iu_rru_ul_dbg.x86 -lpthread
$CC $DEFS  $LIBS $LIBP rru_bbu.c rru_udps.c -o iu_rru_ul.arm
$CC $DEFS1 $LIBS $LIBP rru_bbu.c rru_udps.c -o iu_rru_ul_dbg.arm
DEFS=" -DBUILDING_IP_UDP -DBUILDING_RRU_DL -DBUILDING_RRU_UL"
DEFS1="-DBUILDING_IP_UDP -DBUILDING_RRU_DL -DBUILDING_RRU_UL "
#gcc $DEFS  rru_bbu.c rru_udps.c -o iu_rru.x86 -lpthread
#gcc $DEFS1 rru_bbu.c rru_udps.c -o iu_rrudbg.x86 -lpthread
#$CC $DEFS  $LIBS $LIBP rru_bbu.c rru_udps.c -o iu_rru.arm
#$CC $DEFS1 $LIBS $LIBP rru_bbu.c rru_udps.c -o iu_rru_dbg.arm

#BUILDING_RRU_PUSCH_B2B # board-to-board circle-pusch for testing

DEFS=" -DBUILDING_IP_UDP -DBUILDING_RRU_DL "
DEFS1="-DBUILDING_IP_UDP -DBUILDING_RRU_DL  "
$CC $DEFS  -I.. $LIBS $LIBP rru_bbu.c rru_udps.c ad9371-iiostream-jacky.c -o iu_irru_dl.arm
$CC $DEFS1 -I.. $LIBS $LIBP rru_bbu.c rru_udps.c ad9371-iiostream-jacky.c -o iu_irru_dl_dbg.arm
DEFS=" -DBUILDING_IP_UDP -DBUILDING_RRU_UL "
DEFS1="-DBUILDING_IP_UDP -DBUILDING_RRU_UL  "
$CC $DEFS  -I.. $LIBS $LIBP rru_bbu.c rru_udps.c ad9371-iiostream-jacky.c -o iu_irru_ul.arm
$CC $DEFS1 -I.. $LIBS $LIBP rru_bbu.c rru_udps.c ad9371-iiostream-jacky.c -o iu_irru_ul_dbg.arm
DEFS=" -DBUILDING_IP_UDP -DBUILDING_RRU_DL  -DBUILDING_RRU_UL "
DEFS1="-DBUILDING_IP_UDP -DBUILDING_RRU_DL  -DBUILDING_RRU_UL  "
$CC $DEFS  -I.. $LIBS $LIBP rru_bbu.c rru_udps.c ad9371-iiostream-jacky.c -o iu_irru.arm
$CC $DEFS1 -I.. $LIBS $LIBP rru_bbu.c rru_udps.c ad9371-iiostream-jacky.c -o iu_irru_dbg.arm

#$CC irqgpio_sysfs.c -o gpio-int.arm
