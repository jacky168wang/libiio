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

#DEFS=" -DBUILDING_RAWS_UDP -DBUILDING_BBU_DL"
#DEFS1="-DBUILDING_RAWS_UDP -DBUILDING_BBU_DL "
#gcc $DEFS  rru_bbu.c rru_raws.c -o ru_bbu_dl.x86 -lpthread
#gcc $DEFS1 rru_bbu.c rru_raws.c -o ru_bbu_dl_dbg.x86 -lpthread
#$CC $DEFS  $LIBS $LIBP rru_bbu.c rru_raws.c -o ru_bbu_dl.arm
#$CC $DEFS1 $LIBS $LIBP rru_bbu.c rru_raws.c -o ru_bbu_dl_dbg.arm
#DEFS=" -DBUILDING_RAWS_UDP -DBUILDING_BBU_UL"
#DEFS1="-DBUILDING_RAWS_UDP -DBUILDING_BBU_UL "
#gcc $DEFS  rru_bbu.c rru_raws.c -o ru_bbu_ul.x86 -lpthread
#gcc $DEFS1 rru_bbu.c rru_raws.c -o ru_bbu_ul_dbg.x86 -lpthread
#$CC $DEFS  $LIBS $LIBP rru_bbu.c rru_raws.c -o ru_bbu_ul.arm
#$CC $DEFS1 $LIBS $LIBP rru_bbu.c rru_raws.c -o ru_bbu_ul_dbg.arm
#DEFS=" -DBUILDING_RAWS_UDP -DBUILDING_BBU_DL -DBUILDING_BBU_UL"
#DEFS1="-DBUILDING_RAWS_UDP -DBUILDING_BBU_DL -DBUILDING_BBU_UL "
#gcc $DEFS  rru_bbu.c rru_raws.c -o ru_bbu.x86 -lpthread
#gcc $DEFS1 rru_bbu.c rru_raws.c -o ru_bbudbg.x86 -lpthread
#$CC DEFS  $LIBS $LIBP rru_bbu.c rru_raws.c -o ru_bbu.arm
#$CC DEFS1 $LIBS $LIBP rru_bbu.c rru_raws.c -o ru_bbu_dbg.arm

