#!/bin/bash

CC=arm-linux-gnueabihf-gcc
#$CC -print-sysroot
#echo $SYSROOT
#$CC--sysroot=$SYSROOT

LIBS="-liio -lusb-1.0 -lserialport -lxml2 -ludev -lpthread"
LIBP="-L${LIBP_TARGET} -Wl,-rpath=${LIBP_TARGET}"
#LIBP='-L${LIBP_TARGET} -L${LIBP_TARGET}/../../lib -Wl,-rpath=$LIBP_TARGET'
#$CC -I.. $LIBS $LIBP $1 -o $2

# ./tools/testing/selftests/net/psock_tpacket.c
gcc psock_tpacket.c -o psock_tpacket.x86 -lpthread -O0 -g
$CC $LIBS $LIBP psock_tpacket.c -o psock_tpacket.arm

gcc inet_rawsmmap_tx.c -o inet_rawsmmap_tx.x86 -lpthread -O0 -g
$CC $LIBS $LIBP inet_rawsmmap_tx.c -o inet_rawsmmap_tx.arm
DEFS="-DBUILDING_RRU_UL"
$CC -I.. $DEFS $LIBS $LIBP inet_rawsmmap_tx.c ad9371-iiostream-jacky.c rru_bbu.c -o inet_rawsmmap_tx_iio.arm

gcc inet_rawsmmap_rx_tpv3.c -o inet_rawsmmap_rx_tpv3.x86 -lpthread -O0 -g
$CC $LIBS $LIBP inet_rawsmmap_rx_tpv3.c -o inet_rawsmmap_rx_tpv3.arm
DEFS="-DBUILDING_RRU_DL"
$CC -I.. $DEFS $LIBS $LIBP inet_rawsmmap_rx_tpv3.c ad9371-iiostream-jacky.c rru_bbu.c -o inet_rawsmmap_rx_tpv3_iio.arm

#gcc inet_rawsmmap_rx_fanout.c -o inet_rawsmmap_rx_fanout.x86 -lpthread -O0 -g
#$CC $LIBS $LIBP inet_rawsmmap_rx_fanout.c -o inet_rawsmmap_rx_fanout.arm

#---------------- libiio only

$CC -I.. $LIBS $LIBP ad9371-iiostream.c -o ad9371org.arm

DEFS=" -DBUILDING_ORGINAL_ADI"
$CC -I.. $DEFS $LIBS $LIBP ad9371-iiostream-jacky.c -o iioadi.arm

DEFS=" -DBUILDING_SIMILAR_ADI "
$CC -I.. $DEFS  $LIBS $LIBP ad9371-iiostream-jacky.c -o iiojacky.arm

