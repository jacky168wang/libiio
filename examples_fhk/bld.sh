#!/bin/sh
#
#   preprocess/compile/assemble/link with cross-compiler
#
# Copyright (C) 2018 FACC Inc.
# Author: Jacky Wang <kenwj@sina.com>
#
# License: GPL, version 2.1
#

#== Clean
if [ "$1" == "clean" ];then
	rm -f *.arm
	exit 0
fi

#== Public Flags for PreCompile/Compile/Assemble/Link
CFLAGS="$CFLAGS -I.. -I. -I./mykonos -I./talise"
#CFLAGS="$CFLAGS -Wall"
LD_LIBRARY_PATH=../fhk_arm_libs
#LD_LIBRARY_PATH=~/arm-linux-gnueabihf
LDFLAGS="-L${LD_LIBRARY_PATH} -Wl,-rpath=${LD_LIBRARY_PATH}"
LDFLAGS="$LDFLAGS -liio"
LDFLAGS="$LDFLAGS -ldbus-1"
#LDFLAGS="$LDFLAGS -lusb"
#LDFLAGS="$LDFLAGS -lusb-1.0"
#LDFLAGS="$LDFLAGS -lserialport"
#LDFLAGS="$LDFLAGS -lxml2"
#LDFLAGS="$LDFLAGS -ludev"
LDFLAGS="$LDFLAGS -lpthread"

#-- workable files on both platforms (for evaluation at the first stage)
#gcc $DEFS $CFLAGS $LDFLAGS $1 -o $1.x86
#[ $? -ne 0 ] && exit 1 || file $1.x86
#$CC $DEFS $CFLAGS $LDFLAGS $1 -o $1.arm
#[ $? -ne 0 ] && exit 1 || file $1.arm

#-- workable files on CC platform only (for evaluation and testing)
DEFS="-DDEBUG"
$CC $DEFS $CFLAGS $LDFLAGS $1 -o $1.arm
[ $? -ne 0 ] && exit 1 || file $1.arm

exit 0
