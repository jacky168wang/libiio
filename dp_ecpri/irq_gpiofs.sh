#!/bin/sh
#
# access hardware GPIOs from user-space rather than kernel-space
# Example1
#
# Copyright (C) 2018~2020 FACC Inc.
# Author: Jacky Wang <kenwj@sina.com>
# Reference:
#   https://developer.ridgerun.com/wiki/index.php/How_to_use_GPIO_signals
#
# License: GPL, version 2.1
#------------------------------------------------------------------------------
# Total Procedure:
#   Reference:
#     http://www.wiki.xilinx.com/Linux+GPIO+Driver
#     http://www.wiki.xilinx.com/GPIO+User+Space+App
#   Kconfig options:
#     CONFIG_GPIO_SYSFS=y
#     CONFIG_SYSFS=y
#     CONFIG_EXPERIMENTAL=y
#     CONFIG_GPIO_XILINX=y
#   Performance on the ML507 reference system:
#     Pretty good: the GPIO can be toggled about every 4 usec
#   Commands to setup the GPIO:
#     console> mount -t sysfs sysfs /sys
#     console> echo 240 > /sys/class/gpio/export
#     console> echo out > /sys/class/gpio/gpio240/direction
#     console> echo 1 > /sys/class/gpio/gpio240/value
#   Bash-script to toggle the gpio for testing
#     while [ 1 ]; do
#       echo 1 > /sys/class/gpio/gpio240/value
#       echo 0 > /sys/class/gpio/gpio240/value
#     done
#------------------------------------------------------------------------------

show_usage()
{
    printf "\ngpio.sh <gpio pin number> [in|out [<value>]]\n"
}

if [ \( $# -eq 0 \) -o \( $# -gt 3 \) ] ; then
    show_usage
    printf "\n\nERROR: incorrect number of parameters\n"
    exit 255
fi

#doesn't hurt to export a gpio more than once
echo $1 > /sys/class/gpio/export

if [  $# -eq 1 ] ; then
   cat /sys/class/gpio/gpio$1/value
   exit 0
fi

if [ \( "$2" != "in" \) -a  \( "$2" != "out" \) ] ; then
    show_usage
    printf "\n\nERROR: second parameter must be 'in' or 'out'\n"
    exit 255
fi

echo $2 > /sys/class/gpio/gpio$1/direction

if [  $# -eq 2 ] ; then
   cat /sys/class/gpio/gpio$1/value
   exit 0
fi


VAL=$3

if [ $VAL -ne 0 ] ; then
    VAL=1
fi

echo $VAL > /sys/class/gpio/gpio$1/value