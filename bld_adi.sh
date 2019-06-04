#!/bin/sh
################################################################################
# https://wiki.analog.com/resources/tools-software/linux-software/libiio

host_prebld()
{
    sudo apt-get install libxml2 libxml2-dev bison flex libcdk5-dev cmake
    sudo apt-get install libaio-dev libusb-1.0-0-dev libserialport-dev \
        libxml2-dev libavahi-client-dev doxygen graphviz

    git clone https://github.com/analogdevicesinc/libiio.git
    cd libiio
    cat .gitignore
    #...
}

host_itree_bld()
{
    cmake ./
    #...
    git status
    #.version
    #90-libiio.rules
    #generateDocumentationAndDeploy.sh
    #iio-config.h

    make all
    #...
    git status
    tests/iio_attr
    tests/iio_writedev

    sudo make install
    #...
}

host_itree_cln()
{
    make clean

    git add bld_adi.sh

    #git clean -d -f -x
}

host_otree_bld()
{
    mkdir -p build/ && cd build/

    cmake ..

    make all

    file iiod/iiod

    cd ..
}

host_otree_cln()
{
    cd build

    make clean

    rm -rf *

    cd ..
}

#cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_PATH}/usr/share/buildroot/toolchainfile.cmake .
#make all
#sudo make install DESTDIR=${TOOLCHAIN_PATH}
targ_otree_bld()
{
    mkdir -p build/ && cd build/

    arm_cc

    which arm-linux-gnueabihf-gcc

    CMAKE_TOOLCHAIN_FILE=../cmake/platforms/linux/arm-gnueabi.toolchain.cmake

    cmake ..

    make all

    file iiod/iiod

    arm_rm
}

targ_otree_cln()
{
    cd build

    arm_cc

    make clean

    arm_rm

    rm -rf *

    cd ..
}



host_itree_cln
host_otree_cln
targ_otree_cln
if [ $1=="-h" ];then
    host_otree_bld
else
    targ_otree_bld
fi
