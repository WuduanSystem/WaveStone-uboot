#!/bin/bash
make distclean
ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- make  rugo_defconfig
ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- make -j8
