#!/bin/bash

start_time=$(date +%s)

OUT_PATH=~/android/kernel/out

echo "Cleaning build directory..."
rm -rf $OUT_PATH/KERNEL_OBJ
rm -rf $OUT_PATH/system
mkdir $OUT_PATH/KERNEL_OBJ
mkdir $OUT_PATH/system

EXIT=0

THREADS=4
COMPILER=~/android/arm-cortex_a9-linux-gnueabihf-linaro_4.7.4-2014.01/bin/arm-cortex_a9-linux-gnueabihf-

KERNEL_PATH=~/android/kernel/android_kernel_acer_t30
DEFCONFIG=cyanogenmod_picasso_mf_defconfig

make -j$THREADS -C $KERNEL_PATH O=$OUT_PATH/KERNEL_OBJ ARCH=arm CROSS_COMPILE=$COMPILER $DEFCONFIG || EXIT=$?
make -j$THREADS -C $KERNEL_PATH O=$OUT_PATH/KERNEL_OBJ ARCH=arm CROSS_COMPILE=$COMPILER headers_install || EXIT=$?
make -j$THREADS -C $KERNEL_PATH O=$OUT_PATH/KERNEL_OBJ ARCH=arm CROSS_COMPILE=$COMPILER zImage || EXIT=$?
make -j$THREADS -C $KERNEL_PATH O=$OUT_PATH/KERNEL_OBJ ARCH=arm CROSS_COMPILE=$COMPILER modules || EXIT=$?
make -j$THREADS -C $KERNEL_PATH O=$OUT_PATH/KERNEL_OBJ INSTALL_MOD_PATH=../system ARCH=arm CROSS_COMPILE=COMPILER modules_install || EXIT=$?

echo Exit flag: $EXIT
if [ $EXIT -ge 1 ]; then
   exit;
fi

finish_time=$(date +%s)
echo "Time taken: $((finish_time - start_time)) seconds."


# make flashable zip

BOOTIMG_TOOLS=~/android/bootimg_tools

rm $BOOTIMG_TOOLS/boot/new-ramdisk.cpio.gz
rm $BOOTIMG_TOOLS/boot/zImage
rm -rf $BOOTIMG_TOOLS/boot/ramdisk/*
cp -rf ~/android/kernel/a700_ramdisk/* $BOOTIMG_TOOLS/boot/ramdisk/

# repack ramdisk
$BOOTIMG_TOOLS/repack_ramdisk $BOOTIMG_TOOLS/boot/ramdisk

# copy zImage
cp -f $OUT_PATH/KERNEL_OBJ/arch/arm/boot/zImage $BOOTIMG_TOOLS/boot/

# make boot.img
$BOOTIMG_TOOLS/mkbootimg --kernel $BOOTIMG_TOOLS/boot/zImage --ramdisk $BOOTIMG_TOOLS/boot/new-ramdisk.cpio.gz -o $BOOTIMG_TOOLS/boot.img

cp -f $BOOTIMG_TOOLS/boot.img ~/android/kernel/a700_kernel_work/ramdiskkernel/

cp -f $OUT_PATH/system/lib/modules/3.1.10-z-r1+/kernel/fs/cifs/cifs.ko ~/android/kernel/a700_kernel_work/ramdiskkernel/system/lib/modules/

rm ~/android/kernel/a700_kernel_work/z-kernel.zip

cd ~/android/kernel/a700_kernel_work/ramdiskkernel
zip -r9 ~/android/kernel/a700_kernel_work/z-kernel.zip *

echo "Flashable zip ready at ~/android/kernel/a700_kernel_work/z-kernel.zip"

exit $EXIT
