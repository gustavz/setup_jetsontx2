#!/bin/bash

#Break execution on any error received
set -e

#Locally suppress stderr to avoid raising not relevant messages
exec 3>&2
exec 2> /dev/null
con_dev=$(ls /dev/video* | wc -l)
exec 2>&3

if [ $con_dev -ne 0 ];
then
	echo -e "\e[32m"
	read -p "Remove all RealSense cameras attached. Hit any key when ready"
	echo -e "\e[0m"
fi

#Include usability functions
source ./scripts/patch-utils.sh

# Get the required tools and headers to build the kernel
#sudo apt-get install linux-headers-generic build-essential git
sudo apt-get install build-essential git

#Additional packages to build patch
kernel_name="kernel-4.4"
require_package libssl-dev

[ ! -d ${kernel_name} ] && git clone https://github.com/jetsonhacks/buildJetsonTX2Kernel.git && cd buildJetsonTX2Kernel && ./getKernelSources.sh && cd .. && cp -R /usr/src/kernel/${kernel_name} ./${kernel_name}

echo -e "Starting to apply kernel patches and kernel building"
cd ./${kernel_name}
kernel_branch="master"
LINUX_BRANCH=$(uname -r)

#Check if we need to apply patches or get reload stock drivers (Developers' option)
[ "$#" -ne 0 -a "$1" == "reset" ] && reset_driver=1 || reset_driver=0

if [ $reset_driver -eq 1 ];
then
  echo -e "\e[43mUser requested to rebuild and reinstall ubuntu-xenial stock drivers\e[0m"
else
	# Patching kernel for RealSense devices
	echo -e "\e[32mApplying realsense-uvc patch\e[0m"
	patch -p1 < ../scripts/realsense-camera-formats_ubuntu-xenial-${kernel_branch}.patch
	echo -e "\e[32mApplying realsense-metadata patch\e[0m"
	patch -p1 < ../scripts/realsense-metadata-ubuntu-xenial-${kernel_branch}.patch
	echo -e "\e[32mApplying realsense-hid patch\e[0m"
	patch -p1 < ../scripts/realsense-hid-ubuntu-xenial-${kernel_branch}.patch
	echo -e "\e[32mApplying realsense-powerlinefrequency-fix patch\e[0m"
	patch -p1 < ../scripts/realsense-powerlinefrequency-control-fix.patch
fi

# Copy configuration
echo -e "Copying kernel config"
sudo cp /usr/src/linux-headers-$(uname -r)/.config .
sudo cp /usr/src/linux-headers-$(uname -r)/Module.symvers .

# Basic build for kernel modules
#yes "" | make silentoldconfig modules_prepare
echo -e "\e[32mPrepare kernel modules configuration\e[0m"
sudo make silentoldconfig modules_prepare

#Vermagic identity is required since kernel 4.3 ... 4.10
IFS='.' read -a kernel_version <<< "$LINUX_BRANCH"
if [ ${kernel_version[1]} > 3 ];
then
  sudo sed -i "s/\".*\"/\"$LINUX_BRANCH\"/g" ./include/generated/utsrelease.h
  sudo sed -i "s/.*/$LINUX_BRANCH/g" ./include/config/kernel.release
fi

# Build the uvc, accel and gyro modules
KBASE=`pwd`
cd drivers/media/usb/uvc
sudo cp $KBASE/Module.symvers .

echo -e "\e[32mCompiling uvc module\e[0m"
sudo make -j -C $KBASE M=$KBASE/drivers/media/usb/uvc/ modules
echo -e "\e[32mCompiling accelerometer and gyro modules\e[0m"
sudo make -j -C $KBASE M=$KBASE/drivers/iio/accel modules
sudo make -j -C $KBASE M=$KBASE/drivers/iio/gyro modules
echo -e "\e[32mCompiling v4l2-core modules\e[0m"
sudo make -j -C $KBASE M=$KBASE/drivers/media/v4l2-core modules

echo -e "Copy the patched modules to a sane location"
sudo cp $KBASE/drivers/media/usb/uvc/uvcvideo.ko ~/$LINUX_BRANCH-uvcvideo.ko
#sudo cp $KBASE/drivers/iio/accel/hid-sensor-accel-3d.ko ~/$LINUX_BRANCH-hid-sensor-accel-3d.ko
#sudo cp $KBASE/drivers/iio/gyro/hid-sensor-gyro-3d.ko ~/$LINUX_BRANCH-hid-sensor-gyro-3d.ko
mkdir -p ~/$LINUX_BRANCH
sudo cp $KBASE/drivers/media/v4l2-core/*.ko ~/$LINUX_BRANCH/
sudo cp $KBASE/drivers/media/usb/uvc/*.ko ~/$LINUX_BRANCH/

echo -e "\e[32mPatched kernels modules were created successfully\n\e[0m"

# Load the newly-built modules
try_module_insert videobuf2-core        ~/$LINUX_BRANCH/videobuf2-core.ko       /lib/modules/`uname -r`/kernel/drivers/media/v4l2-core/videobuf2-core.ko
try_module_insert videobuf2-v4l2        ~/$LINUX_BRANCH/videobuf2-v4l2.ko       /lib/modules/`uname -r`/kernel/drivers/media/v4l2-core/videobuf2-v4l2.ko
try_module_insert videobuf2-vmalloc     ~/$LINUX_BRANCH/videobuf2-vmalloc.ko     /lib/modules/`uname -r`/kernel/drivers/media/v4l2-core/videobuf2-vmalloc.ko
try_module_insert videobuf2-memops      ~/$LINUX_BRANCH/videobuf2-memops.ko     /lib/modules/`uname -r`/kernel/drivers/media/v4l2-core/videobuf2-memops.ko
try_module_insert uvcvideo              ~/$LINUX_BRANCH/uvcvideo.ko             /lib/modules/`uname -r`/kernel/drivers/media/usb/uvc/uvcvideo.ko
#try_module_insert videodev				~/$LINUX_BRANCH-videodev.ko 			/lib/modules/`uname -r`/kernel/drivers/media/v4l2-core/videodev.ko
# try_module_insert hid_sensor_accel_3d 	~/$LINUX_BRANCH-hid-sensor-accel-3d.ko 	/lib/modules/`uname -r`/kernel/drivers/iio/accel/hid-sensor-accel-3d.ko
# try_module_insert hid_sensor_gyro_3d	~/$LINUX_BRANCH-hid-sensor-gyro-3d.ko 	/lib/modules/`uname -r`/kernel/drivers/iio/gyro/hid-sensor-gyro-3d.ko

echo -e "\e[92m\n\e[1mScript has completed. Please consult the installation guide for further instruction.\n\e[0m"
