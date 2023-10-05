#!/bin/bash

VERSION=1.0.0.2

config_kernel() {
 pushd kernel
 make menuconfig
 popd
}

build_kernel() {
  pushd kernel

  if [ "$1" == "clean" ]; then
    make distclean
  else
    #make epc1107_defconfig
    make uImage dtbs modules -j32

    if [ "$?" != "0" ]; then
      echo "Error: cannot build kernel"
      exit
    fi

    cp arch/arm/boot/uImage ../output
    cp arch/arm/boot/dts/epc1107_rgb_800x480.dtb ../output
    cp arch/arm/boot/dts/epc1107_rgb_480x272.dtb ../output
    cp arch/arm/boot/dts/epc1107_mipi_1280x800.dtb ../output
    cp arch/arm/boot/dts/epc1107_mipi_720x720.dtb ../output
  fi

  popd
}

build_uboot() {
  echo $(pwd)
  pushd uboot

  if [ "$1" == "clean" ]; then
    make distclean
  else
    make zmp110x_evb_config
    make -j4

    if [ "$?" != "0" ]; then
      echo "Error: cannot build uboot"
      exit
    fi

    cp u-boot.img ../output
    cp spl/u-boot-spl.bin ../output
  fi

  popd
}

build_prepare() {
  cd $(dirname $1)

  mkdir -p output

  #sudo rm /opt/arm-zlgmcu-linux-uclibcgnueabi
  #sudo ln -s $(pwd)/../arm-zlgmcu-linux-uclibcgnueabi /opt
  #if [ "$?" != "0" ]; then   
  #  echo "Error: Cannot create /opt/arm-zlgmcu-linux-uclibcgnueabi symlink"
  #  exit 1
  #fi
  export FORCE_UNSAFE_CONFIGURE=1

  export ARCH=arm
  export CROSS_COMPILE=/opt/arm-zlgmcu-linux-uclibcgnueabi/bin/arm-zlgmcu-linux-uclibcgnueabi-
}

build_prepare $0

case "x$1" in

  xconfig_kerenl)
    config_kernel
    echo "Config Kerenl"
    ;;
    
  xconfig_uboot)
    config_uboot
    echo "Config Uboot"
    ;;

  xclean)
    build_uboot clean
    build_kernel clean
    rm output -rf
    ;;
    
  x)
    build_uboot
    build_kernel
    echo "Build Finish"
    ;;
  *)
    echo "Usage: $(basename $0) [clean]"
esac

