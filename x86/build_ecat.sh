#!/bin/bash

DIR_TOP="$(pwd)"
#DIR_LINUX="$DIR_TOP/linux-5.10.36"
DIR_LINUX="$DIR_TOP/linux-xenomai-iotg"
DIR_ECAT="$DIR_TOP/igh_ecat"

echo "DIR_TOP: $DIR_TOP"
echo "DIR_LINUX: $DIR_LINUX"
echo "DIR_ECAT: $DIR_ECAT"


cd $DIR_ECAT
autoupdate
./bootstrap

configure_option="--enable-wildcards=yes --disable-generic --disable-8139too --enable-igb --enable-igc --enable-e1000e"
configure_option2="--enable-rtdm=yes --with-xenomai-dir=/usr/xenomai"
configure_cmd="./configure --prefix=/usr/local --with-linux-dir=$DIR_LINUX --with-module-dir=/kernel/drivers/ethercat"
configure_cmd="$configure_cmd $configure_option $configure_option2"

echo "pwd: $PWD"
echo "configure_cmd: $configure_cmd"
eval $configure_cmd

### copy kernel modules to here
cp -r $DIR_LINUX/dist $DIR_ECAT
echo "#### ECAT: make modules ####"
make modules

echo "#### ECAT: make install ####"
make DESTDIR=$DIR_ECAT/dist install

echo "#### ECAT: make module_install ####"
make INSTALL_MOD_PATH=$DIR_ECAT/dist modules_install
