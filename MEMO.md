### Init Yocto build

```
$ mkdir rpi
$ cd rpi
$ git clone -b gatesgarth git://git.yoctoproject.org/poky.git
$ git clone -b gatesgarth git://git.openembedded.org/meta-openembedded
$ git clone -b gatesgarth git://git.openembedded.org/meta-raspberrypi
$ source poky/oe-init-build-env ./build
```

./build/conf/bblayers.conf
```
# POKY_BBLAYERS_CONF_VERSION is increased each time build/conf/bblayers.conf
# changes incompatibly
POKY_BBLAYERS_CONF_VERSION = "2" 

BBPATH = "${TOPDIR}"
BBFILES ?= ""

BBLAYERS ?= " \ 
  ~/yocto-builds/rpi/poky/meta \
  ~/yocto-builds/rpi/poky/meta-poky \
  ~/yocto-builds/rpi/poky/meta-yocto-bsp \
  ~/yocto-builds/rpi/meta-raspberrypi \
  ~/yocto-builds/rpi/meta-openembedded/meta-oe \
  ~/yocto-builds/rpi/meta-openembedded/meta-python \
  ~/yocto-builds/rpi/meta-openembedded/meta-networking \
  ~/yocto-builds/rpi/meta-openembedded/meta-multimedia \
  ~/yocto-builds/rpi/meta-openembedded/meta-filesystems \
  "
```

./build/conf/local.conf
```
...
MACHINE ??= "raspberrypi3"
...
DISTRO_FEATURES_append = "bluez5 bluetooth wifi systemd"
VIRTUAL-RUNTIME_init_manager = "systemd"
IMAGE_INSTALL_append = "crda iw bluez5 wpa-supplicant openssh"
```

### Create SD card image and SDK
```
$ cd rpi
$ . poky/oe-init-build-env
$ bitbake core-image-minimal
$ bitbake core-image-minimal -c populate_sdk
$ tmp/deploy/sdk/poky-glibc-x86_64-core-image-minimal-cortexa7t2hf-neon-vfpv4-raspberrypi3-toolchain-3.2.2.sh
```

### Build Kernel
```
$ cd <linux source root>
$ . /opt/poky/3.2.2/environment-setup-cortexa7t2hf-neon-vfpv4-poky-linux-gnueabi 
$ KERNEL=kernel7
$ make ARCH=arm CROSS_COMPILE=arm-poky-linux-gnueabi- bcm2709_defconfig
$ make ARCH=arm CROSS_COMPILE=arm-poky-linux-gnueabi- zImage modules dtbs
```

### Build skeleton driver
```
$ cd <linux source root>
$ . /opt/poky/3.2.2/environment-setup-cortexa7t2hf-neon-vfpv4-poky-linux-gnueabi 
$ make ARCH=arm CROSS_COMPILE=arm-poky-linux-gnueabi- -C . M=drivers/char/nrf24/
