
CC ?= gcc
PATH_XENO = /usr/xenomai

XENO_CONFIG := $(PATH_XENO)/bin/xeno-config
#INSTALL_ECAT = /usr/etherlab
INSTALL_ECAT = /usr/local

CFLAGS := $(shell $(XENO_CONFIG) --skin=alchemy --cflags)
LDFLAGS := $(shell $(XENO_CONFIG) --skin=alchemy --ldflags)
#CC := $(shell $(XENO_CONFIG) --cc)

CFLAGS += -I$(INSTALL_ECAT)/include
LDFLAGS += -L$(INSTALL_ECAT)/lib
LDFLAGS += -lethercat_rtdm $(INSTALL_ECAT)/lib/libethercat_rtdm.a

PATH_XENO_INC = $(PATH_XENO)/include
INC = -I$(PATH_XENO_INC)/trank -I$(PATH_XENO_INC)/ -I$(PATH_XENO_INC)/mercury -I$(PATH_XENO_INC)/cobalt

LIB = -L/usr/xenomai/lib

#PARAM = $(LIB) -D_GNU_SOURCE -D_REENTRANT -Wall -pipe -D__XENO__ -lnative -lxenomai -lrt -pthread $(INC)
PARAM = $(LIB) -D_GNU_SOURCE -D_REENTRANT -Wall -pipe -D__XENO__ -lrt -pthread $(INC)


TAR = io_test_rt_lan9252

.PHONY: all
all: clean build

build:
	$(CC) $(TAR).c -o $(TAR).app $(CFLAGS) $(LDFLAGS) $(PARAM)

clean:
	rm -rf *.app

exec:
	./$(TAR).app

help:
	@echo cmd to cross compile
	@echo source /opt/fsl-imx-wayland/4.14-sumo/environment-setup-aarch64-poky-linux
	@echo export ARCH=arm64
	@echo unset LDFLAGS

install:
	cp $(TAR).app /nfsshare

uninstall:
	rm -f /mnt/share/$(TAR).app

