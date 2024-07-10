CROSS_COMPILE := aarch64-linux-gnu-
CC := $(CROSS_COMPILE)gcc
INSTALL_XENO=/usr/xenomai

XENO_CONFIG := /usr/xenomai/bin/xeno-config

CFLAGS_XENOCFG = $(shell $(XENO_CONFIG) --skin=alchemy --cflags)
LDFLAGS_XENOCFG = $(shell $(XENO_CONFIG) --skin=alchemy --ldflags)

CFLAGS+=-I$(INSTALL_XENO)/include -I$(INSTALL_XENO)/include/cobalt
#CFLAGS+=-I/usr/include/x86_64-linux-gnu/

LDFLAGS+= -L$(INSTALL_XENO)/lib -ltrank -lalchemy -lcobalt
CC_FLAGS= -Wall -D_GNU_SOURCE -D_REENTRANT -mtune=cortex-a53

#CROSS_COMPILE?=/opt/fsl-imx-wayland/4.14-sumo/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-
#CC?=$(CROSS_COMPILE)gcc --sysroot=/opt/fsl-imx-wayland/4.14-sumo/sysroots/aarch64-poky-linux

OBJS_C=main.c
OBJS_O=$(patsubst %.c,%.o,$(OBJS_C) )
OBJS_ELF=$(patsubst %.c,%.elf,$(OBJS_C) )
TAR=main.app

##############################################################
all: build
	
build2:
	$(CC) -c $(OBJS_C) $(LDFLAGS) $(CFLAGS) $(CC_FLAGS) $(CFLAGS_XENOCFG)
	$(LD) $(OBJS_O) -o $(OBJS_ELF) $(LDFLAGS) $(LDFLAGS_XENOCFG)
	#$(OBJCOPY) -O binary -S $(OBJS_ELF) $(TAR)

build:
	$(CC) $(OBJS_C) -o $(TAR) $(LDFLAGS) $(CFLAGS) $(CC_FLAGS) $(CFLAGS_XENOCFG) $(LDFLAGS_XENOCFG)

clean:
	rm -f *.app *.o

check_inc:
	$(CC) -print-search-dirs

install:
	cp -f $(TAR) /nfsshare/

help:
	@echo $(CC)

help_cflags:
	@echo $(CFLAGS)
