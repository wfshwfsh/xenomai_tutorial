###### CONFIGURATION ######

### List of modules to be build
MODULES = irq_gpio

###### KERNEL MODULE BUILD (no change required normally) ######
ifneq ($(MODULES),)

### Default to sources of currently running kernel
#KSRC ?= /lib/modules/$(shell uname -r)/build
KSRC ?= /home/will/git/xeno/linux/dist/lib/modules/5.10.110-xenomai+/build

OBJS     := ${patsubst %, %.o, $(MODULES)}
CLEANMOD := ${patsubst %, .%*, $(MODULES)}
PWD      := $(shell if [ "$$PWD" != "" ]; then echo $$PWD; else pwd; fi)

### Kernel 2.6 or 3.0

obj-m        := $(OBJS)
EXTRA_CFLAGS := -I$(KSRC)/include/xenomai -I$(KSRC)/include/xenomai/rtdm 
#$(ADD_CFLAGS)

all::
	ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- $(MAKE) -C $(KSRC) M=$(PWD) modules


## Target for capturing 2.4 module CFLAGS
modules:
	@echo "$(CFLAGS)"

clean::
	$(RM) $(CLEANMOD) *.o *.ko *.mod *.mod.c Module*.symvers Module.markers modules.order
	$(RM) -R .tmp*

help:
	@echo "KSRC: $(KSRC)"
	@echo "OBJS: $(OBJS)"
	@echo "obj-m: $(obj-m)"

endif
