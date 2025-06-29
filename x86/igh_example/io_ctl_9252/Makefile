
CC ?= gcc

#INSTALL_ECAT = /usr/etherlab
INSTALL_ECAT = /usr/local

#CFLAGS := $(shell $(XENO_CONFIG) --skin=native --cflags)
#LDFLAGS := $(shell $(XENO_CONFIG) --skin=native --ldflags)

CFLAGS += -I$(INSTALL_ECAT)/include -I$(INSTALL_XENO)/include -I$(INSTALL_XENO)/include/cobalt
#CFLAGS += -I/usr/include/x86_64-linux-gnu/

LDFLAGS += -L$(INSTALL_XML)/lib -L$(INSTALL_ECAT)/lib -L$(INSTALL_XENO)/lib
LDFLAGS += -lethercat $(INSTALL_ECAT)/lib/libethercat.a
CC_FLAGS= -Wall
#CC_FLAGS= -Wall -D_GNU_SOURCE -D_REENTRANT -mtune=cortex-a53


TAR = ecat_test.app
DEPS = 
OBJ = ecat_test.o 

.PHONY: all
all: $(TAR) exec

%.o: %.c $(DEPS)
	@echo "oooooooo"
	$(CC) -c -o $@ $< $(CFLAGS) $(LDFLAGS)

$(TAR): $(OBJ)
	@echo "*** link ***"
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)

clean:
	rm -f ${OBJ} ${TAR}

exec:
	./$(TAR)

help:
	@echo cmd to cross compile
	@echo source /opt/fsl-imx-wayland/4.14-sumo/environment-setup-aarch64-poky-linux
	@echo export ARCH=arm64
	@echo unset LDFLAGS
	@echo $(SRC)

install:
	cp $(TAR).app /nfsshare

uninstall:
	rm -f /nfsshare/$(TAR).app
