###### CONFIGURATION ######
CROSS_COMPILE ?= aarch64-linux-gnu-
CC := $(CROSS_COMPILE)gcc

### List of applications to be build
APPLICATIONS = mydrivertest


all:: $(APPLICATIONS)

clean::
	$(RM) $(APPLICATIONS) *.o

help:
	@echo "CROSS=$(CROSS_COMPILE)"
	@echo "CC=$(CC)"

