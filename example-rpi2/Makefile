# Makefile for example application of Chopstx

PROJECT = sample

CHOPSTX = ..
LDSCRIPT= sample.ld
CSRC = sys.c sample.c ttable.c

###################################
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

MCU   = cortex-a7
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DMAKE_ENTRY_PUBLIC -DHAVE_SYS_H -DFREE_STANDING -DSYST_BMC2709
##OPT   = -O3 -Os -g -march=armv7ve -mfpu=neon -mhard-float
OPT   = -O3 -Os -g -march=armv7ve
LIBS  =

####################
include ../rules.mk

board.h:
	@echo Please make a symbolic link \'board.h\' to a file in ../board;
	@exit 1

sys.c: board.h

ttable.c : mktt.py
	python3 $< >$@

distclean: clean
	rm -f board.h
