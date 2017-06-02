# Makefile for example application of Chopstx

PROJECT = sample

### This is for GNU/Linux

CHOPSTX = ..
LDSCRIPT= 
CSRC = sample.c

CHIP=gnu-linux
USE_SYS=
EMULATION=yes

###################################
CROSS = 
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

MCU   = none
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DGNU_LINUX_EMULATION # -DUSE_SYS3 -DFREE_STANDING -DMHZ=48
OPT   = -g # -O3 -Os 
LIBS  = -lpthread

####################
include ../rules.mk

board.h:
	@echo Please make a symbolic link \'board.h\' to a file in ../board;
	@exit 1

distclean: clean
	rm -f board.h
