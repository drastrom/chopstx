# Makefile for example application of Chopstx

PROJECT = sample

### This if for GNU/Linux

CHOPSTX = ..
LDSCRIPT= sample.ld
CSRC = sample.c

CHIP=gnu-linux
USE_SYS=

###################################
CROSS = 
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

MCU   = none
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DGNU_LINUX_EMULATION # -DUSE_SYS3 -DFREE_STANDING -DMHZ=48
OPT   = -g # -O3 -Os 
LIBS  =

####################
include ../rules.mk

# Override OUTFILES, CFLAGS, LDFLAGS
OUTFILES  = $(BUILDDIR)/$(PROJECT)
CFLAGS    = $(OPT) $(CWARN) -Wa,-alms=$(BUILDDIR)/$(notdir $(<:.c=.lst)) $(DEFS)
LDFLAGS   = $(LLIBDIR)
CFLAGS   += -MD -MP -MF .dep/$(@F).d

$(PROJECT): $(OBJS) $(OBJS_ADD)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(OBJS_ADD) $(LIBS)

board.h:
	@echo Please make a symbolic link \'board.h\' to a file in ../board;
	@exit 1

sys.c: board.h

distclean: clean
	rm -f board.h
