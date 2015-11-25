# Makefile for Orange Pi PC demo of Chopstx

PROJECT = sample
CHOPSTX = ..
LDSCRIPT= sample.ld
CSRC = chx-sun8iw7.c sample.c

###################################
HOSTCC=gcc
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

MCU   = cortex-a7
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DMAKE_ENTRY_PUBLIC -DFREE_STANDING -DSUNXI_SUN8IW7

##OPT   = -O3 -Os -g -march=armv7ve -mfpu=neon -mhard-float
OPT   = -O3 -Os -g -march=armv7ve
LIBS  =

OUTFILES = $(BUILDDIR)/boot0.bin
####################
include ../rules.mk

chx-sun8i7w.c: board-orange-pi-pc.h

### boot0.bin
BOOT0_SSRC=boot0.S
BOOT0_CSRC=boot-init.c boot-main.c # boot-dram.c
BOOT0_OBJS   = $(addprefix $(BUILDDIR)/, $(notdir $(BOOT0_SSRC:.S=.o)))
BOOT0_OBJS  += $(addprefix $(BUILDDIR)/, $(notdir $(BOOT0_CSRC:.c=.o)))
BOOT0_LINK_OPTIONS=-nostdlib -nostartfiles -Wl,--section-start=.text=0x0000

$(BUILDDIR)/mkhd $(BOOT0_OBJS): | $(BUILDDIR)

$(BUILDDIR)/boot0.bin: $(BUILDDIR)/boot0-raw.bin $(BUILDDIR)/mkhd
	./$(BUILDDIR)/mkhd $< $@

$(BUILDDIR)/boot0-raw.bin: $(BUILDDIR)/boot0
	$(OBJCOPY) --only-section=.text -O binary $< $@

$(BUILDDIR)/boot0: $(BOOT0_OBJS)
	$(CC) -e _start $(BOOT0_LINK_OPTIONS) -o $@ $(BOOT0_OBJS)

$(BUILDDIR)/boot0.o: boot0.S Makefile
	$(CC) -c -march=armv7ve $< -o $@

$(BUILDDIR)/boot-init.o: boot-init.c io.h
	$(CC) -c $(CFLAGS) -I. $(IINCDIR) $< -o $@

$(BUILDDIR)/boot-main.o: boot-main.c io.h
	$(CC) -c $(CFLAGS) -I. $(IINCDIR) $< -o $@

$(BUILDDIR)/boot-dram.o: boot-dram.c io.h
	$(CC) -c $(CFLAGS) -I. $(IINCDIR) $< -o $@

$(BUILDDIR)/mkhd: mkhd.c
	$(HOSTCC) -o $@ mkhd.c

.PHONY : distclean
