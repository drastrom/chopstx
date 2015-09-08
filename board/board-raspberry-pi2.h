#define BOARD_NAME "Raspberry Pi 2"
#define BOARD_ID    0x4c2c12b1

/* IO setup.  */

#define BCM2709_ST_BASE			0x3F003000

#define BCM2709_GPIO_BASE		0x3F200000
#define BCM2709_UART0_BASE		0x3F201000
#define BCM2709_MMCI0_BASE		0x3F202000
#define BCM2709_I2S_BASE		0x3F203000
#define BCM2709_SPI0_BASE		0x3F204000
#define BCM2709_BSC0_BASE		0x3F205000
#define BCM2709_UART1_BASE		0x3F215000
#define BCM2709_EMMC_BASE		0x3F300000
#define BCM2709_SMI_BASE		0x3F600000
#define BCM2709_BSC1_BASE		0x3F804000
#define BCM2709_USB_BASE		0x3F980000

/* GPIO registers.  */
#define BCM2709_GPIO_FSEL0		(BCM2709_GPIO_BASE+0)
#define BCM2709_GPIO_FSEL1		(BCM2709_GPIO_BASE+0x04)
#define BCM2709_GPIO_FSEL2		(BCM2709_GPIO_BASE+0x08)
#define BCM2709_GPIO_FSEL3		(BCM2709_GPIO_BASE+0x0C)
#define BCM2709_GPIO_FSEL4		(BCM2709_GPIO_BASE+0x10)
#define BCM2709_GPIO_FSEL5		(BCM2709_GPIO_BASE+0x14)
#define BCM2709_GPIO_SET0		(BCM2709_GPIO_BASE+0x1C)
#define BCM2709_GPIO_SET1		(BCM2709_GPIO_BASE+0x20)
#define BCM2709_GPIO_CLR0		(BCM2709_GPIO_BASE+0x28)
#define BCM2709_GPIO_CLR1		(BCM2709_GPIO_BASE+0x2C)
#define BCM2709_GPIO_LEV0		(BCM2709_GPIO_BASE+0x34)
#define BCM2709_GPIO_LEV1		(BCM2709_GPIO_BASE+0x38)
#define BCM2709_GPIO_EDS0		(BCM2709_GPIO_BASE+0x40)
#define BCM2709_GPIO_EDS1		(BCM2709_GPIO_BASE+0x44)
#define BCM2709_GPIO_REN0		(BCM2709_GPIO_BASE+0x4C)
#define BCM2709_GPIO_REN1		(BCM2709_GPIO_BASE+0x50)
#define BCM2709_GPIO_FEN0		(BCM2709_GPIO_BASE+0x58)
#define BCM2709_GPIO_FEN1		(BCM2709_GPIO_BASE+0x5C)
#define BCM2709_GPIO_HEN0		(BCM2709_GPIO_BASE+0x64)
#define BCM2709_GPIO_HEN1		(BCM2709_GPIO_BASE+0x68)
#define BCM2709_GPIO_LEN0		(BCM2709_GPIO_BASE+0x70)
#define BCM2709_GPIO_LEN1		(BCM2709_GPIO_BASE+0x74)
#define BCM2709_GPIO_AREN0		(BCM2709_GPIO_BASE+0x7C)
#define BCM2709_GPIO_AREN1		(BCM2709_GPIO_BASE+0x80)
#define BCM2709_GPIO_AFEN0		(BCM2709_GPIO_BASE+0x88)
#define BCM2709_GPIO_AFEN1		(BCM2709_GPIO_BASE+0x8C)
#define BCM2709_GPIO_PUD		(BCM2709_GPIO_BASE+0x94)
#define BCM2709_GPIO_PUDCLK0		(BCM2709_GPIO_BASE+0x98)
#define BCM2709_GPIO_PUDCLK1		(BCM2709_GPIO_BASE+0x9C)

/* ARM control logic.  */

#define BCM2709_CONTROL0		0x3F00B000
#define BCM2709_CONTROL1		0x3F00B440
#define BCM2709_STATUS			0x3F00B444
#define BCM2709_IRQ_PENDING0		0x3F00B200
#define BCM2709_IRQ_PENDING1		0x3F00B204
#define BCM2709_IRQ_PENDING2		0x3F00B208
#define BCM2709_FIQ_CONTROL		0x3F00B20C
#define BCM2709_IRQ_ENABLE1		0x3F00B210
#define BCM2709_IRQ_ENABLE2		0x3F00B214
#define BCM2709_IRQ_ENABLE3		0x3F00B218
#define BCM2709_IRQ_DISABLE1		0x3F00B21C
#define BCM2709_IRQ_DISABLE2		0x3F00B220
#define BCM2709_IRQ_DISABLE3		0x3F00B224

/* ARM timer (SP804 like) module.  */
#define BCM2709_ARM_TIMER_LOD		0x3F00B400
#define BCM2709_ARM_TIMER_VAL		0x3F00B404
#define BCM2709_ARM_TIMER_CTL		0x3F00B408
#define BCM2709_ARM_TIMER_CLI		0x3F00B40C
#define BCM2709_ARM_TIMER_RIS		0x3F00B410
#define BCM2709_ARM_TIMER_MIS		0x3F00B414
#define BCM2709_ARM_TIMER_RLD		0x3F00B418
#define BCM2709_ARM_TIMER_DIV		0x3F00B41C
#define BCM2709_ARM_TIMER_CNT		0x3F00B420

/* ... */

/* ARM local registers.  */
/* Core timer registers.  */
#define BCM2709_CORE_PRESCALER		0x40000008
#define BCM2709_CORE_TIMER_LO		0x4000001C
#define BCM2709_CORE_TIMER_HI		0x40000020

/* GPU interrupt control register.  */
#define BCM2709_GPU_IRQ_CNTL		0x4000000C

/* PMU interrupt control registers.  */
#define BCM2709_PMU_IRQ_CNTL_SET	0x40000010
#define BCM2709_PMU_IRQ_CNTL_CLR	0x40000014

/* Local timer registers.  */
/* 0: Core0 IRQ, 1: Core1 IRQ,..., 4: Core0 FIQ, 5: COre1 FIQ,... */
#define BCM2709_LOCAL_TIMER_IRQ_CNTL	0x40000028
/* bit31=Intr(RO), bit29=IntrEnable, bit28=TimerEnable,0-27 Reload value */
#define BCM2709_LOCAL_TIMER_CNTL_STATUS	0x40000034
/* bit31=Intr flag clear(high to clear), bit30=Reload(high to reload) */
#define BCM2709_LOCAL_TIMER_CLR_RELOAD	0x40000038

/*
 * Timers interrupt control registers/bits.
 */
#define BCM2709_CORE0_TIMER_IRQ_CNTL	0x40000040
#define BCM2709_CORE1_TIMER_IRQ_CNTL	0x40000044
#define BCM2709_CORE2_TIMER_IRQ_CNTL	0x40000048
#define BCM2709_CORE3_TIMER_IRQ_CNTL	0x4000004C

#define BCM2709_TIMER0_ENABLE_IRQ	0x00000001
#define BCM2709_TIMER1_ENABLE_IRQ	0x00000002
#define BCM2709_TIMER2_ENABLE_IRQ	0x00000004
#define BCM2709_TIMER3_ENABLE_IRQ	0x00000008
#define BCM2709_TIMER0_ENABLE_FIQ	0x00000010
#define BCM2709_TIMER1_ENABLE_FIQ	0x00000020
#define BCM2709_TIMER2_ENABLE_FIQ	0x00000040
#define BCM2709_TIMER3_ENABLE_FIQ	0x00000080

/*
 * Mailbox interrupt control registers/bits.
 */
#define BCM2709_CORE0_MBOX_IRQ_CNTL	0x40000050
#define BCM2709_CORE1_MBOX_IRQ_CNTL	0x40000054
#define BCM2709_CORE2_MBOX_IRQ_CNTL	0x40000058
#define BCM2709_CORE3_MBOX_IRQ_CNTL	0x4000005C

#define BCM2709_MBOX0_ENABLE_IRQ	0x00000001
#define BCM2709_MBOX1_ENABLE_IRQ	0x00000002
#define BCM2709_MBOX2_ENABLE_IRQ	0x00000004
#define BCM2709_MBOX3_ENABLE_IRQ	0x00000008
#define BCM2709_MBOX0_ENABLE_FIQ	0x00000010
#define BCM2709_MBOX1_ENABLE_FIQ	0x00000020
#define BCM2709_MBOX2_ENABLE_FIQ	0x00000040
#define BCM2709_MBOX3_ENABLE_FIQ	0x00000080

/*
 * Interrupt source registers/bits.
 */
#define BCM2709_CORE0_IRQ_SOURCE	0x40000060
#define BCM2709_CORE1_IRQ_SOURCE	0x40000064
#define BCM2709_CORE2_IRQ_SOURCE	0x40000068
#define BCM2709_CORE3_IRQ_SOURCE	0x4000006C
#define BCM2709_CORE0_FIQ_SOURCE	0x40000070
#define BCM2709_CORE1_FIQ_SOURCE	0x40000074
#define BCM2709_CORE2_FIQ_SOURCE	0x40000078
#define BCM2709_CORE3_FIQ_SOURCE	0x4000007C

#define BCM2709_INT_SRC_TIMER0		0x00000001
#define BCM2709_INT_SRC_TIMER1		0x00000002
#define BCM2709_INT_SRC_TIMER2		0x00000004
#define BCM2709_INT_SRC_TIMER3		0x00000008
#define BCM2709_INT_SRC_MBOX0		0x00000010
#define BCM2709_INT_SRC_MBOX1		0x00000020
#define BCM2709_INT_SRC_MBOX2		0x00000040
#define BCM2709_INT_SRC_MBOX3		0x00000080
#define BCM2709_INT_SRC_GPU		0x00000100	/* One core only.  */
#define BCM2709_INT_SRC_PMU		0x00000200
#define BCM2709_INT_SRC_AXI		0x00000400	/* Core 0 only.  */
#define BCM2709_INT_SRC_LTIMER		0x00000800

/*
 * Mailbox write-set registers. (Write only)
 */
#define BCM2709_CORE0_MBOX0_SET		0x40000080
#define BCM2709_CORE0_MBOX1_SET		0x40000084
#define BCM2709_CORE0_MBOX2_SET		0x40000088
#define BCM2709_CORE0_MBOX3_SET		0x4000008C
#define BCM2709_CORE1_MBOX0_SET		0x40000090
#define	BCM2709_CORE1_MBOX1_SET		0x40000094
#define	BCM2709_CORE1_MBOX2_SET		0x40000098
#define	BCM2709_CORE1_MBOX3_SET		0x4000009C
#define	BCM2709_CORE2_MBOX0_SET		0x400000A0
#define	BCM2709_CORE2_MBOX1_SET		0x400000A4
#define	BCM2709_CORE2_MBOX2_SET		0x400000A8
#define	BCM2709_CORE2_MBOX3_SET		0x400000AC
#define	BCM2709_CORE3_MBOX0_SET		0x400000B0
#define	BCM2709_CORE3_MBOX1_SET		0x400000B4
#define	BCM2709_CORE3_MBOX2_SET		0x400000B8
#define	BCM2709_CORE3_MBOX3_SET		0x400000BC

/*
 * Mailbox write-clear registers. (Readable/ Write high to clear)
 */
#define BCM2709_CORE0_MBOX0_RDCLR	0x400000C0
#define BCM2709_CORE0_MBOX1_RDCLR	0x400000C4
#define BCM2709_CORE0_MBOX2_RDCLR	0x400000C8
#define BCM2709_CORE0_MBOX3_RDCLR	0x400000CC
#define BCM2709_CORE1_MBOX0_RDCLR	0x400000D0
#define BCM2709_CORE1_MBOX1_RDCLR	0x400000D4
#define BCM2709_CORE1_MBOX2_RDCLR	0x400000D8
#define BCM2709_CORE1_MBOX3_RDCLR	0x400000DC
#define BCM2709_CORE2_MBOX0_RDCLR	0x400000E0
#define BCM2709_CORE2_MBOX1_RDCLR	0x400000E4
#define BCM2709_CORE2_MBOX2_RDCLR	0x400000E8
#define BCM2709_CORE2_MBOX3_RDCLR	0x400000EC
#define BCM2709_CORE3_MBOX0_RDCLR	0x400000F0
#define BCM2709_CORE3_MBOX1_RDCLR	0x400000F4
#define BCM2709_CORE3_MBOX2_RDCLR	0x400000F8
#define BCM2709_CORE3_MBOX3_RDCLR	0x400000FC

/* Some IRQ numbers on VIC.  */
#define BCM2709_VIC_USB			9
#define BCM2709_VIC_GPIO0		49
#define BCM2709_VIC_GPIO1		50
#define BCM2709_VIC_GPIO2		51
#define BCM2709_VIC_GPIO3		52
#define BCM2709_VIC_I2C			53
#define BCM2709_VIC_SPI			54
#define BCM2709_VIC_I2SPCM		55
#define BCM2709_VIC_SDIO		56
#define BCM2709_VIC_UART		57
#define BCM2709_ARM_TIMER0		64
#define BCM2709_LOC_INT_TIMER0		96
#define BCM2709_LOC_INT_TIMER1		97
#define BCM2709_LOC_INT_TIMER2		98
#define BCM2709_LOC_INT_TIMER3		99
#define BCM2709_LOC_INT_MBOX0		100
#define BCM2709_LOC_INT_MBOX1		101
#define BCM2709_LOC_INT_MBOX2		102
#define BCM2709_LOC_INT_MBOX3		103
#define BCM2709_LOC_INT_GPU		104
#define BCM2709_LOC_INT_PMU		105
#define BCM2709_LOC_INT_AXI		106
#define BCM2709_LOC_INT_LTIMER		107