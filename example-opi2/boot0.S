	.org	0x0000
	.text
	.global	_start
	.arm
_start:
	b	start
boot_header:
	.ascii	"eGON.BT0"
	.word	0x00000000	// Check sum: to be filled
	.word	0x00008000	// Length: to be filled
	.word	0x00000030	// Header size
	.word	0x00000000
	.word	0x00000000
	.word	0x00000000
	.word	0x00000000
	.ascii	"\000\0004.0.0\000"

	.align	2
start:	
	mrs	r0, cpsr
	bic	r0, r0, #0x001f	// ARMV7_MODE_MASK
	orr	r0, r0, #0x0013	// ARMV7_SVC_MODE
	orr	r0, r0, #0x00c0	// ARMV7_IRQ_MASK | ARMV7_FIQ_MASK
	bic	r0, r0, #0x0200	// Little Endian
	msr	cpsr_c, r0
	////
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #0x2000	// (--V-------------) vector table: 0000
	bic	r0, r0, #0x0007	// (-------------CAM) MMU:          disable 
	//// 
	orr	r0, r0, #0x0800	// (----Z-----------) Branch prediction enable
	orr	r0, r0, #0x1000	// (---I------------) I-cache enable
	mcr	p15, 0, r0, c1, c0, 0
	////
	mov	sp, #0x0004b000
	blx	boot_init
	blx	boot_dram
	////
	@bl	clear_bss
	blx	boot_main	// NEVER RETURN
0:	b	0b
	////
clear_bss:
	ldr	r0, =__bss_start
	ldr	r1, =_end
	mov	r2, #0
	////
0:	str	r2, [r0]
	add	r0, r0, #4
	cmp	r0, r1
	bne	0b
	////
	mov	pc, lr

	.align	2
	.text
	.global	_reset
	.thumb
_reset:	
	mov	r0, #0
	ldr	r1, 1f
	str	r0, [r1]
0:
	b	0b
1:
	.word	0x01f01d40 @ CPU_SYS_RST_REG
