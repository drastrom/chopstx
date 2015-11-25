/*
 * entry.c - Entry routine when reset and interrupt vectors.
 *
 * Copyright (C) 2013, 2014, 2015  Flying Stone Technology
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * This file is a part of Chopstx, a thread library for embedded.
 *
 * Chopstx is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Chopstx is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As additional permission under GNU GPL version 3 section 7, you may
 * distribute non-source form of the Program without the copy of the
 * GNU GPL normally required by section 4, provided you inform the
 * receipents of GNU GPL by a written offer.
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>

#ifdef HAVE_SYS_H
#define INLINE __attribute__ ((used))
#include "sys.h"
#include "board.h"
#undef STM32F10X_MD		/* Prepare for high density device, too.  */
#else
# if !defined(SUNXI_SUN8IW7)
#include "board.h"
#include "clk_gpio_init.c"
# endif
#endif


#ifdef MAKE_ENTRY_PUBLIC
#define STATIC_ENTRY
#else
#define STATIC_ENTRY static
#endif

extern uint8_t __main_stack_end__;
extern void svc (void);
extern void preempt (void);
extern void chx_timer_expired (void);
extern void chx_handle_intr (void);

#if defined(__ARM_ARCH_7A__)
extern void chx_secondary_init (void);

/* Save r0, r1, r2, r3, r12, lr, pc, xpsr and branch to real handlers.  */
static void __attribute__ ((naked))
svc_handler (void)
{
  asm volatile ("sub	sp, sp, #8\n\t"
		"srsia	sp, #19\n\t"	/* svc 19 */
		"str	r12, [sp, #-8]!\n\t"
		"push	{r0, r1, r2, r3}\n\t"
		"mrs	r0, LR_usr\n\t"
		"str	r0, [sp, #5*4]\n\t"
		"b	svc" : /* no output */ : /* no input */ : "memory");
}

# if defined(SUNXI_SUN8IW7)
static void __attribute__ ((naked))
irq_handler (void)
{
  asm volatile ("sub	r14, r14, #4\n\t"
		"sub	sp, sp, #8\n\t"
		"srsia	sp, #18\n\t"	/* irq 18 */
		"str	r12, [sp, #-8]!\n\t"
		"push	{r0, r1, r2, r3}\n\t"
		"mrs	r0, LR_usr\n\t"
		"str	r0, [sp, #5*4]\n\t"
		"bl	chx_irq\n\t"
		"cmp	r0, #0\n\t"
		"beq	2f\n\t"
		/* Change to SVC mode after copying stack.  */
		"mrs	r2, SP_svc\n\t"
		"sub	r3, r2, #32\n\t"
		"msr	SP_svc, r3\n\t"
	"0:\n\t"
		"ldr	r1, [sp], #4\n\t"
		"str	r1, [r3], #4\n\t"
		"cmp	r3, r2\n\t"
		"bne	0b\n\t"
		"cpsid	i, #19\n\t"
		"cmp	r0, #1\n\t"
		"beq	1f\n\t"
		"b	preempt\n"
	"1:\n\t"
		"bl	chx_timer_expired\n"
	"2:\n\t"
		"ldm	sp!, {r0-r3}\n\t"
		"ldr	r12, [sp], #4\n\t"
		"add	sp, #4\n\t"
		"rfe	sp!" : /* no output */ : /* no input */ : "memory");
}
# else
static void __attribute__ ((naked))
irq_handler (void)
{
  asm volatile ("sub	r14, r14, #4\n\t"
		"sub	sp, sp, #8\n\t"
		"srsia	sp, #18\n\t"	/* irq 18 */
		"str	r12, [sp, #-8]!\n\t"
		"push	{r0, r1, r2, r3}\n\t"
		"mrs	r0, spsr\n\t"
		"and	r0, r0, #31\n\t"
		"cmp	r0, #19\n\t"	/* svc 19 */
		"ite	eq\n\t"
		"mrseq	r1, LR_svc\n\t"
		"mrsne	r1, LR_usr\n\t"
		"str	r1, [sp, #5*4]\n\t"
		"bl	irq\n\t"
		"cmp	r0, #0\n\t"
		"beq	2f\n\t"
		/* Change to SVC mode after copying stack.  */
		"mrs	r2, SP_svc\n\t"
		"sub	r3, r2, #32\n\t"
		"msr	SP_svc, r3\n\t"
	"0:\n\t"
		"ldr	r1, [sp], #4\n\t"
		"str	r1, [r3], #4\n\t"
		"cmp	r3, r2\n\t"
		"bne	0b\n\t"
		"cpsid	i, #19\n\t"
		"cmp	r0, #1\n\t"
		"beq	1f\n\t"
		"b	preempt\n"
	"1:\n\t"
		"bl	chx_timer_expired\n"
	"2:\n\t"
		"ldm	sp!, {r0-r3}\n\t"
		"ldr	r12, [sp], #4\n\t"
		"add	sp, #4\n\t"
		"rfe	sp!" : /* no output */ : /* no input */ : "memory");
}
# endif
#endif

static void nmi (void)
{
  for (;;);
}

#if !defined(__ARM_ARCH_7A__)
static void hard_fault (void)
{
#if defined(__ARM_ARCH_6M__)
  register uint32_t primask;

  asm ("mrs	%0, PRIMASK" : "=r" (primask));

  if (primask)
    asm volatile ("b	svc");
  else
    for (;;);
#else
  for (;;);
#endif
}
#endif

#if !defined(__ARM_ARCH_7A__)
static void mem_manage (void)
{
  for (;;);
}
#endif

static void bus_fault (void)
{
#if defined(__ARM_ARCH_7A__)
  /* r0=DFSR r1=DFAR r2=IFSR r3=IFAR */
  asm volatile ("mrc	p15, 0, r0, c5, c0, 0\n\t"
		"mrc	p15, 0, r1, c6, c0, 0\n\t"
		"mrc	p15, 0, r2, c5, c0, 1\n\t"
		"mrc	p15, 0, r3, c6, c0, 2"
		 : : : "r0", "r1", "r2", "r3");
#endif
  for (;;);
}

static void usage_fault (void)
{
  for (;;);
}

#if !defined(__ARM_ARCH_7A__)
static void none (void)
{
}
#endif

#define C_S_SUB(arg0, arg1, arg2) arg0 #arg1 arg2
#define COMPOSE_STATEMENT(arg0,arg1,arg2)  C_S_SUB (arg0, arg1, arg2)

#if defined(__ARM_ARCH_6M__)
__attribute__ ((used,section(".bss.startup.0")))
uint32_t vectors_in_ram[48];
#endif

/*
 * This routine only changes PSP and not MSP.
 */
STATIC_ENTRY __attribute__ ((naked,section(".text.startup.0")))
void entry (void)
{
  asm volatile (
#if defined(__ARM_ARCH_7A__)
		"bl	stack_init\n\t"
		"bl	mmu_init\n\t"
#else
		"bl	clock_init\n\t"
#endif
		/* Clear BSS section.  */
		"mov	r0, #0\n\t"
		"ldr	r1, =_bss_start\n\t"
		"ldr	r2, =_bss_end\n"
	"0:\n\t"
		"cmp	r1, r2\n\t"
		"beq	1f\n\t"
#if defined(__ARM_ARCH_6M__)
		"str	r0, [r1]\n\t"
		"add	r1, #4\n\t"
#else
		"str	r0, [r1], #4\n\t"
#endif
		"b	0b\n"
	"1:\n\t"
#if !defined(__ARM_ARCH_7A__)
		/* Copy data section.  */
		"ldr	r1, =_data\n\t"
		"ldr	r2, =_edata\n\t"
		"ldr	r3, =_textdata\n"
	"2:\n\t"
		"cmp	r1, r2\n\t"
		"beq	3f\n\t"
#if defined(__ARM_ARCH_6M__)
		"ldr	r0, [r3]\n\t"
		"str	r0, [r1]\n\t"
		"add	r3, #4\n\t"
		"add	r1, #4\n\t"
#else
		"ldr	r0, [r3], #4\n\t"
		"str	r0, [r1], #4\n\t"
#endif
		"b	2b\n"
	"3:\n\t"
#else
		/* Copy exception vectors.  */
		"ldr	r0, =except_vector\n\t"
		"mov	r1, #0\n\t"
		"mov	r2, #32\n"
	"2:\n\t"
		"ldr	r3, [r0], #4\n\t"
		"str	r3, [r1], #4\n\t"
		"cmp	r1, r2\n\t"
		"bne	2b\n\t"
		"ldr	r0, =vector_table\n\t"
		"mov	r2, #64\n"
	"3:\n\t"
		"ldr	r3, [r0], #4\n\t"
		"str	r3, [r1], #4\n\t"
		"cmp	r1, r2\n\t"
		"bne	3b\n\t"
		"bl	irq_init\n\t"
#endif
		/* Switch to PSP.  */
		"ldr	r0, =__process0_stack_end__\n\t"
		COMPOSE_STATEMENT ("sub	r0, #", CHOPSTX_THREAD_SIZE, "\n\t")
#if defined(__ARM_ARCH_7A__)
		"msr	SP_usr, r0\n\t" /* Process (main routine) stack.  */
		"cpsid	i\n\t"
		"isb\n\t"
#else
		"msr	PSP, r0\n\t" /* Process (main routine) stack.  */
		"mov	r1, #2\n\t"
		"msr	CONTROL, r1\n\t"
		"isb\n\t"
#endif
		"bl	chx_init\n\t"
		"bl	chx_systick_init\n\t"
		"bl	gpio_init\n\t"
#if defined(__ARM_ARCH_7A__)
		"bl	chx_secondary_init\n\t"
#endif
		/* Enable interrupts.  */
#if defined(__ARM_ARCH_7M__)
		"mov	r0, #0\n\t"
		"msr	BASEPRI, r0\n\t"
#endif
#if defined(__ARM_ARCH_7A__)
		"cpsie	i, #31\n\t"
#else
		"cpsie	i\n\t"
#endif

		/* Call main.  */
		"mov	r1, r0\n\t"
		"bl	main\n"
	"4:\n\t"
		"b	4b"
		: /* no output */ : /* no input */ : "memory");
#if defined(__ARM_ARCH_7A__)
  asm volatile ("@ Exception vector table.\n\t"
		".align	2\n"
	"except_vector:\n\t"
		"ldr	pc, L_rst\n\t"
		"ldr	pc, L_rst+4\n\t"
		"ldr	pc, L_rst+8\n\t"
		"ldr	pc, L_rst+12\n\t"
		"ldr	pc, L_rst+16\n\t"
		"ldr	pc, L_rst+20\n\t"
		"ldr	pc, L_rst+24\n\t"
		"ldr	pc, L_rst+28\n"
		"L_rst:");
#endif
}

typedef void (*handler)(void);

#if !defined(__ARM_ARCH_7A__)
handler vector_table[] __attribute__ ((section(".startup.vectors"))) = {
  (handler)&__main_stack_end__,
  entry,
  nmi,		/* nmi */
  hard_fault,		/* hard fault */
  /* 0x10 */
  mem_manage,		/* mem manage */
  bus_fault,		/* bus fault */
  usage_fault,		/* usage fault */
  none,
  /* 0x20 */
  none, none, none,		/* reserved */
  svc,				/* SVCall */
  none,				/* Debug */
  none,				/* reserved */
  preempt,			/* PendSV */
  chx_timer_expired,		/* SysTick */
  /* 0x40 */
  chx_handle_intr /* WWDG */,     chx_handle_intr /* PVD */,
  chx_handle_intr /* TAMPER */,   chx_handle_intr /* RTC */,
  chx_handle_intr /* FLASH */,    chx_handle_intr /* RCC */,
  chx_handle_intr /* EXTI0 */,    chx_handle_intr /* EXTI1 */,
  /* 0x60 */
  chx_handle_intr /* EXTI2 */,    chx_handle_intr /* EXTI3 */,
  chx_handle_intr /* EXTI4 */,    chx_handle_intr /* DMA1 CH1 */,
  chx_handle_intr /* DMA1 CH2 */, chx_handle_intr /* DMA1 CH3 */,
  chx_handle_intr /* DMA1 CH4 */, chx_handle_intr /* DMA1 CH5 */,
  /* 0x80 */
  chx_handle_intr /* DMA1 CH6 */, chx_handle_intr /* DMA1 CH7 */,
  chx_handle_intr /* ADC1_2 */,   chx_handle_intr /* USB HP */,
  /* 0x90 */
  chx_handle_intr /* USB LP */,   chx_handle_intr /* CAN */,
  /* ... and more.  EXT9_5, TIMx, I2C, SPI, USART, EXT15_10 */
  chx_handle_intr,                chx_handle_intr,
  /* 0xA0 */
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  /* 0xc0 */
#if !defined(__ARM_ARCH_6M__)
  /* STM32F0 doesn't have more.  */
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
#endif
#if !defined(STM32F10X_MD)
  /* High-density chips have more; RTCAlarm, USBWakeup, ... , DMA2_Channel4_5 */
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
#endif
};
#else
extern void entry_thumb (void);

handler vector_table[] = {
#if defined(SUNXI_SUN8IW7)
  entry,
#else
  entry_thumb,
#endif
  usage_fault,			/* Undefined instruction */
  svc_handler,
  bus_fault,			/* Prefetch Abort */
  bus_fault,			/* Data Abort */
  usage_fault,			/* Hyp Trap */
  irq_handler,
  nmi,				/* FIQ */
};
#endif
