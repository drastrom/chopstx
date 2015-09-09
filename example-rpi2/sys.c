#include <stdint.h>
#include <stdlib.h>
#include "board.h"

/* 2 IRQs are used for system use:
   BCM2836_COREn_TIMER_IRQ_CNTL, BCM2836_TIMER1_ENABLE_IRQ: systick
   BCM2836_COREn_MBOX_IRQ_CNTL, BCM2836_MBOX0_ENABLE_IRQ: pendsv
 */

extern void chx_handle_intr (uint32_t irq_num);

#define MAX_CORE 4

static inline int
core_id (void)
{
  int i;
  asm ("mrc	p15, 0, %0, c0, c0, 5\n" : "=r" (i));
  return i & 3;
}

static inline void
clr_mbox0 (void)
{
  int i = core_id ();
  *(volatile uint32_t *) (BCM2709_CORE0_MBOX0_RDCLR + 0x10*i) = 1;
  asm volatile ("dsb");
}

static inline void
clr_tick (void)
{
  *(volatile uint32_t *) (BCM2709_ARM_TIMER_CTL) = 0x003E0000;
  *(volatile uint32_t *) (BCM2709_ARM_TIMER_CLI) = 0;
  asm volatile ("dsb");
}

int
irq (void)
{
  uint32_t irqs, lirqs;
  uint32_t i;

#define CHK_INTR(q, num) \
  if ((q) & (1 << ((num) & 0x1f))) \
    { chx_handle_intr ((num)); return 0; }
#define CHK_TICK(q, num) \
  if ((q) & (1 << ((num) & 0x1f))) \
    { clr_tick (); /* call chx_timer_expired */ return 1; }
#define CHK_PendSV(q, num) \
  if ((q) & (1 << ((num) & 0x1f))) \
    { clr_mbox0 (); /* go preempt */ return 2; }

  asm volatile ("dmb");

  /* Interrupts except arm timer and MBOX0. */
  irqs = *(volatile uint32_t *) BCM2709_IRQ_PENDING1;
  /* CHK_INTR (irqs, ...) Currently only USB is handled for IRQ1.  */
  CHK_INTR (irqs, BCM2709_VIC_USB);

  irqs = *(volatile uint32_t *) BCM2709_IRQ_PENDING2;
  CHK_INTR (irqs, BCM2709_VIC_GPIO0);
  CHK_INTR (irqs, BCM2709_VIC_GPIO1);
  CHK_INTR (irqs, BCM2709_VIC_GPIO2);
  CHK_INTR (irqs, BCM2709_VIC_GPIO3);
  CHK_INTR (irqs, BCM2709_VIC_I2C);
  CHK_INTR (irqs, BCM2709_VIC_SPI);
  CHK_INTR (irqs, BCM2709_VIC_I2SPCM);
  CHK_INTR (irqs, BCM2709_VIC_SDIO);
  CHK_INTR (irqs, BCM2709_VIC_UART);

  /* Get MPIDR for core number.  */
  i = core_id ();
  lirqs = *(volatile uint32_t *) (BCM2709_CORE0_IRQ_SOURCE + 4*i);
  CHK_INTR (lirqs, BCM2709_LOC_INT_TIMER0);
  CHK_INTR (lirqs, BCM2709_LOC_INT_TIMER1);
  CHK_INTR (lirqs, BCM2709_LOC_INT_TIMER2);
  CHK_INTR (lirqs, BCM2709_LOC_INT_TIMER3);
  CHK_INTR (lirqs, BCM2709_LOC_INT_MBOX1);
  CHK_INTR (lirqs, BCM2709_LOC_INT_MBOX2);
  CHK_INTR (lirqs, BCM2709_LOC_INT_MBOX3);

  /* Check SysTick.  */
  irqs = *(volatile uint32_t *) BCM2709_IRQ_PENDING0;
  *(volatile uint32_t *)0x500C = irqs; 
  CHK_TICK (irqs, BCM2709_ARM_TIMER0);

  /* Check MBOX0.  */
  CHK_PendSV (lirqs, BCM2709_LOC_INT_MBOX0);

  /* Bad intr.  */
  return 0;
}

void
sys_clr_intr (uint8_t irq_num __attribute__ ((unused)))
{
}

void
sys_enable_intr (uint8_t irq_num)
{
  if (irq_num < 32)
    {
      *(volatile uint32_t *) BCM2709_IRQ_ENABLE1 = (1 << irq_num);
    }
  else if (irq_num < 32*2)
    {
      *(volatile uint32_t *) BCM2709_IRQ_ENABLE2 = (1 << (irq_num & 0x1f));
    }
  else if (irq_num < 32*3)
    {
      *(volatile uint32_t *) BCM2709_IRQ_ENABLE3 = (1 << (irq_num & 0x1f));
    }
  else if (irq_num >= BCM2709_LOC_INT_TIMER0
	   && irq_num <= BCM2709_LOC_INT_TIMER3)
    {
      int i;
      for (i = 0; i < MAX_CORE; i++)
	{
	  uint32_t val;
	  val = *(volatile uint32_t *) (BCM2709_CORE0_TIMER_IRQ_CNTL + 4*i);
	  val |= (1 << (irq_num - BCM2709_LOC_INT_TIMER0));
	  *(volatile uint32_t *) (BCM2709_CORE0_TIMER_IRQ_CNTL + 4*i) = val;
	}
    }
  else if (irq_num >= BCM2709_LOC_INT_MBOX0
	   && irq_num <= BCM2709_LOC_INT_MBOX3)
    {
      int i;
      for (i = 0; i < MAX_CORE; i++)
	{
	  uint32_t val;
	  val = *(volatile uint32_t *) (BCM2709_CORE0_MBOX_IRQ_CNTL + 4*i);
	  val |= (1 << (irq_num - BCM2709_LOC_INT_MBOX0));
	  *(volatile uint32_t *) (BCM2709_CORE0_MBOX_IRQ_CNTL + 4*i) = val;
	}
    }
  asm volatile ("dsb");
}

void
sys_disable_intr (uint8_t irq_num)
{
  if (irq_num < 32)
    {
      *(volatile uint32_t *) BCM2709_IRQ_DISABLE1 = (1 << irq_num);
    }
  else if (irq_num < 32*2)
    {
      *(volatile uint32_t *) BCM2709_IRQ_DISABLE2 = (1 << (irq_num & 0x1f));
    }
  else if (irq_num < 32*3)
    {
      *(volatile uint32_t *) BCM2709_IRQ_DISABLE3 = (1 << (irq_num & 0x1f));
    }
  else if (irq_num >= BCM2709_LOC_INT_TIMER0
	   && irq_num <= BCM2709_LOC_INT_TIMER3)
    {
      int i;
      for (i = 0; i < MAX_CORE; i++)
	{
	  uint32_t val;
	  val = *(volatile uint32_t *) (BCM2709_CORE0_TIMER_IRQ_CNTL + 4*i);
	  val &= ~(1 << (irq_num - BCM2709_LOC_INT_TIMER0));
	  *(volatile uint32_t *) (BCM2709_CORE0_TIMER_IRQ_CNTL + 4*i) = val;
	}
    }
  else if (irq_num >= BCM2709_LOC_INT_MBOX0
	   && irq_num <= BCM2709_LOC_INT_MBOX3)
    {
      int i;
      for (i = 0; i < MAX_CORE; i++)
	{
	  uint32_t val;
	  val = *(volatile uint32_t *) (BCM2709_CORE0_MBOX_IRQ_CNTL + 4*i);
	  val &= ~(1 << (irq_num - BCM2709_LOC_INT_MBOX0));
	  *(volatile uint32_t *) (BCM2709_CORE0_MBOX_IRQ_CNTL + 4*i) = val;
	}
    }
  asm volatile ("dsb");
}

void
sys_set_intr_prio (uint8_t n __attribute__ ((unused)))
{
}

void
sys_request_preemption (int id)
{
  *(volatile uint32_t *) (BCM2709_CORE0_MBOX0_SET + 0x10*id) = 1;
  asm volatile ("dsb");
}

#if 0
void
sys_cpu_sched_lock (void)
{
  uint32_t val;
  int id = core_id ();
  val = *(volatile uint32_t *) (BCM2709_CORE0_MBOX_IRQ_CNTL + 4*id);
  val &= ~(1 << 0);
  *(volatile uint32_t *) (BCM2709_CORE0_MBOX_IRQ_CNTL + 4*id) = val;
  *(volatile uint32_t *) BCM2709_IRQ_DISABLE3 = (1 << (irq_num & 0x1f));
}

void
sys_cpu_sched_unlock (void)
{
  uint32_t val;
  int id = core_id ();
  val = *(volatile uint32_t *) (BCM2709_CORE0_MBOX_IRQ_CNTL + 4*id);
  val |= (1 << 0);
  *(volatile uint32_t *) (BCM2709_CORE0_MBOX_IRQ_CNTL + 4*id) = val;
  *(volatile uint32_t *) BCM2709_IRQ_ENABLE3 = (1 << (irq_num & 0x1f));
}
#endif

void
irq_init (void)
{
  uint32_t val;
  /* Route GPU interrupt to IRQ of this core.  */
  val = *(volatile uint32_t *)BCM2709_GPU_IRQ_CNTL;
  val = (val & ~0xf) | core_id ();
  *(volatile uint32_t *)BCM2709_GPU_IRQ_CNTL = val;
  /* Enable systick and soft irq.  */
  sys_enable_intr (BCM2709_ARM_TIMER0);
  sys_enable_intr (BCM2709_LOC_INT_MBOX0);
}

void
clock_init (void)
{
  *(volatile uint32_t *) BCM2709_ARM_TIMER_DIV = 0xf9;
  asm volatile ("dsb");
}

/* GPIO pin function select:
   000 input
   001 output
   100 alt. function 0
   101 alt. function 1
   110 alt. function 2
   111 alt. function 3
   011 alt. function 4
   010 alt. function 5

   SEL0: bit31-30 reserved, pin9, pin8, ..., pin0
   SEL1: bit31-30 reserved, pin19, pin18, ..., pin10
   SEL2: bit31-30 reserved, pin29, pin28, ..., pin20
   SEL3: bit31-30 reserved, pin39, pin38, ..., pin30
   SEL4: bit31-30 reserved, pin49, pin48, ..., pin40
   SEL5: bit31-12 reserved, pin53, pin52, pin51, pin50
*/

void
gpio_init (void)
{
  /* Only set ACT led (pin 47) pin to output and pin 40-49 to input.  */
  volatile uint32_t *sel40 = (volatile uint32_t *) BCM2709_GPIO_FSEL4;
  *sel40 = (1 << (3 * (47 - 40)));
}

void
set_led (int on)
{
  volatile uint32_t *set32 = (volatile uint32_t *) BCM2709_GPIO_SET1;
  volatile uint32_t *clr32 = (volatile uint32_t *) BCM2709_GPIO_CLR1;
  if (on)
    *set32 = 1 << (47 - 32);
  else
    *clr32 = 1 << (47 - 32);
}

#if 0
void
jtag_init (void)
{
  volatile uint32_t *pud = (volatile uint32_t *) BCM2709_GPIO_PUD;
  volatile uint32_t *pudclk0 = (volatile uint32_t *) BCM2709_GPIO_PUDCLK0;
  volatile uint32_t *fsel0 = (volatile uint32_t *) BCM2709_GPIO_FSEL0;
  volatile uint32_t *fsel2 = (volatile uint32_t *) BCM2709_GPIO_FSEL2;

  uint32_t val;
  int i;
  
  *pud = 0;
  for (i = 0; i < 150; i++)
    (void) 0;
  *pudclk0 = (1<<4)|(1<<22)|(1<<24)|(1<<25)|(1<<27);
  for (i = 0; i < 150; i++)
    (void) 0;
  *pudclk0 = 0;

  /* GPIO pin4 alt5 */
  val = *fsel0;
  *fsel0 = (val & 0xffff8fff) | 0x2000;

  /* GPIO pin22, 24, 25, 27 alt5 */
  val = *fsel2;
  *fsel2 = (val & 0xff1c0e3f) | 0x61b0c0;
}
#endif

#if defined(__ARM_NEON__)
static void
neon_init (void)
{
  uint32_t val;

  /* Enable vfp/neon.
     CPACR.CP11(bit23-22)=.CP10(bit21-20)=3 FPEXC.EN(bit30)=1 */
  asm volatile ("mrc	p15, 0, %0, c1, c0, 2" : "=r" (val));
  asm volatile ("mcr	p15, 0, %0, c1, c0, 2" : : "r" (val | (0xf << 20)));
  asm volatile ("vmsr	fpexc, %0" : : "r" (1 << 30));
  return;
}
# endif

extern uint8_t __svc_stack_end__;
extern uint8_t __irq_stack_end__;
extern uint8_t __fiq_stack_end__;
extern uint8_t __abt_stack_end__;
extern uint8_t __und_stack_end__;
extern uint8_t __svc_stack_size__;
extern uint8_t __exc_stack_size__;
extern uint32_t default_translation_table[];

static void
mmu_init (void)
{
  uint32_t val;

  /* Minimal MMU initialization with a flat translation.  ??? It
     looks that we get external abort errors with LDREX/STREX
     unless MMU is enabled with SCTLR.M=1 and the corresponding
     descriptor for the memory operand of LDREX/STREX sets C bit.  */
  /* Invalidate I&D TLBs.  */
  asm volatile ("mcr	p15, 0, %0, c8, c7, 0" : : "r" (0));
  /* TTBCR */
  asm volatile ("mcr	p15, 0, %0, c2, c0, 2" : : "r" (0));
  /* TTBR0 */
  val = (uint32_t) default_translation_table;
  asm volatile ("mcr	p15, 0, %0, c2, c0, 0" : : "r" (val));
  /* DACR domain0=3, domain1=1 rests= 0 */
  val = (1<<2)|(3<<0);
  asm volatile ("mcr	p15, 0, %0, c3, c0, 0" : : "r" (val));
  /* SCTLR.TE(bit30)=1, .I(bit12)=.C(bit2)=.A(bit1)=0 .M(bit0)=1 */
  asm volatile ("mcr	p15, 0, %0, c1, c0, 0" : : "r" (0x40c50879));
  asm volatile ("dsb");
  return;
}

__attribute__ ((naked,section(".text.startup.0")))
void entry_arm (void)
{
  asm volatile (".arm\n\t"
		"ADR	r0, entry_thumb\n\t"
		"orr	r0, #1\n\t"
		"bx	r0\n\t"
		".thumb\n\t"
		".global entry_thumb\n"
	"entry_thumb:\n\t"
		"cpsid	i\n\t"
		"mrc	p15, 0, r2, c0, c0, 5\n\t"
		"and	r2, r2, #3\n\t"
		"ldr	r3, =__svc_stack_size__\n\t"
		"muls	r3, r2\n\t"
		"ldr	r1, =__svc_stack_end__\n\t"
		"subs	r1, r1, r3\n\t"
		"msr	SP_svc, r1\n\t"
		"ldr	r1, =__irq_stack_end__\n\t"
		"subs	r1, r1, r3\n\t"
		"msr	SP_irq, r1\n\t"
		"ldr	r3, =__exc_stack_size__\n\t"
		"muls	r3, r2\n\t"
		"ldr	r1, =__fiq_stack_end__\n\t"
		"subs	r1, r1, r3\n\t"
		"msr	SP_fiq, r1\n\t"
		"ldr	r1, =__abt_stack_end__\n\t"
		"subs	r1, r1, r3\n\t"
		"msr	SP_abt, r1\n\t"
		"ldr	r1, =__und_stack_end__\n\t"
		"subs	r1, r1, r3\n\t"
		"msr	SP_und, r1");

# if defined(__ARM_NEON__)
  neon_init ();
# endif
  mmu_init ();
  asm volatile ("b	entry");
}

/* Wake up another core.  */
static void
wakeup_core (int coreid, void (*func)(void))
{
  volatile uint32_t *set, *rdclr;
  int retry = 20;

  set = (volatile uint32_t *) (BCM2709_CORE0_MBOX3_SET + 0x10*coreid);
  rdclr = (volatile uint32_t *) (BCM2709_CORE0_MBOX3_RDCLR + 0x10*coreid);

  asm volatile ("dsb");
  *set = (uint32_t) (void *) func;
  while (--retry > 0)
    {
      if (*rdclr == 0)
	break;
      __sync_synchronize ();
    }

  return;
}

static void __attribute__ ((naked))
entry_core (void)
{
  asm volatile ("mrc	p15, 0, r2, c0, c0, 5\n\t"
		"and	r2, r2, #3\n\t"
		"ldr	r3, =__svc_stack_size__\n\t"
		"muls	r3, r2\n\t"
		"ldr	r1, =__svc_stack_end__-32\n\t"
		"subs	r1, r1, r3\n\t"
		"msr	SP_svc, r1\n\t"
		"ldr	r1, =__irq_stack_end__\n\t"
		"subs	r1, r1, r3\n\t"
		"msr	SP_irq, r1\n\t"
		"ldr	r3, =__exc_stack_size__\n\t"
		"muls	r3, r2\n\t"
		"ldr	r1, =__fiq_stack_end__\n\t"
		"subs	r1, r1, r3\n\t"
		"msr	SP_fiq, r1\n\t"
		"ldr	r1, =__abt_stack_end__\n\t"
		"subs	r1, r1, r3\n\t"
		"msr	SP_abt, r1\n\t"
		"ldr	r1, =__und_stack_end__\n\t"
		"subs	r1, r1, r3\n\t"
		"msr	SP_und, r1");

# if defined(__ARM_NEON__)
  neon_init ();
# endif
  mmu_init ();
  asm volatile (/* Set SCTLR.I(bit12) and SCTLR.C(bit2) to enable caches.  */
		"mrc	p15, 0, r0, c1, c0, 0\n\t"
		"orr	r0, r0, 0x4000\n\t"
		"orr	r0, r0, 0x4\n\t"
		"mcr	p15, 0, r0, c1, c0, 0\n\t"
		"dsb" : : : "r0");
  asm volatile ("b	preempt");
}

void
sys_secondary_init (void)
{
  int id, im = core_id ();

  for (id = 0; id < MAX_CORE; id++)
    {
      if (id == im)
	continue;
      wakeup_core (id, entry_core);
    }
}
