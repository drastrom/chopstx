#define MAX_CORE 4

#include <stdint.h>
#include <chx_hard_regs.h>
#include "io.h"

DEFINE_HW (GIC_DIST, static, 0x01c81000) {
  volatile uint32_t ICDDCR;
  volatile uint32_t ICDICTR;
  volatile uint32_t ICDIIDR;
};

DEFINE_HW (GIC_DIST_SETEN, static, 0x01c81100) {
  volatile uint32_t ICDISER[32];
};

DEFINE_HW (GIC_DIST_CLREN, static, 0x01c81180) {
  volatile uint32_t ICDICER[32];
};

DEFINE_HW (GIC_DIST_SETPEND, static, 0x01c81200) {
  volatile uint32_t ICDISPR[32];
};

DEFINE_HW (GIC_DIST_CLRPEND, static, 0x01c81280) {
  volatile uint32_t ICDICPR[32];
};

DEFINE_HW (GIC_DIST_SETACT, static, 0x01c81300) {
  volatile uint32_t ICDISAR[32];
};

DEFINE_HW (GIC_DIST_CLRACT, static, 0x01c81380) {
  volatile uint32_t ICDICAR[32];
};

DEFINE_HW (GIC_DIST_PRIO, static, 0x01c81400) {
  volatile uint32_t ICDIPR[256];
};

DEFINE_HW (GIC_DIST_TARGET, static, 0x01c81800) {
  volatile uint32_t ICDIPTR[256];
};


DEFINE_HW (GIC_DIST_CONFIG, static, 0x01c81c00) {
  volatile uint32_t ICDICFR[64];
};

DEFINE_HW (GIC_SGI, static, 0x01c81f00) {
  volatile uint32_t ICDSGIR;
};

DEFINE_HW (GIC_CPU_INTF, static, 0x01c82000) {
  volatile uint32_t CTRL;
  volatile uint32_t PRIO_MASK;
  volatile uint32_t BINARY_POINT;
  volatile uint32_t INTR_ACK;
  volatile uint32_t INTR_END;
  volatile uint32_t RUNNING_PRIO;
  volatile uint32_t HIGHEST_PEND;
  volatile uint32_t AIAR;
  volatile uint32_t AEOI;
  volatile uint32_t AHIGHEST_PEND;
};


void chx_enable_intr (uint8_t irq);

static void
chx_intr_init (void)
{
  uint32_t gic_irqs;
  uint32_t i;

  GIC_DIST->ICDDCR = 0;	 /* Disable interrupts. */

  gic_irqs = ((GIC_DIST->ICDICTR & 0x1f)+1)*32;

  for (i = 0; i < gic_irqs; i += 16)
    /* level-triggered. */
    GIC_DIST_CONFIG->ICDICFR[i>>4] = 0;

  for (i = 0; i < gic_irqs; i += 4)
    /* same priorities: 0xa0.  */
    GIC_DIST_PRIO->ICDIPR[i>>2] = 0xa0a0a0a0;

  for (i = 0; i < gic_irqs; i += 4)
    /* set processor target */
    GIC_DIST_TARGET->ICDIPTR[i>>2] = 0x0f0f0f0f;

  GIC_DIST_CLREN->ICDICER[0] = 0xffff0000;
  GIC_DIST_SETEN->ICDISER[0] = 0x0000ffff;
  for (i=32; i < gic_irqs; i+=32)
    /* disable all external interrupts */
    GIC_DIST_CLREN->ICDICER[i>>5] = 0xffffffff;

  for (i=32; i < gic_irqs; i+=32)
    /* clear all interrupt active state */
    GIC_DIST_CLRACT->ICDICAR[i>>5] = 0xffffffff;

  GIC_DIST->ICDDCR = 1;	 /* Enable interrupts. */

  GIC_CPU_INTF->CTRL = 0;
  GIC_CPU_INTF->PRIO_MASK = 0xf0;
  GIC_CPU_INTF->CTRL = 1;
  asm volatile ("dsb");

  chx_enable_intr (0);		/* preempt SGI */
  chx_enable_intr (50);		/* Timer 0     */
}

void
chx_enable_intr (uint8_t irq)
{
  GIC_DIST_SETEN->ICDISER[irq >> 5] |= 1 << (irq & 0x1f);
  asm volatile ("dsb");
}

void
chx_clr_intr (uint8_t irq)
{
  GIC_DIST_CLRPEND->ICDICPR[irq >> 5] |= 1 << (irq & 0x1f);
  asm volatile ("dsb");
}

void
chx_disable_intr (uint8_t irq)
{
  GIC_DIST_CLREN->ICDICER[irq >> 5] |= 1 << (irq & 0x1f);
  asm volatile ("dsb");
}

extern uint32_t _translation_table[4096];

static const uint32_t translation_table_template[] = {
  /*
   * 0xBBBNAPDC
   *
   *   BBB: section Base address
   *
   * 11c0e: cached section
   *   N: <NS>0<nG><S> : NS=0, nG=0, S=1 : Shareable
   *   A: <AP2><TEX210>: TEX=001 (normal memory)
   *   P: <AP10>0<DOM3>: AP=3 : Full access
   *   D: <DOM210><XN> : DOM=0, XN (execute never)=0
   *   C: <C><B>1<PXN> : C=1, B=1, PXN (privileged XN)=0
   *
   * 10413: strongly-orderd section
   *   N: <NS>0<nG><S> : NS=0, nG=0, S=1 : Shareable
   *   A: <AP2><TEX210>: TEX=000 (strongry ordered)
   *   P: <AP10>0<DOM3>: AP=1 : Access PL1 only
   *   D: <DOM210><XN> : DOM=0, XN (execute never)=1
   *   C: <C><B>1<PXN> : C=0, B=0, PXN (privileged XN)=1
   *
   * TEX C B: 001 1 1 : Outer and Inner WB, WA
   * TEX C B: 000 0 0 : Strongly-ordered
   */
  0x00011c0e, 0x00111c0e, 0x00211c0e, 0x00311c0e,
  0x00411c0e, 0x00511c0e, 0x00611c0e, 0x00711c0e,
  0x00811c0e, 0x00911c0e, 0x00a11c0e, 0x00b11c0e,
  0x00c11c0e, 0x00d11c0e, 0x00e11c0e, 0x00f11c0e,

  /* System I/O control, etc. */
  0x01010413, 0x01110413, 0x01210413, 0x01310413,
  0x01410413, 0x01510413, 0x01610413, 0x01710413,
  0x01810413, 0x01910413, 0x01a10413, 0x01b10413,
  0x01c10413, 0x01d10413, 0x01e10413, 0x01f10413,

  /* Other data will be updated, later.  */
};

void __attribute__ ((naked))
stack_init (void)
{
  uint32_t r;

  /* Enable SMP mode.  */
  asm volatile ("mrc	p15, 0, %0, c1, c0, 1\n\t"
	        "orr	%0, %0, #1 << 6\n\t"
	        "mcr	p15, 0, %0, c1, c0, 1" : "=r" (r));

  asm volatile ("cpsid	i\n\t"
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
		"msr	SP_und, r1\n\t"
		"bx	lr");
}

void
mmu_init (void)
{
  uint32_t i;

  /* Initialize MMU translation table. */
  for (i = 0; i < sizeof (translation_table_template)/ sizeof(uint32_t); i++)
    _translation_table[i] = translation_table_template[i];

  for (i = 0x020; i <= 0x3ff; i++)
    _translation_table[i] = (i << 20) | 0x10413;

  for (i = 0x400; i <= 0x7ff; i++)
    _translation_table[i] = (i << 20) | 0x11c0e;

  for (i = 0x800; i <= 0xfff; i++)
    _translation_table[i] = (i << 20) | 0x10413;

  /*
   * Minimal MMU initialization with a flat translation.
   *
   * See ARM Architecture Reference Manual ARMv7-A edition:
   *	 B3.2 The effects of disabling MMUs on VMSA behavior
   * It says: when disabled,
   *	"This means the access is Non-cacheable."
   *
   * Although not explicitly addressed, exclusive memory access by
   * LDREX/STREX apparently depends on cache system.  So, it's no
   * wonder when it doesn't work for non-cacheable memory.  In fact,
   * we get "Data Abort" exception (of external synchronous abort on
   * LDREX instruction's data access), unless MMU is enabled and the
   * memory is cacheable _and_ it's in SMP mode.
   */
  /* Invalidate I&D TLBs.  */
  asm volatile ("mcr	p15, 0, %0, c8, c7, 0" : : "r" (0));
#if 0
  /* TTBCR: N=7, TTBR0 has 128 entries */
  asm volatile ("mcr	p15, 0, %0, c2, c0, 2" : : "r" (0x0007));
#else
  asm volatile ("mcr	p15, 0, %0, c2, c0, 2" : : "r" (0x0000));
#endif
  /* TTBR0 */
  asm volatile ("mcr	p15, 0, %0, c2, c0, 0" : : "r" (_translation_table));
  /* DACR domain0=3, domain1=1 rests= 0 */
  asm volatile ("mcr	p15, 0, %0, c3, c0, 0" : : "r" ((1<<2)|(3<<0)));
  /* SCTLR.TE(bit30)=1, .C(bit2)=..A(bit1)=0 .I(bit12)=.M(bit0)=1 */
  asm volatile ("mcr	p15, 0, %0, c1, c0, 0" : : "r" (0x40c51879));
  asm volatile ("dsb");
}

static void
cache_init (void)
{
  uint32_t r;
  /*
   * Set SCTLR.
   * I (bit12) = 1 for I-cache
   * C (bit2) =  1 for D/U-cache
   */
  asm volatile ("mrc	p15, 0, %0, c1, c0, 0\n\t"
		"orr	%0, %0, 0x4\n\t"
		"mcr	p15, 0, %0, c1, c0, 0\n\t"
		"dsb"
		: "=r" (r));
}

void
gpio_init (void)
{
}

void
irq_init (void)
{
  chx_intr_init ();
}

extern void set_led (uint32_t on_off);
extern void set_led_red (uint32_t on_off);

uint32_t
chx_irq (void)
{
  extern void chx_handle_intr (uint32_t irq_num);
  uint32_t id;

  asm volatile ("dmb");

  id = GIC_CPU_INTF->INTR_ACK;
  if (id >= 1023)
    /* spurious irq*/
    return 0;

  if (id == 50)			/* Timer */
    {
      GIC_CPU_INTF->INTR_END = id; /* OK??? */
      asm volatile ("dsb");
      return 1;
    }

  if (id <= 15)			/* SGI: preemption */
    {
      GIC_CPU_INTF->INTR_END = id; /* OK??? */
      asm volatile ("dsb");
      return 2;
    }

  chx_handle_intr (id);
  GIC_CPU_INTF->INTR_END = id;
  asm volatile ("dsb");
  return 0;
}
	 

void
sys_request_preemption (uint32_t id)
{
  GIC_SGI->ICDSGIR = (1 << (16 + id));
  asm volatile ("dsb");
}

DEFINE_HW (CPUS_RESET, static, 0x01f01c00) {
  volatile uint32_t CTRL;
};

DEFINE_HW (CPU_GATING, static, 0x01f01d44) {
  volatile uint32_t REG;
};

DEFINE_HW (GENERAL_CONTROL, static, 0x01f01d84) {
  volatile uint32_t REG;
};

DEFINE_HW (PLL_UNDOC, static, 0x01c20030) { /* Undocumented.  What? */
  volatile uint32_t CTRL;
};

#if 1
#define CCI_BASE 0x01790000
#else
#define CCI_BASE 0x01c90000
#endif
#define SNOOP_CTRL_ADDR   (CCI_BASE+0x0000)
#define SNOOP_CONFIG_ADDR (CCI_BASE+0x4000)

DEFINE_HW (SNOOP_CTRL, static, SNOOP_CTRL_ADDR) {
  volatile uint32_t OVERRIDE;
  volatile uint32_t SPEC;
  volatile uint32_t SECURE_ACCESS;
  volatile uint32_t STATUS;
};

DEFINE_HW (SNOOP_CONFIG, static, SNOOP_CONFIG_ADDR) {
  volatile uint32_t REG;
};

struct cpu_ctrl {
  volatile uint32_t reset;
  volatile uint32_t ctrl;
  volatile uint32_t status;
  volatile uint32_t dummy[13];
};

DEFINE_HW (CPU_CTRL, static, 0x01f01c40) {
  struct cpu_ctrl cpu[4];
};

DEFINE_HW (CPUCFG_P, static, 0x01f01da4) {
  volatile uint32_t REG0;
};

DEFINE_HW (CPUX_PWR, static, 0x01f01540) {
  volatile uint32_t clamp[4];
};

DEFINE_HW (CPU_PWROFF, static, 0x01f01500) {
  volatile uint32_t REG;
};

static void
enable_cpu (int id)
{
  /* Step 1: Assert core reset, while
             L1_RST_DISABLE hold low (L1 cache is reset by hardware).  */
  GENERAL_CONTROL->REG &= ~(1<<id);
  CPU_CTRL->cpu[id].reset = 0x00;
  delay (500);

  /* Step2: Release power clamp */
  CPUX_PWR->clamp[id] = 0xfe;
  delay (1000);
  CPUX_PWR->clamp[id] = 0xf8;
  delay (500);
  CPUX_PWR->clamp[id] = 0xe0;
  delay (500);
  CPUX_PWR->clamp[id] = 0x80;
  delay (500);
  CPUX_PWR->clamp[id] = 0x00;
  delay (1000);
  while (CPUX_PWR->clamp[id] != 0x00)
    ;

  /* Step 3: Clear power-off gating. */
  CPU_PWROFF->REG &= ~(1<<id);
  delay (1000);

  /* Step 4: De-assert core reset.  */
  CPU_CTRL->cpu[id].reset = 0x03;
}


static void __attribute__ ((naked))
entry_core (void)
{
  stack_init ();
  mmu_init ();
  cache_init ();
  asm volatile ("b	preempt");
}


static inline int
core_id (void)
{
  int i;
  asm ("mrc	p15, 0, %0, c0, c0, 5\n" : "=r" (i));
  return i & 3;
}


void
chx_secondary_init (void)
{
  int id, im = core_id ();

  CPUCFG_P->REG0 = (uint32_t)entry_core;
  asm volatile ("dsb");

  for (id = 0; id < MAX_CORE; id++)
    if (id != im)
      enable_cpu (id);

  cache_init ();

#if 0
  CPUS_RESET->CTRL |= 1;	/* De-assert CPUS reset.  */

  CPU_GATING->REG |= 0x10;	/* Make sure enabling L2 clock.  */
  GENERAL_CONTROL->REG; /* L2 reset???*/
#endif

#if 0
  /* How we can enable coherency of cache ? */
  {
     uint32_t a;

     /* Enable CCI clock???  */
     PLL_UNDOC->CTRL |= 0x80000000;

     delay (100);
     for (a = 0x00100000; a < 0x01000000; a += 0x00100000)
       {
	 uint32_t b;

	 asm ("ldr	%0, [%1]" : "=r" (b) : "r" (a+0x90100));
	 if (b == 0x2000)
	   {
	     asm volatile ("movs	r3, #0\n\t"
			   "str	%0, [r3]\n\t" : : "r" (a) : "r3");
	     asm volatile ("movs	r3, #0x20\n\t"
			   "str	%0, [r3]\n\t" : : "r" (b) : "r3");
	     break;
	   }
       }
     for (a = 0x01400000; a < 0x3ff00000; a += 0x00100000)
       {
	 uint32_t b;

	 asm ("ldr	%0, [%1]" : "=r" (b) : "r" (a+0x90100));
	 if (b == 0x2000)
	   {
	     asm volatile ("movs	r3, #0\n\t"
			   "str	%0, [r3]\n\t" : : "r" (a) : "r3");
	     asm volatile ("movs	r3, #0x20\n\t"
			   "str	%0, [r3]\n\t" : : "r" (b) : "r3");
	     break;
	   }
       }
     for (a = 0xc0000000; a < 0xfff00000; a += 0x00100000)
       {
	 uint32_t b;

	 asm ("ldr	%0, [%1]" : "=r" (b) : "r" (a+0x90100));
	 if (b == 0x2000)
	   {
	     asm volatile ("movs	r3, #0\n\t"
			   "str	%0, [r3]\n\t" : : "r" (a) : "r3");
	     asm volatile ("movs	r3, #0x20\n\t"
			   "str	%0, [r3]\n\t" : : "r" (b) : "r3");
	     break;
	   }
       }

     /* Enable CCI function. */
     SNOOP_CTRL->SECURE_ACCESS |= 1;
     SNOOP_CONFIG->REG |= 3;
     while ((SNOOP_CTRL->STATUS & 1))
       ;
     a = SNOOP_CONFIG->REG;
   }
#endif
}
