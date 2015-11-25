#include <stdint.h>
#include <chx_hard_regs.h>
#include "io.h"

DEFINE_HW (DRAMCOM, static, 0x01c62000) {
  volatile uint32_t cr;
  uint32_t res0[3];
  volatile uint32_t mcr[16][2];
  volatile uint32_t bwcr;
  volatile uint32_t maer;
  volatile uint32_t mapr;
  volatile uint32_t mcgcr;
  volatile uint32_t cpu_bwcr;
  volatile uint32_t gpu_bwcr;
  volatile uint32_t ve_bwcr;
  volatile uint32_t disp_bwcr;
  volatile uint32_t other_bwcr;
  volatile uint32_t total_bwcr;
  uint32_t res1[2];
  volatile uint32_t swonr;
  volatile uint32_t swoffr;
  uint32_t res2[2];
  volatile uint32_t cccr;
  uint32_t res3[459];
  volatile uint32_t protect;
};

DEFINE_HW (DRAMCTL0, static, 0x01c63000) {
  volatile uint32_t pir;
  volatile uint32_t pwrctl;
  volatile uint32_t mrctrl;
  volatile uint32_t clken;
  volatile uint32_t pgsr[2];
  volatile uint32_t statr;
  uint32_t res1[5];
  volatile uint32_t mr[4];
  volatile uint32_t pllgcr;
  volatile uint32_t ptr[5];
  volatile uint32_t dramtmg[9];
  volatile uint32_t odtcfg;
  volatile uint32_t pitmg[2];
  uint32_t res2[1];
  volatile uint32_t rfshctl0;
  volatile uint32_t rfshtmg;
  volatile uint32_t rfshctl1;
  volatile uint32_t pwrtmg;
  uint32_t res3[8];
  volatile uint32_t dqsgmr;
  volatile uint32_t dtcr;
  volatile uint32_t dtar[4];
  volatile uint32_t dtdr[2];
  volatile uint32_t dtmr[2];
  volatile uint32_t dtbmr;
  volatile uint32_t catr[2];
  volatile uint32_t dtedr[2];
  uint32_t res4[2];
  volatile uint32_t pgcr[4];
  volatile uint32_t iovcr[2];
  volatile uint32_t dqsdr;
  volatile uint32_t dxccr;
  volatile uint32_t odtmap;
  volatile uint32_t zqctl[2];
  uint32_t res6[5];
  volatile uint32_t zqcr;
  volatile uint32_t zqsr;
  volatile uint32_t zqdr[3];
  uint32_t res7[27];
  volatile uint32_t sched;
  volatile uint32_t perfhpr[2];
  volatile uint32_t perflpr[2];
  volatile uint32_t perfwr[2];
  uint32_t res8[11];
  volatile uint32_t aciocr;
  uint32_t res9[61];
  struct {
    volatile uint32_t mdlr;
    volatile uint32_t lcdlr[3];
    volatile uint32_t iocr[11];
    volatile uint32_t bdlr6;
    volatile uint32_t gtr;
    volatile uint32_t gcr;
    volatile uint32_t gsr[3];
    uint32_t res0[11];
  } datx[4];
  uint32_t res10[226];
  volatile uint32_t upd2;
};


static void
delay_us (uint32_t usec)
{
  delay (usec * 451);
}

struct dram_para {
  int bus_width;
  int dual_rank;
  int page_size_order;
  int row_bits;
};

static const uint8_t bin_to_mgray[32] = {
  0x00, 0x01, 0x02, 0x03, 0x06, 0x07, 0x04, 0x05,
  0x0c, 0x0d, 0x0e, 0x0f, 0x0a, 0x0b, 0x08, 0x09,
  0x18, 0x19, 0x1a, 0x1b, 0x1e, 0x1f, 0x1c, 0x1d,
  0x14, 0x15, 0x16, 0x17, 0x12, 0x13, 0x10, 0x11,
};

static const uint8_t mgray_to_bin[32] = {
  0x00, 0x01, 0x02, 0x03, 0x06, 0x07, 0x04, 0x05,
  0x0e, 0x0f, 0x0c, 0x0d, 0x08, 0x09, 0x0a, 0x0b,
  0x1e, 0x1f, 0x1c, 0x1d, 0x18, 0x19, 0x1a, 0x1b,
  0x10, 0x11, 0x12, 0x13, 0x16, 0x17, 0x14, 0x15,
};

#define DRAM_CLK 672
static int
ns_to_tick (int nsec)
{
  return ((DRAM_CLK/2) * nsec + 999) / 1000;
}


#define PTR3_TDINIT1(x)		((x) << 20)
#define PTR3_TDINIT0(x)		((x) <<  0)

#define PTR4_TDINIT3(x)		((x) << 20)
#define PTR4_TDINIT2(x)		((x) <<  0)

#define DRAMTMG0_TWTP(x)	((x) << 24)
#define DRAMTMG0_TFAW(x)	((x) << 16)
#define DRAMTMG0_TRAS_MAX(x)	((x) <<  8)
#define DRAMTMG0_TRAS(x)	((x) <<  0)

#define DRAMTMG1_TXP(x)		((x) << 16)
#define DRAMTMG1_TRTP(x)	((x) <<  8)
#define DRAMTMG1_TRC(x)		((x) <<  0)

#define DRAMTMG2_TCWL(x)	((x) << 24)
#define DRAMTMG2_TCL(x)		((x) << 16)
#define DRAMTMG2_TRD2WR(x)	((x) <<  8)
#define DRAMTMG2_TWR2RD(x)	((x) <<  0)

#define DRAMTMG3_TMRW(x)	((x) << 16)
#define DRAMTMG3_TMRD(x)	((x) << 12)
#define DRAMTMG3_TMOD(x)	((x) <<  0)

#define DRAMTMG4_TRCD(x)	((x) << 24)
#define DRAMTMG4_TCCD(x)	((x) << 16)
#define DRAMTMG4_TRRD(x)	((x) <<  8)
#define DRAMTMG4_TRP(x)		((x) <<  0)

#define DRAMTMG5_TCKSRX(x)	((x) << 24)
#define DRAMTMG5_TCKSRE(x)	((x) << 16)
#define DRAMTMG5_TCKESR(x)	((x) <<  8)
#define DRAMTMG5_TCKE(x)	((x) <<  0)

#define RFSHTMG_TREFI(x)	((x) << 16)
#define RFSHTMG_TRFC(x)		((x) <<  0)

#define max(a,b) (((a) > (b)) ? (a) : (b))

static void
set_timing (void)
{
  const int tcwl = 4; /* CWL 8 */
  const int twr  = max (ns_to_tick (15), 3);
  const int twtr = max (ns_to_tick (8), 4);

  /* Set mode register */
  DRAMCTL0->mr[0] = 0x1c70;  /* CL=11, WR=12 */
  DRAMCTL0->mr[1] = 0x40;
  DRAMCTL0->mr[2] = 0x18;    /* CWL=8 */
  DRAMCTL0->mr[3] = 0x0;

  /* Set DRAM timing */
  const int twtp	= tcwl + 2 + twr;  /* WL + BL / 2 + tWR */
  const int tfaw = ns_to_tick (50);
  const int trasmax= 24;
  const int tras = ns_to_tick (38);
  DRAMCTL0->dramtmg[0] = DRAMTMG0_TWTP (twtp) | DRAMTMG0_TFAW (tfaw)
                       | DRAMTMG0_TRAS_MAX (trasmax) | DRAMTMG0_TRAS (tras);

  const int txp  = max (ns_to_tick (8), 3);
  const int trtp = max (ns_to_tick (8), 4);
  const int trc  = ns_to_tick (53);
  DRAMCTL0->dramtmg[1] = DRAMTMG1_TXP (txp) | DRAMTMG1_TRTP (trtp)
                       | DRAMTMG1_TRC (trc);

  const int tcl	= 6; /* CL 12 */
  const int trd2wr	= tcl + 2 + 1 - tcwl; 	/* RL + BL / 2 + 2 - WL */
  const int twr2rd	= tcwl + 2 + twtr; 	/* WL + BL / 2 + tWTR */
  DRAMCTL0->dramtmg[2] = DRAMTMG2_TCWL (tcwl) | DRAMTMG2_TCL (tcl)
                       | DRAMTMG2_TRD2WR (trd2wr) | DRAMTMG2_TWR2RD (twr2rd);

  const int tmrw   = 0;
  const int tmrd   = 4;
  const int tmod   = 12;
  DRAMCTL0->dramtmg[3] = DRAMTMG3_TMRW (tmrw) | DRAMTMG3_TMRD (tmrd)
                       | DRAMTMG3_TMOD (tmod);

  const int trcd = ns_to_tick (15);
  const int tccd = 2;
  const int trrd = max (ns_to_tick (10), 4);
  const int trp  = ns_to_tick (15);
  DRAMCTL0->dramtmg[4] = DRAMTMG4_TRCD (trcd) | DRAMTMG4_TCCD (tccd)
                       | DRAMTMG4_TRRD (trrd) | DRAMTMG4_TRP (trp);

  const int tcksrx = 5;
  const int tcksre = 5;
  const int tckesr = 4;
  const int tcke   = 3;
  DRAMCTL0->dramtmg[5] = DRAMTMG5_TCKSRX (tcksrx) | DRAMTMG5_TCKSRE (tcksre)
                       | DRAMTMG5_TCKESR (tckesr) | DRAMTMG5_TCKE (tcke);

  /* Set two rank timing */
  DRAMCTL0->dramtmg[8] = (DRAMCTL0->dramtmg[8] & ~0xff00) | 0x6610;

  /* Set PHY interface timing */
  const int t_rdata_en	= 4;
  const int wr_latency	= 2;
  DRAMCTL0->pitmg[0] = 0x200000 | (t_rdata_en << 16)
                     | (1 << 8) | (wr_latency << 0);

  /* Set PHY timing */
  const int tdinit0 = (500 * DRAM_CLK) + 1;		/* 500us */
  const int tdinit1 = (360 * DRAM_CLK) / 1000 + 1;	/* 360ns */
  const int tdinit2 = (200 * DRAM_CLK) + 1;		/* 200us */
  const int tdinit3 = (1 * DRAM_CLK) + 1;		/* 1us */
  DRAMCTL0->ptr[3]= PTR3_TDINIT0 (tdinit0) | PTR3_TDINIT1 (tdinit1);
  DRAMCTL0->ptr[4]= PTR4_TDINIT2 (tdinit2) | PTR4_TDINIT3 (tdinit3);

  /* Set refresh timing */
  const int trefi = ns_to_tick (7800) / 32;
  const int trfc  = ns_to_tick (350);
  DRAMCTL0->rfshtmg = RFSHTMG_TREFI (trefi) | RFSHTMG_TRFC (trfc);
}


static void
set_master_priority (void)
{
  DRAMCOM->bwcr = 0x00010190;
  DRAMCOM->mapr = 0x01;

  DRAMCOM->mcr[0][0] = 0x0200000d;
  DRAMCOM->mcr[0][1] = 0x00800100;
  DRAMCOM->mcr[1][0] = 0x06000009;
  DRAMCOM->mcr[1][1] = 0x01000400;
  DRAMCOM->mcr[2][0] = 0x0200000d;
  DRAMCOM->mcr[2][1] = 0x00600100;
  DRAMCOM->mcr[3][0] = 0x0100000d;
  DRAMCOM->mcr[3][1] = 0x00200080;

  DRAMCOM->mcr[4][0] = 0x07000009;
  DRAMCOM->mcr[4][1] = 0x01000640;
  DRAMCOM->mcr[5][0] = 0x0100000d;
  DRAMCOM->mcr[5][1] = 0x00200080;
  DRAMCOM->mcr[6][0] = 0x01000009;
  DRAMCOM->mcr[6][1] = 0x00400080;
  DRAMCOM->mcr[7][0] = 0x0100000d;
  DRAMCOM->mcr[7][1] = 0x00400080;

  DRAMCOM->mcr[8][0] = 0x0100000d;
  DRAMCOM->mcr[8][1] = 0x00400080;
  DRAMCOM->mcr[9][0] = 0x04000009;
  DRAMCOM->mcr[9][1] = 0x00400100;
  DRAMCOM->mcr[10][0]= 0x2000030d;
  DRAMCOM->mcr[10][1]= 0x04001800;
  DRAMCOM->mcr[11][0]= 0x04000009;
  DRAMCOM->mcr[11][1]= 0x00400120;
}

static void
sys_init (void)
{
  MBUS_CLK->REG0 &= ~0x80000000;
  MBUS_RST->REG &= ~0x80000000;
  BUS_CLK_GATING->REG0 &= ~0x00004000;
  BUS_SOFT_RST->REG0 &= ~0x00004000;
  PLL_DDR->CTRL &= ~0x80000000;
  delay_us (10);

  DRAM_CONFIG->REG &= ~0x80000000;
  delay_us (1000);

  /* CLK = 24000000 * k * n / m */
  PLL_DDR->CTRL = 0x80000000 | (1<<20) | (27 << 8) | (1<<4) | 0;
  /*              EN           UPD        N=28       K=2     M=1  */
  while ((PLL_DDR->CTRL & 0x80000000) == 0)
    ;
  delay_us (50);

  DRAM_CONFIG->REG = (DRAM_CONFIG->REG & ~0x30000f)
                   | 0x010000; /* DIV=1, SRC=PLL5, UPD */
  delay_us (20);
  while ((DRAM_CONFIG->REG & 0x010000) != 0)
    ;

  delay_us (3000);

  BUS_SOFT_RST->REG0   |= 0x4000;
  BUS_CLK_GATING->REG0 |= 0x4000;
  MBUS_RST->REG |= 0x80000000;
  MBUS_CLK->REG0 |= 0x80000000;

  DRAM_CONFIG->REG |= 0x80000000;
  delay_us (10);

  DRAMCTL0->clken = 0xc00e;
  delay_us (500);
}

#define DATX_IOCR_DQ(x)	(x)		/* DQ0-7 IOCR index */
#define DATX_IOCR_DM	(8)		/* DM IOCR index */
#define DATX_IOCR_DQS	(9)		/* DQS IOCR index */
#define DATX_IOCR_DQSN	(10)		/* DQSN IOCR index */

static void
dq_delay (uint32_t delay_read, uint32_t delay_write)
{
  int i, j;
  uint32_t val;

  for (i = 0; i < 4; i++)
    {
      val = (((delay_write >> (i * 4)) & 0xf) << 8)
	  | ((delay_read >> (i * 4)) & 0xf);

      for (j = DATX_IOCR_DQ(0); j <= DATX_IOCR_DM; j++)
	DRAMCTL0->datx[i].iocr[j] |= val;
    }

  DRAMCTL0->pgcr[0] &= ~(1 << 26);

  for (i = 0; i < 4; i++)
    {
      val = (((delay_write >> (16 + i * 4)) & 0xf) << 8)
	  | ((delay_read >> (16 + i * 4)) & 0xf);

      DRAMCTL0->datx[i].iocr[DATX_IOCR_DQS] |= val;
      DRAMCTL0->datx[i].iocr[DATX_IOCR_DQSN]|= val;
    }

  DRAMCTL0->pgcr[0] |= (1 << 26);

  delay_us(1);
}

#define PIR_CLRSR	(0x1 << 27)	/* clear status registers */
#define PIR_QSGATE	(0x1 << 10)	/* Read DQS gate training */
#define PIR_DRAMINIT	(0x1 << 8)	/* DRAM initialization */
#define PIR_DRAMRST	(0x1 << 7)	/* DRAM reset */
#define PIR_PHYRST	(0x1 << 6)	/* PHY reset */
#define PIR_DCAL	(0x1 << 5)	/* DDL calibration */
#define PIR_PLLINIT	(0x1 << 4)	/* PLL initialization */
#define PIR_ZCAL	(0x1 << 1)	/* ZQ calibration */
#define PIR_INIT	(0x1 << 0)	/* PHY initialization trigger */
#define PGSR_INIT_DONE	(0x1 << 0)	/* PHY init done */

static void
phy_init (uint32_t val)
{
  DRAMCTL0->pir = val | PIR_INIT;
  while ((DRAMCTL0->pgsr[0] & PGSR_INIT_DONE) == 0)
    ;
}

static void
zq_calibration (void)
{
  int i;

  DRAMCTL0->zqcr = (DRAMCTL0->zqcr & ~0x00ffffff) | (3881979 & 0x00ffffff);
  phy_init (PIR_ZCAL);
  delay_us (10);

  for (i = 0; i < 3; i++)
    {
      uint32_t v, zq;

      v = DRAMCTL0->zqdr[i];

      zq = mgray_to_bin[v & 0x1f];
      v &= ~(0x1f << 8);
      v |= bin_to_mgray[zq] << 8;

      zq = mgray_to_bin[(v >> 16) & 0x1f];
      v &= ~(0x1f << 24);
      v |= bin_to_mgray[zq] << 24;

      DRAMCTL0->zqdr[i] = v;
    }
}

#define CR_BL8		(0x4 << 20)
#define CR_1T		(0x1 << 19)
#define CR_2T		(0x0 << 19)
#define CR_LPDDR3	(0x7 << 16)
#define CR_LPDDR2	(0x6 << 16)
#define CR_DDR3		(0x3 << 16)
#define CR_DDR2		(0x2 << 16)
#define CR_SEQUENTIAL	(0x1 << 15)
#define CR_INTERLEAVED	(0x0 << 15)
#define CR_32BIT	(0x1 << 12)
#define CR_16BIT	(0x0 << 12)
#define CR_BUS_WIDTH(x)	((x) == 32 ? CR_32BIT : CR_16BIT)
#define CR_PAGE_SIZE(x)	((x) - 3) << 8)
#define CR_ROW_BITS(x)	(((x) - 1) << 4)
#define CR_EIGHT_BANKS	(0x1 << 2)
#define CR_FOUR_BANKS	(0x0 << 2)
#define CR_DUAL_RANK	(0x1 << 0)
#define CR_SINGLE_RANK	(0x0 << 0)

static void
set_cr (struct dram_para *para)
{
  DRAMCOM->cr = CR_BL8 | CR_1T | CR_DDR3 | CR_INTERLEAVED | CR_EIGHT_BANKS 
              | CR_BUS_WIDTH (para->bus_width)
              | (para->dual_rank ? CR_DUAL_RANK : CR_SINGLE_RANK)
              | CR_PAGE_SIZE (para->page_size_order)
              | CR_ROW_BITS (para->row_bits);
}


static int
channel_init (struct dram_para *para)
{
  uint32_t i;

  set_cr (para);
  set_timing ();
  set_master_priority ();

  DRAMCTL0->pgcr[0] &= ~0x4000003f;
  DRAMCTL0->pgcr[1] = (DRAMCTL0->pgcr[1] & ~0x01000000)|  0x04000000;

#define MAGIC 0x94be6fa3

  DRAMCOM->protect = MAGIC;
  delay_us (100);
  DRAMCTL0->upd2 = (DRAMCTL0->upd2 & ~0x0fff0000) | 0x00500000;
  DRAMCOM->protect = 0x0;
  delay_us (100);

  /* ODT */
  for (i = 0; i < 4; i++)
    DRAMCTL0->datx[i].gcr &= ~((0x3 << 14)| (0x3 << 12)|
			       (0x3 << 4) | (0x3 << 2) | (0x1 << 1));

  DRAMCTL0->aciocr  |= 0x2;
  DRAMCTL0->pgcr[2] |= (0x3 << 6);
  DRAMCTL0->pgcr[0] &= ~((0x3 << 14) | (0x3 << 12));
  DRAMCTL0->pgcr[2] = (DRAMCTL0->pgcr[2]& ~((0x3 << 10) | (0x3 << 8)))
                    | ((0x1 << 10) | (0x2 << 8));

  /* data training configuration */
  DRAMCTL0->dtcr = (DRAMCTL0->dtcr & ~(0xf << 24))|(0x1 << 24);

  dq_delay (0x00007979, 0x6aaa0000);
  delay_us (50);

  zq_calibration ();

  phy_init (PIR_PLLINIT | PIR_DCAL | PIR_PHYRST | PIR_DRAMRST
	    | PIR_DRAMINIT | PIR_QSGATE);

  /* detect ranks and bus width */
  if ((DRAMCTL0->pgsr[0] & (0xfe << 20)))
    {
      if (((DRAMCTL0->datx[0].gsr[0] >> 24) & 0x2)
	  || ((DRAMCTL0->datx[1].gsr[0] >> 24) & 0x2))
	para->dual_rank = 0;
      else
	para->dual_rank = 1;

      if (((DRAMCTL0->datx[2].gsr[0] >> 24) & 0x1)
	  || ((DRAMCTL0->datx[3].gsr[0] >> 24) & 0x1))
	{
	  DRAMCTL0->datx[2].gcr = 0x0;
	  DRAMCTL0->datx[3].gcr = 0x0;
	  para->bus_width = 16;
	}

      set_cr (para);
      delay_us (20);

      DRAMCTL0->dtcr = (DRAMCTL0->dtcr & ~(0xf << 24))
	             | ((para->dual_rank ? 0x3 : 0x1) << 24);
      phy_init (PIR_QSGATE);
      if ((DRAMCTL0->pgsr[0] & (0xfe << 20)))
	return -1;
    }

  while ((DRAMCTL0->statr & 0x1) == 0)
    ;

  DRAMCTL0->rfshctl0 |= 0x80000000;
  delay_us (10);
  DRAMCTL0->rfshctl0 &= ~0x80000000;
  delay_us (10);

  DRAMCTL0->pgcr[3] =  0x00aa0060;

  DRAMCTL0->zqcr |= 0x80000000; /* power down */

  DRAMCOM->maer = 0xffffffff;
  return 0;
}


DEFINE_HW (DRAM_BASE_ADDR, static, 0x40000000) {
  volatile uint32_t MEM[1024*1024*128];
};

static int
memory_check (uint32_t offset)
{
  DRAM_BASE_ADDR->MEM[0] = 0;
  DRAM_BASE_ADDR->MEM[offset>>2] = 0xaa55aa55;

  return DRAM_BASE_ADDR->MEM[0] == DRAM_BASE_ADDR->MEM[offset];
}

static void
auto_detect_dram_size (struct dram_para *para)
{
  para->page_size_order = 9;
  para->row_bits = 16;
  set_cr (para);

  for (para->row_bits = 11; para->row_bits < 16; para->row_bits++)
    if (memory_check ((1 << (para->row_bits + para->page_size_order + 3))))
      break;

  para->page_size_order = 13;
  set_cr (para);

  for (para->page_size_order = 9; para->page_size_order <= 13;
       para->page_size_order++)
    if (memory_check (para->page_size_order))
      break;
}

extern void set_led_pwr (uint32_t on_off);

uint32_t
boot_dram (void)
{
  struct dram_para para = {
    .bus_width = 32,
    .dual_rank = 0,
    .page_size_order = 12,
    .row_bits = 14,
  };


  /* Return DRAM size */
  {
    asm volatile ("movs	r3, #0x0004b000\n\t"
		  "str	%0, [r3], #4\n\t"
		  "str	%0, [r3], #4\n\t"
		  "str	%0, [r3]"
		  :
		  : "r" (0), "r" (0), "r" (0)
		  : "r3");
  }

  sys_init ();
  if (channel_init (&para) < 0)
    return 0;

  set_led_pwr (1);

  if (para.dual_rank)
    DRAMCTL0->odtmap = 0x00000303;
  else
    DRAMCTL0->odtmap = 0x00000201;
  delay_us (1);

  DRAMCTL0->odtcfg = 0x0c000400;

  DRAMCOM->cccr |= (1 << 31);
  delay_us (10);

  auto_detect_dram_size (&para);
  set_cr (&para);

  /* Return DRAM size */
  {
    asm volatile ("movs	r3, #0x0004b000\n\t"
		  "str	%0, [r3], #4\n\t"
		  "str	%0, [r3], #4\n\t"
		  "str	%0, [r3]"
		  :
		  : "r" (para.row_bits), "r" (para.page_size_order),
		    "r" (para.dual_rank)
		  : "r3");
  }

  return (1 << (para.row_bits + para.page_size_order + para.dual_rank + 3));
}
