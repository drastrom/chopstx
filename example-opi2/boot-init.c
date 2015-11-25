#include <stdint.h>
#include <chx_hard_regs.h>
#include "io.h"

DEFINE_HW (CCU_SECURITY_SWITCH, static, 0x01c202f0) {
  volatile uint32_t CONFIG;
};

static void
clk_init (void)
{
  uint32_t v;

  CCU_SECURITY_SWITCH->CONFIG = 7;

  /* Configure AXI: OSC24M, AXI_DIV = /2 or /3 */
  CPUX_AXI_CFG->REG |= 0x10001;
  delay (10);

  /* Enable PLL_CPUX: 408MHz */
  PLL_CPUX->CTRL = 0x80001000;
  while ((PLL_CPUX->CTRL & 0x10000000) == 0)
    ;

  /* Configure APB and AXI: APB_DIV=/2, AXI_DIV=/2 */
  v = CPUX_AXI_CFG->REG;
  v &= ~0x0303;
  v |=  0x0101;
  CPUX_AXI_CFG->REG = v;
  delay (10);

  /* Configure AHB1_APB1_CFG_REG
   * SRC=AXI, APB1_CLK=/2, AHB1_PRE=/3, AHB1_CLK=/1
   */
  v = AHB1_APB1_CFG->REG;
  v &= ~0x3330;
  v |=  0x2180;
  AHB1_APB1_CFG->REG = v;

  /* Enable DMA */
  BUS_SOFT_RST->REG0   |= 0x40;	/* De-assert reset of DMA */
  BUS_CLK_GATING->REG0 |= 0x40;

  /* Assert MBUS domain, Enable MBUS */
  MBUS_RST->REG = 0x80000000;
  MBUS_CLK->REG0 = 0x81000001;
  MBUS_CLK->REG0 = 0x81000001;

  /* Enable PERIPH0 PLL */
  PLL_PERIPH0->CTRL |= 0x80000000;

  /* Configure CPUX_AXI_CFG_REG again, selecting source of PLL_CPUX */
  v = CPUX_AXI_CFG->REG;
  v &= ~0x030000;
  v |=  0x020000;
  CPUX_AXI_CFG->REG = v;

  delay (1000);

  asm volatile ("movs	r3, #0" : : : "memory");
  asm volatile ("mcr	15, 0, r3, c7, c10, 5" : : : "memory"); /*DMB*/
  asm volatile ("mcr	15, 0, r3, c7, c5, 4" : : : "memory");  /*ISB*/

  /* Enable GPIO clock */
  BUS_CLK_GATING->REG2 |= 0x20;	/* APB1:  For Port A, C */
  RPRCM_GATING->REG |= 0x01;	/* RPRCM: For Port L    */
}


void
boot_init (void)
{
  uint32_t v;

  clk_init ();

  /* Configure JTAG: PA0-PA3 */
  v = PORTA->CFG0;
  v &= ~0x0000ffff;
  v |=  0x00003333;
  PORTA->CFG0 = v;

  /* Configure PA15 */
  v = PORTA->CFG1;
  v &= ~0xf0000000;
  v |=  0x10000000;
  PORTA->CFG1 = v;

  /* Configure PL10 */
  v = PORTL->CFG1;
  v &= ~0x00000f00;
  v |=  0x00000100;
  PORTL->CFG1 = v;
}
